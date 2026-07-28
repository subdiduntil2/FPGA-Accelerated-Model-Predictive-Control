function [accel_cmd, steer_cmd] = fcs_mpc_v4(x, y, psi, v, ref_x, ref_y, ref_v)
    %#codegen
    % Area/timing-optimised restatement of fcs_mpc_v3. Bit-identical outputs.
    % cost(s,a) = A(s) + B(a) is separable, so 17 kinematic cones + 8 speed
    % terms replace 136 full evaluations, and the 136-deep argmin becomes two
    % independent first-minimum scans (17 and 8).
    V_MAX = int16(640);
    W_ERR = int16(15); W_STEER = int16(2); W_SPEED = int16(2);
    W_OVER = int16(6); OV_MAX = int16(255);
    STEER_OPTS = int16([0, -2, 2, -4, 4, -7, 7,-10,10,-14,14,-18,18,-22,22,-26,26]);
    ACCEL_OPTS = int16([0, 20, 10, 4, -4, -10, -20, -31]);
    q1_a = [0, 3, 6, 9, 12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46];
    q1_b = [49, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88];
    q1_c = [90, 92, 94, 96, 98, 100, 102, 104, 106, 107, 109, 111, 112, 114, 115, 116];
    q1_d = [118, 119, 120, 121, 122, 123, 124, 124, 125, 126, 126, 127, 127, 127, 127, 127];
    Q1 = int16([q1_a, q1_b, q1_c, q1_d]);
    Q2 = Q1(end:-1:1); Q3 = -Q1; Q4 = -Q2;
    SIN_LUT = [Q1, Q2, Q3, Q4]; COS_LUT = [Q2, Q3, Q4, Q1];
    CLAMP_VAL = int16(1000);

    % --- Steer branch: 17 cones (kinematics + LUT + distance + effort) ---
    minA = int16(32767); best_str = int16(0);
    v_red = bitsra(v, 4);
    for s_idx = 1:17
        delta = STEER_OPTS(s_idx);
        dpsi = bitsra(v * delta, 9);
        psi_next = psi + dpsi;
        lut_idx = bitand(psi_next, int16(255)) + int16(1);
        c_val = COS_LUT(lut_idx); s_val = SIN_LUT(lut_idx);
        dx = bitsra(v_red * c_val, 5); dy = bitsra(v_red * s_val, 5);
        err_x = (x + dx) - ref_x; err_y = (y + dy) - ref_y;
        if err_x < 0; abs_x = -err_x; else; abs_x = err_x; end
        if err_y < 0; abs_y = -err_y; else; abs_y = err_y; end
        if abs_x > CLAMP_VAL; abs_x = CLAMP_VAL; end
        if abs_y > CLAMP_VAL; abs_y = CLAMP_VAL; end
        A = (abs_x + abs_y) * W_ERR + abs(delta) * W_STEER;
        if A < minA; minA = A; best_str = delta; end
    end

    % --- Accel branch: 8 speed terms ---
    minB = int16(32767); best_acc = int16(0);
    for a_idx = 1:8
        accel = ACCEL_OPTS(a_idx);
        v_next = v + accel;
        if v_next > V_MAX; v_next = V_MAX; end
        if v_next < 0;     v_next = int16(0); end
        ov = v_next - ref_v;
        if ov < 0;      ov = int16(0); end
        if ov > OV_MAX; ov = OV_MAX;   end
        rw = v_next;
        if rw > ref_v;  rw = ref_v;    end
        B = ov * W_OVER - rw * W_SPEED;
        if B < minB; minB = B; best_acc = accel; end
    end
    accel_cmd = best_acc; steer_cmd = best_str;
end
