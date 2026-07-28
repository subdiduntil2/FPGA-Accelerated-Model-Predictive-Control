function stats = fcs_mpc_v4_tb(opts)
%% fcs_mpc_v4_tb.m -- closed-loop sim for fcs_mpc_v4 (area/timing-optimised, bit-
%% exact to v3). Batch: stats = fcs_mpc_v4_tb(struct('plots',false)).

if nargin < 1; opts = struct(); end
if ~isfield(opts, 'plots'); opts.plots = true; end
if ~isfield(opts, 'write'); opts.write = true; end
if isfield(opts, 'VIDX'); VIDX = opts.VIDX; else; VIDX = 'near'; end

SF_POS  = 64;  SF_PSI  = 41;  SF_V = 64;
Ts      = 0.1; T_SIM   = 80;  time = 0:Ts:T_SIM;
STEER_DECODE = 0.025; V_DRAG = 0.99; LOOK_MIN = 0.8; LOOK_GAIN = 0.25;
v_phys = 0.0; x_phys = 0; y_phys = 0; psi_phys = 0; L_phys = 2.5;

% Speed profile parameters, overridable through opts for tuning sweeps
V_TOP   = 10.0;  % m/s ceiling on straights (V_MAX = 640 units)
A_LAT   = 4.0;   % m/s^2 lateral limit -> corner speed sqrt(A_LAT/kappa); 4.2 also passes
A_BRK   = 3.5;   % m/s^2 braking used in the backward pass
A_ACC   = 3.0;   % m/s^2 acceleration used in the forward pass
if isfield(opts, 'V_TOP'); V_TOP = opts.V_TOP; end
if isfield(opts, 'A_LAT'); A_LAT = opts.A_LAT; end
if isfield(opts, 'A_BRK'); A_BRK = opts.A_BRK; end
if isfield(opts, 'A_ACC'); A_ACC = opts.A_ACC; end

% Reference racetrack (identical to v2)
seg1_x = 0:0.5:60; seg1_y = zeros(size(seg1_x));
th2 = linspace(-pi/2, pi/2, 60);
seg2_x = 60 + 15 * cos(th2); seg2_y = 15 + 15 * sin(th2);
seg3a_x = 60:-0.5:45; seg3a_y = 30 * ones(size(seg3a_x));
seg3b_x = 44.5:-0.5:15.5; phase = (45 - seg3b_x) / 30 * 2 * pi;
seg3b_y = 30 + 4.0 * sin(phase);
seg3c_x = 15:-0.5:0; seg3c_y = 30 * ones(size(seg3c_x));
th4 = linspace(pi/2, 3*pi/2, 60);
seg4_x = 0 + 15 * cos(th4); seg4_y = 15 + 15 * sin(th4);
ref_path_x = [seg1_x, seg2_x(2:end), seg3a_x(2:end), seg3b_x, seg3c_x, seg4_x(2:end-1)];
ref_path_y = [seg1_y, seg2_y(2:end), seg3a_y(2:end), seg3b_y, seg3c_y, seg4_y(2:end-1)];
total_points = length(ref_path_x);

% Profile computed once; the int16 VREF table lands in reference_track.h so
% the C harness indexes identical numbers instead of recomputing.
vprof = build_speed_profile(ref_path_x, ref_path_y, V_TOP, A_LAT, A_BRK, A_ACC);
VREF_INT = int16(round(vprof * SF_V));
VREF_INT(VREF_INT > 640) = int16(640);

hist_pos = nan(length(time), 2); hist_vel = nan(length(time), 1);
hist_str = nan(length(time), 1); hist_acc = nan(length(time), 1);
hist_err = nan(length(time), 1); hist_idx = nan(length(time), 1);
hist_vref = nan(length(time), 1);

N = length(time);
hist_x_in   = zeros(N,1,'int16'); hist_y_in   = zeros(N,1,'int16');
hist_psi_in = zeros(N,1,'int16'); hist_v_in   = zeros(N,1,'int16');
hist_rx_in  = zeros(N,1,'int16'); hist_ry_in  = zeros(N,1,'int16');
hist_rv_in  = zeros(N,1,'int16');
hist_eacc   = zeros(N,1,'int16'); hist_estr   = zeros(N,1,'int16');
hist_xp = zeros(N,1); hist_yp = zeros(N,1); hist_psip = zeros(N,1); hist_vp = zeros(N,1);

disp('Starting Simulation (Speed Profile Mode)...');
laps_completed = 0; prev_min_idx = 1;

for k = 1:length(time)
    ddx = ref_path_x - x_phys; ddy = ref_path_y - y_phys;
    dist_sq = ddx.*ddx + ddy.*ddy;
    [min_sq_val, min_idx] = min(dist_sq);
    hist_err(k) = sqrt(min_sq_val);

    if (min_idx < 50) && (prev_min_idx > total_points - 50); laps_completed = laps_completed + 1; end
    prev_min_idx = min_idx; hist_idx(k) = min_idx;

    lookahead_dist = max(LOOK_MIN, LOOK_GAIN * v_phys);
    acc_dist = 0; t_idx = min_idx;
    while acc_dist < lookahead_dist
        curr_i = t_idx; next_i = mod(t_idx, total_points) + 1;
        dx = ref_path_x(next_i) - ref_path_x(curr_i);
        dy = ref_path_y(next_i) - ref_path_y(curr_i);
        acc_dist = acc_dist + sqrt(dx*dx + dy*dy);
        t_idx = next_i;
        if acc_dist > lookahead_dist + 5; break; end
    end
    r_x = ref_path_x(t_idx); r_y = ref_path_y(t_idx);
    if strcmp(VIDX, 'near'); r_v = VREF_INT(min_idx);   % profile at current point
    else;                    r_v = VREF_INT(t_idx);    % profile at lookahead point
    end

    % Pre-update state, then quantize and call the controller
    hist_xp(k) = x_phys; hist_yp(k) = y_phys; hist_psip(k) = psi_phys; hist_vp(k) = v_phys;
    x_in = int16(x_phys*SF_POS); y_in = int16(y_phys*SF_POS);
    psi_wrap = mod(psi_phys + pi, 2*pi) - pi;
    psi_in = int16(psi_wrap*SF_PSI); v_in = int16(v_phys*SF_V);
    rx_in = int16(r_x*SF_POS); ry_in = int16(r_y*SF_POS);

    [acc_cmd_int, str_cmd_int] = fcs_mpc_v4(x_in, y_in, psi_in, v_in, rx_in, ry_in, r_v);

    hist_x_in(k) = x_in; hist_y_in(k) = y_in; hist_psi_in(k) = psi_in;
    hist_v_in(k) = v_in; hist_rx_in(k) = rx_in; hist_ry_in(k) = ry_in;
    hist_rv_in(k) = r_v;
    hist_eacc(k) = acc_cmd_int; hist_estr(k) = str_cmd_int;
    hist_vref(k) = double(r_v) / SF_V;

    acc_cmd = double(acc_cmd_int) / SF_V;
    str_cmd = double(str_cmd_int) * STEER_DECODE;
    [sp, cp] = sincos_det(psi_phys); [ss, cs] = sincos_det(str_cmd);
    x_phys = x_phys + v_phys * cp * Ts;
    y_phys = y_phys + v_phys * sp * Ts;
    psi_phys = psi_phys + (v_phys / L_phys) * (ss / cs) * Ts;
    v_phys = v_phys + acc_cmd; v_phys = v_phys * V_DRAG;
    if v_phys < 0; v_phys = 0; end  % defensive; unreachable with this cost

    hist_pos(k,:) = [x_phys, y_phys]; hist_vel(k) = v_phys;
    hist_str(k) = str_cmd; hist_acc(k) = acc_cmd / Ts;
end

total_laps_float = laps_completed + (hist_idx(end) / total_points);
fprintf('Total Laps: %.2f | Max Error: %.3f m | Top Speed: %.2f m/s\n', ...
    total_laps_float, max(hist_err), max(hist_vel));

% Self-check: replay the same quantised inputs through v3 (if on the path)
if exist('fcs_mpc_v3', 'file') == 2
    mm = 0;
    for k = 1:N
        [a3, s3] = fcs_mpc_v3(hist_x_in(k), hist_y_in(k), hist_psi_in(k), ...
                              hist_v_in(k), hist_rx_in(k), hist_ry_in(k), hist_rv_in(k));
        mm = mm + double(a3 ~= hist_eacc(k) || s3 ~= hist_estr(k));
    end
    fprintf('v4 vs v3 equivalence: %d mismatches / %d samples\n', mm, N);
end

stats = struct('laps', total_laps_float, 'max_err', max(hist_err), ...
    'v_top', max(hist_vel), 'err', hist_err, 'vel', hist_vel, ...
    'vref', hist_vref, 'pos', hist_pos, 'idx', hist_idx, ...
    'acc', hist_eacc, 'str', hist_estr);

if opts.write
    out_dir = pwd;
    write_trajectory_files(out_dir, hist_x_in, hist_y_in, hist_psi_in, ...
        hist_v_in, hist_rx_in, hist_ry_in, hist_rv_in, hist_eacc, hist_estr);
    write_reference_track_h(out_dir, ref_path_x, ref_path_y, VREF_INT, N, Ts, L_phys, ...
        SF_POS, SF_PSI, SF_V, STEER_DECODE, V_DRAG, LOOK_MIN, LOOK_GAIN, ...
        V_TOP, A_LAT, A_BRK, A_ACC);
    write_closed_loop_csv(out_dir, hist_xp, hist_yp, hist_psip, hist_vp, ...
        hist_x_in, hist_y_in, hist_psi_in, hist_v_in, hist_rx_in, hist_ry_in, ...
        hist_eacc, hist_estr, hist_err, hist_rv_in);
end

if opts.plots
    figure('Color','w', 'Position', [100 50 900 1000]);
    subplot(5,1,1); plot(ref_path_x, ref_path_y, 'k--', hist_pos(:,1), hist_pos(:,2), 'b.-');
    axis equal; title(sprintf('Laps: %.2f | Speed-Profile MPC (v4)', total_laps_float));
    subplot(5,1,2); plot(time, hist_vel, 'r', time, hist_vref, 'b--');
    ylabel('Vel'); ylim([0 11]); title('Velocity (red) vs profile v_{ref} (blue)');
    subplot(5,1,3); plot(time, rad2deg(hist_str), 'm'); ylabel('Deg'); ylim([-40 40]); title('Steering');
    subplot(5,1,4); plot(time, hist_acc, 'k'); ylabel('Acc'); ylim([-6 6]); title('Acceleration');
    subplot(5,1,5); plot(time, hist_err, 'b'); ylabel('Error (m)');
    ylim([0 1.2]); yline(1.0, 'r--', 'Limit'); title('Tracking Error'); grid on;
end

end


% =======================================================================
% Menger curvature per point, window-max, v=sqrt(A_LAT/kappa) capped at V_TOP,
% then backward (braking) and forward (accel) passes around the closed track.
function vprof = build_speed_profile(rx, ry, V_TOP, A_LAT, A_BRK, A_ACC)
    n = numel(rx);
    ds = zeros(1, n);
    for i = 1:n
        j = mod(i, n) + 1;
        ds(i) = sqrt((rx(j)-rx(i))^2 + (ry(j)-ry(i))^2);
    end
    w = 2;                      % ~1 m half-chord at 0.5 m spacing
    kappa = zeros(1, n);
    for i = 1:n
        a = mod(i-1-w, n) + 1; b = i; c = mod(i-1+w, n) + 1;
        ax = rx(a); ay = ry(a); bx = rx(b); by = ry(b); cx = rx(c); cy = ry(c);
        area2 = abs((bx-ax)*(cy-ay) - (by-ay)*(cx-ax));   % 2*triangle area
        dab = sqrt((bx-ax)^2 + (by-ay)^2);
        dbc = sqrt((cx-bx)^2 + (cy-by)^2);
        dca = sqrt((ax-cx)^2 + (ay-cy)^2);
        if dab*dbc*dca > 1e-9
            kappa(i) = 2*area2 / (dab*dbc*dca);
        end
    end
    ks = zeros(1, n);           % conservative: max curvature in a +/-3 window
    for i = 1:n
        kmax = 0;
        for d = -3:3
            kmax = max(kmax, kappa(mod(i-1+d, n) + 1));
        end
        ks(i) = kmax;
    end
    vprof = min(V_TOP, sqrt(A_LAT ./ max(ks, 1e-9)));
    for pass = 1:2              % backward pass: brake early enough
        for i = n:-1:1
            j = mod(i, n) + 1;
            vprof(i) = min(vprof(i), sqrt(vprof(j)^2 + 2*A_BRK*ds(i)));
        end
    end
    for pass = 1:2              % forward pass: physically reachable ramp-up
        for i = 1:n
            p = mod(i-2, n) + 1;
            vprof(i) = min(vprof(i), sqrt(vprof(p)^2 + 2*A_ACC*ds(p)));
        end
    end
end

% =======================================================================
% Deterministic sin/cos, IEEE-754 ops only, identical to the C harness (shared plant)
function [s, c] = sincos_det(z)
    zr = z - 6.283185307179586 * round(z / 6.283185307179586);
    sg = 1.0;
    if zr > 1.5707963267948966;        zr = 3.141592653589793 - zr;  sg = -1.0;
    elseif zr < -1.5707963267948966;   zr = -3.141592653589793 - zr; sg = -1.0;
    end
    z2 = zr * zr;
    s = zr * (1.0 + z2*(-1.6666666666666666e-1 + z2*(8.333333333333333e-3 + z2*(-1.984126984126984e-4 + ...
        z2*(2.7557319223985893e-6 + z2*(-2.505210838544172e-8 + z2*1.6059043836821613e-10))))));
    c = sg * (1.0 + z2*(-5.0e-1 + z2*(4.1666666666666664e-2 + z2*(-1.388888888888889e-3 + ...
        z2*(2.48015873015873e-5 + z2*(-2.7557319223985894e-7 + z2*2.08767569878681e-9))))));
end

function write_trajectory_files(out_dir, x, y, psi, v, rx, ry, rv, eacc, estr)
    N = length(x);
    assert(length(y)==N && length(psi)==N && length(v)==N && length(rx)==N ...
        && length(ry)==N && length(rv)==N && length(eacc)==N && length(estr)==N);

    dat_specs = { ...
        'x.dat',                  x;   'y.dat',                  y;   ...
        'psi.dat',                psi; 'v.dat',                  v;   ...
        'ref_x.dat',              rx;  'ref_y.dat',              ry;  ...
        'ref_v.dat',              rv;  ...
        'accel_cmd_expected.dat', eacc;'steer_cmd_expected.dat', estr ...
    };
    for i = 1:size(dat_specs,1)
        fid = fopen(fullfile(out_dir, dat_specs{i,1}), 'w');
        if fid < 0, continue; end
        fprintf(fid, '%04x\n', typecast(int16(dat_specs{i,2}), 'uint16'));
        fclose(fid);
    end
    fprintf('Wrote 9 .dat files (%d samples each)\n', N);

    h_specs = { ...
        'x_data',     x,    'sfix14 (1m=SF_POS=64)';     ...
        'y_data',     y,    'sfix13';                    ...
        'psi_data',   psi,  'sfix9  (2*pi~=256)';        ...
        'v_data',     v,    'ufix10 (1m/s=SF_V=64)';     ...
        'ref_x_data', rx,   'sfix14';                    ...
        'ref_y_data', ry,   'ufix12';                    ...
        'ref_v_data', rv,   'ufix10 (speed profile)';    ...
        'accel_exp',  eacc, 'sfix6';                     ...
        'steer_exp',  estr, 'sfix6';                     ...
    };
    h = fopen(fullfile(out_dir, 'trajectory_data.h'), 'w');
    fprintf(h, '#ifndef TRAJECTORY_DATA_H\n#define TRAJECTORY_DATA_H\n\n');
    fprintf(h, '#include <stdint.h>\n\n#define TRAJECTORY_LEN %d\n\n', N);
    for i = 1:size(h_specs,1)
        a = int16(h_specs{i,2});
        fprintf(h, '/* %s -- %s */\n', h_specs{i,1}, h_specs{i,3});
        fprintf(h, 'static const int16_t %s[TRAJECTORY_LEN] = {\n', h_specs{i,1});
        for j = 1:12:N
            row = a(j:min(j+11, N));
            fprintf(h, '    '); fprintf(h, '%6d, ', row(1:end-1));
            if (j+11) >= N, fprintf(h, '%6d\n', row(end));
            else,           fprintf(h, '%6d,\n', row(end)); end
        end
        fprintf(h, '};\n\n');
    end
    fprintf(h, '#endif\n'); fclose(h);
    fprintf('Wrote trajectory_data.h (%d samples)\n', N);

    pkg_specs = { ...
        'X_DATA',     x;     'Y_DATA',     y;    'PSI_DATA',   psi;   'V_DATA',     v;    ...
        'REF_X_DATA', rx;    'REF_Y_DATA', ry;   'REF_V_DATA', rv;    ...
        'ACCEL_EXP',  eacc;  'STEER_EXP',  estr; ...
    };
    p = fopen(fullfile(out_dir, 'trajectory_data_pkg.vhd'), 'w');
    fprintf(p, '-- Auto-generated by fcs_mpc_v4_tb.m -- DO NOT EDIT BY HAND.\n\n');
    fprintf(p, 'LIBRARY IEEE;\nUSE IEEE.std_logic_1164.ALL;\nUSE IEEE.numeric_std.ALL;\n\n');
    fprintf(p, 'PACKAGE trajectory_data_pkg IS\n\n');
    fprintf(p, '  CONSTANT TRAJECTORY_LEN : integer := %d;\n\n', N);
    fprintf(p, '  TYPE int_array IS ARRAY (NATURAL RANGE <>) OF integer;\n\n');
    for i = 1:size(pkg_specs,1)
        cname = pkg_specs{i,1}; a = int32(pkg_specs{i,2});
        fprintf(p, '  CONSTANT %s : int_array(0 TO TRAJECTORY_LEN-1) := (\n', cname);
        for j = 1:10:N
            row = a(j:min(j+9, N));
            fprintf(p, '    ');
            for kk = 1:length(row)
                if (j-1)+kk < N, fprintf(p, '%6d, ', row(kk)); else, fprintf(p, '%6d', row(kk)); end
            end
            fprintf(p, '\n');
        end
        fprintf(p, '  );\n\n');
    end
    fprintf(p, 'END trajectory_data_pkg;\n'); fclose(p);
    fprintf('Wrote trajectory_data_pkg.vhd (%d samples)\n', N);
end

% Raw track + sim params + VREF speed profile for mpc_closed_loop.c
function write_reference_track_h(out_dir, rx, ry, vref, N_steps, Ts, L, sf_pos, sf_psi, sf_v, steer_dec, drag, look_min, look_gain, v_top, a_lat, a_brk, a_acc)
    h = fopen(fullfile(out_dir, 'reference_track.h'), 'w');
    fprintf(h, '// reference_track.h - auto-generated closed-loop track + sim params (matches fcs_mpc_v4_tb.m). DO NOT EDIT.\n');
    fprintf(h, '#ifndef REFERENCE_TRACK_H\n#define REFERENCE_TRACK_H\n\n');
    fprintf(h, '#define TRACK_POINTS %d\n#define SIM_STEPS    %d\n\n', numel(rx), N_steps);
    fprintf(h, 'static const double TS = %.10g, L_WHEELBASE = %.10g;\n', Ts, L);
    fprintf(h, 'static const double SF_POS = %.10g, SF_PSI = %.10g, SF_V = %.10g;\n', sf_pos, sf_psi, sf_v);
    fprintf(h, 'static const double STEER_DECODE = %.10g, V_DRAG = %.10g;\n', steer_dec, drag);
    fprintf(h, 'static const double LOOKAHEAD_MIN = %.10g, LOOKAHEAD_GAIN = %.10g;\n', look_min, look_gain);
    fprintf(h, 'static const double X0 = 0.0, Y0 = 0.0, PSI0 = 0.0, V0 = 0.0;\n');
    fprintf(h, '// Speed profile source parameters (profile precomputed in MATLAB):\n');
    fprintf(h, '// V_TOP = %.10g m/s, A_LAT = %.10g, A_BRK = %.10g, A_ACC = %.10g m/s^2\n\n', v_top, a_lat, a_brk, a_acc);
    emit_double_array(h, 'REF_X', rx); fprintf(h, '\n');
    emit_double_array(h, 'REF_Y', ry); fprintf(h, '\n');
    emit_int16_array(h, 'VREF', vref); fprintf(h, '\n#endif\n');
    fclose(h);
    fprintf('Wrote reference_track.h (%d track points + VREF profile)\n', numel(rx));
end

function emit_double_array(h, name, v)
    N = numel(v);
    fprintf(h, 'static const double %s[TRACK_POINTS] = {\n', name);
    for j = 1:8:N
        row = v(j:min(j+7, N)); fprintf(h, '  ');
        for k = 1:numel(row)
            if (j-1)+k < N, fprintf(h, '%.10g, ', row(k)); else, fprintf(h, '%.10g', row(k)); end
        end
        fprintf(h, '\n');
    end
    fprintf(h, '};\n');
end

function emit_int16_array(h, name, v)
    N = numel(v);
    fprintf(h, 'static const int16_t %s[TRACK_POINTS] = {\n', name);
    for j = 1:12:N
        row = int32(v(j:min(j+11, N))); fprintf(h, '  ');
        for k = 1:numel(row)
            if (j-1)+k < N, fprintf(h, '%d, ', row(k)); else, fprintf(h, '%d', row(k)); end
        end
        fprintf(h, '\n');
    end
    fprintf(h, '};\n');
end

% Golden closed-loop CSV; v3 appends vref after err so columns 1..14 keep
% their v2 meaning and the comparison script needs only a width update.
function write_closed_loop_csv(out_dir, xp, yp, psip, vp, xq, yq, psiq, vq, rxq, ryq, ai, si, err, rv)
    N = numel(xp);
    f = fopen(fullfile(out_dir, 'matlab_closed_loop.csv'), 'w');
    fprintf(f, 'step,x,y,psi,v,xq,yq,psiq,vq,rxq,ryq,accel,steer,err,vref\n');
    for k = 1:N
        fprintf(f, '%d,%.10g,%.10g,%.10g,%.10g,%d,%d,%d,%d,%d,%d,%d,%d,%.10g,%d\n', ...
            k-1, xp(k), yp(k), psip(k), vp(k), xq(k), yq(k), psiq(k), vq(k), ...
            rxq(k), ryq(k), ai(k), si(k), err(k), rv(k));
    end
    fclose(f);
    fprintf('Wrote matlab_closed_loop.csv (%d samples)\n', N);
end
