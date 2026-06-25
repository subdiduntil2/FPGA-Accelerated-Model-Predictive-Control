%% fcs_mpc_v2_tb3.m -- closed-loop MATLAB sim; emits .dat, .h, and .vhd trajectory data.
clear; clc; close all;

% Configuration
SF_POS  = 64;
SF_PSI  = 41;
SF_V    = 64;
Ts      = 0.1;
T_SIM   = 80;
time    = 0:Ts:T_SIM;

% Initial conditions
v_phys   = 0.0; x_phys = 0; y_phys = 0; psi_phys = 0; L_phys = 2.5;

% Reference racetrack
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

% Histories
hist_pos = nan(length(time), 2); hist_vel = nan(length(time), 1);
hist_str = nan(length(time), 1); hist_acc = nan(length(time), 1);
hist_err = nan(length(time), 1); hist_idx = nan(length(time), 1);

N = length(time);
hist_x_in   = zeros(N,1,'int16');
hist_y_in   = zeros(N,1,'int16');
hist_psi_in = zeros(N,1,'int16');
hist_v_in   = zeros(N,1,'int16');
hist_rx_in  = zeros(N,1,'int16');
hist_ry_in  = zeros(N,1,'int16');
hist_eacc   = zeros(N,1,'int16');
hist_estr   = zeros(N,1,'int16');

disp('Starting Simulation (Constraint Mode)...');
laps_completed = 0; prev_min_idx = 1;

for k = 1:length(time)
    dist_sq = (ref_path_x - x_phys).^2 + (ref_path_y - y_phys).^2;
    [min_sq_val, min_idx] = min(dist_sq);
    hist_err(k) = sqrt(min_sq_val);

    if (min_idx < 50) && (prev_min_idx > total_points - 50); laps_completed = laps_completed + 1; end
    prev_min_idx = min_idx; hist_idx(k) = min_idx;

    lookahead_dist = max(0.8, 0.25 * v_phys);
    acc_dist = 0; t_idx = min_idx;
    while acc_dist < lookahead_dist
        curr_i = t_idx; next_i = mod(t_idx, total_points) + 1;
        dx = ref_path_x(next_i) - ref_path_x(curr_i);
        dy = ref_path_y(next_i) - ref_path_y(curr_i);
        acc_dist = acc_dist + sqrt(dx^2 + dy^2);
        t_idx = next_i;
        if acc_dist > lookahead_dist + 5; break; end
    end
    r_x = ref_path_x(t_idx); r_y = ref_path_y(t_idx);

    x_in = int16(x_phys*SF_POS); y_in = int16(y_phys*SF_POS);
    psi_wrap = mod(psi_phys + pi, 2*pi) - pi;
    psi_in = int16(psi_wrap*SF_PSI); v_in = int16(v_phys*SF_V);
    rx_in = int16(r_x*SF_POS); ry_in = int16(r_y*SF_POS);

    [acc_cmd_int, str_cmd_int] = fcs_mpc_v2(x_in, y_in, psi_in, v_in, rx_in, ry_in);

    hist_x_in(k)   = x_in;
    hist_y_in(k)   = y_in;
    hist_psi_in(k) = psi_in;
    hist_v_in(k)   = v_in;
    hist_rx_in(k)  = rx_in;
    hist_ry_in(k)  = ry_in;
    hist_eacc(k)   = acc_cmd_int;
    hist_estr(k)   = str_cmd_int;

    acc_cmd = double(acc_cmd_int) / SF_V;
    str_cmd = double(str_cmd_int) * 0.025;
    x_phys = x_phys + v_phys * cos(psi_phys) * Ts;
    y_phys = y_phys + v_phys * sin(psi_phys) * Ts;
    psi_phys = psi_phys + (v_phys / L_phys) * tan(str_cmd) * Ts;
    v_phys = v_phys + acc_cmd; v_phys = v_phys * 0.99;

    hist_pos(k,:) = [x_phys, y_phys]; hist_vel(k) = v_phys;
    hist_str(k) = str_cmd; hist_acc(k) = acc_cmd / Ts;
end

total_laps_float = laps_completed + (hist_idx(end) / total_points);
fprintf('Total Laps: %.2f | Max Error: %.3f m\n', total_laps_float, max(hist_err));

% Emit trajectory artefacts (.dat, .h, .vhd)
out_dir = pwd;
write_trajectory_files(out_dir, hist_x_in, hist_y_in, hist_psi_in, ...
    hist_v_in, hist_rx_in, hist_ry_in, hist_eacc, hist_estr);

% Plots
figure('Color','w', 'Position', [100 50 900 1000]);
subplot(5,1,1); plot(ref_path_x, ref_path_y, 'k--', hist_pos(:,1), hist_pos(:,2), 'b.-');
axis equal; title(sprintf('Laps: %.2f | Robust MPC', total_laps_float));
subplot(5,1,2); plot(time, hist_vel, 'r'); ylabel('Vel'); ylim([0 6]); title('Velocity');
subplot(5,1,3); plot(time, rad2deg(hist_str), 'm'); ylabel('Deg'); ylim([-40 40]); title('Steering');
subplot(5,1,4); plot(time, hist_acc, 'k'); ylabel('Acc'); ylim([-3 3]); title('Acceleration');
subplot(5,1,5); plot(time, hist_err, 'b'); ylabel('Error (m)');
ylim([0 1.2]); yline(1.0, 'r--', 'Limit'); title('Tracking Error'); grid on;


% =======================================================================
function write_trajectory_files(out_dir, x, y, psi, v, rx, ry, eacc, estr)
    N = length(x);
    assert(length(y)==N && length(psi)==N && length(v)==N && length(rx)==N ...
        && length(ry)==N && length(eacc)==N && length(estr)==N);

    % .dat files (legacy)
    dat_specs = { ...
        'x.dat',                  x;   'y.dat',                  y;   ...
        'psi.dat',                psi; 'v.dat',                  v;   ...
        'ref_x.dat',              rx;  'ref_y.dat',              ry;  ...
        'accel_cmd_expected.dat', eacc;'steer_cmd_expected.dat', estr ...
    };
    for i = 1:size(dat_specs,1)
        fid = fopen(fullfile(out_dir, dat_specs{i,1}), 'w');
        if fid < 0, continue; end
        fprintf(fid, '%04x\n', typecast(int16(dat_specs{i,2}), 'uint16'));
        fclose(fid);
    end
    fprintf('Wrote 8 .dat files (%d samples each)\n', N);

    % trajectory_data.h (C, for Cora bare-metal app)
    h_specs = { ...
        'x_data',     x,    'sfix14 (1m=SF_POS=64)';     ...
        'y_data',     y,    'sfix13';                    ...
        'psi_data',   psi,  'sfix9  (2*pi~=256)';        ...
        'v_data',     v,    'ufix9  (1m/s=SF_V=64)';     ...
        'ref_x_data', rx,   'sfix14';                    ...
        'ref_y_data', ry,   'ufix12';                    ...
        'accel_exp',  eacc, 'sfix6  (was sfix5)';        ...
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
            fprintf(h, '    ');
            fprintf(h, '%6d, ', row(1:end-1));
            if (j+11) >= N, fprintf(h, '%6d\n', row(end));
            else,           fprintf(h, '%6d,\n', row(end)); end
        end
        fprintf(h, '};\n\n');
    end
    fprintf(h, '#endif\n');
    fclose(h);
    fprintf('Wrote trajectory_data.h (%d samples)\n', N);

    % trajectory_data_pkg.vhd (consumed by the VHDL testbench at elaboration time)
    pkg_specs = { ...
        'X_DATA',     x;     'Y_DATA',     y;    ...
        'PSI_DATA',   psi;   'V_DATA',     v;    ...
        'REF_X_DATA', rx;    'REF_Y_DATA', ry;   ...
        'ACCEL_EXP',  eacc;  'STEER_EXP',  estr; ...
    };
    p = fopen(fullfile(out_dir, 'trajectory_data_pkg.vhd'), 'w');
    fprintf(p, '-- =========================================================================\n');
    fprintf(p, '-- trajectory_data_pkg.vhd\n');
    fprintf(p, '--\n');
    fprintf(p, '-- Auto-generated by fcs_mpc_v2_tb3.m  -- DO NOT EDIT BY HAND.\n');
    fprintf(p, '-- Constant arrays from the closed-loop MATLAB simulation.\n');
    fprintf(p, '-- Consumed by fcs_mpc_v2_fixpt_tb.vhd at elaboration time.\n');
    fprintf(p, '-- Total laps: %.2f   Max tracking error: %.3f m\n', 0.0, 0.0);
    fprintf(p, '-- =========================================================================\n\n');
    fprintf(p, 'LIBRARY IEEE;\n');
    fprintf(p, 'USE IEEE.std_logic_1164.ALL;\n');
    fprintf(p, 'USE IEEE.numeric_std.ALL;\n\n');
    fprintf(p, 'PACKAGE trajectory_data_pkg IS\n\n');
    fprintf(p, '  CONSTANT TRAJECTORY_LEN : integer := %d;\n\n', N);
    fprintf(p, '  TYPE int_array IS ARRAY (NATURAL RANGE <>) OF integer;\n\n');
    for i = 1:size(pkg_specs,1)
        cname = pkg_specs{i,1};
        a = int32(pkg_specs{i,2});
        fprintf(p, '  CONSTANT %s : int_array(0 TO TRAJECTORY_LEN-1) := (\n', cname);
        COLS = 10;
        for j = 1:COLS:N
            row = a(j:min(j+COLS-1, N));
            fprintf(p, '    ');
            last_idx_in_array = (j-1) + length(row);
            for k = 1:length(row)
                abs_idx = (j-1) + k;
                if abs_idx < N
                    fprintf(p, '%6d, ', row(k));
                else
                    fprintf(p, '%6d', row(k));
                end
            end
            fprintf(p, '\n');
        end
        fprintf(p, '  );\n\n');
    end
    fprintf(p, 'END trajectory_data_pkg;\n');
    fclose(p);
    fprintf('Wrote trajectory_data_pkg.vhd (%d samples)\n', N);
end
