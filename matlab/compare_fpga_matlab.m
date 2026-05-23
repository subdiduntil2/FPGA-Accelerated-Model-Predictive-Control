%% compare_fpga_matlab.m
%
% 1-to-1 comparison of FPGA replay results against the original MATLAB
% MPC simulation.
%
% Workflow:
%   1. Build mpc_smoketest.c with -DOUTPUT_MODE=2  (MODE_CSV)
%   2. Run on the Cora Z7-07S, capture serial output to a file
%      (e.g. tera term log, "screen -L /dev/ttyUSB1 115200", picocom -g, etc.)
%      -> fpga_log.csv
%   3. Run this script.

clear; clc; close all;

% --- paths ---
DAT_DIR  = '.';
LOG_PATH = 'fpga_log_v2.csv';

% Decode scales (must match fcs_mpc_v2_tb2.m / fcs_mpc_v2.m)
SF_POS = 64;
SF_PSI = 41;
SF_V   = 64;
ACC_DECODE   = 1 / SF_V;     % m/s per accel-cmd unit (per Ts)
STEER_SCALE  = 0.025;        % rad per steer-cmd unit

% On-disk bit widths of each .dat file (the MATLAB writer truncates each
% signal to its natural width rather than always writing 16-bit).
%  field = {filename, nbits, signed}
%
% NOTE: accel_cmd is sfix6 (was sfix5 in an earlier project iteration).
% Widened to 6 bits because ACCEL_OPTS includes -20, which does not fit
% in 5-bit two's complement (range -16..15).  See report Appendix A.2.
DAT_SPEC = struct( ...
    'x',   {{'x.dat',                  14, true }}, ...
    'y',   {{'y.dat',                  13, true }}, ...
    'psi', {{'psi.dat',                 9, true }}, ...
    'v',   {{'v.dat',                   9, false}}, ...
    'rx',  {{'ref_x.dat',              14, true }}, ...
    'ry',  {{'ref_y.dat',              12, false}}, ...
    'acc', {{'accel_cmd_expected.dat',  6, true }}, ...  % was 5 (sfix5); now sfix6
    'str', {{'steer_cmd_expected.dat',  6, true }});

% --- 1. parse the FPGA CSV ---
fid = fopen(LOG_PATH, 'r');
assert(fid > 0, 'cannot open %s', LOG_PATH);
header = '';
data_lines = {};
while ~feof(fid)
    L = fgetl(fid);
    if ~ischar(L); break; end
    L = strtrim(L);
    if isempty(L), continue; end
    if startsWith(L, '#'), continue; end
    if isempty(header) && startsWith(L, 'idx,')
        header = L; continue;
    end
    data_lines{end+1} = L; %#ok<SAGROW>
end
fclose(fid);
fprintf('parsed %d data rows from %s\n', numel(data_lines), LOG_PATH);

M = zeros(numel(data_lines), 14);
for i = 1:numel(data_lines)
    M(i, :) = sscanf(data_lines{i}, '%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d').';
end
idx   = M(:,1);
gx    = M(:,2);  gy    = M(:,3);
gpsi  = M(:,4);  gv    = M(:,5);
grx   = M(:,6);  gry   = M(:,7);
gacc  = M(:,8);  eacc  = M(:,9);
gstr  = M(:,10); estr  = M(:,11);
pl_ns = M(:,12); e2_ns = M(:,13);
ok    = M(:,14);

% --- 2. cross-check against the .dat ground truth ---
%
% IMPORTANT: as of the mpc_smoketest.c hardening, CSV columns gx..gry are
% the values READ BACK from the FPGA's AXI input registers AFTER the kick
% completes - NOT the host-side echoes from the C app's trajectory_data.h
% arrays.  Therefore the check below ('input-echo check') is a real AXI
% bus integrity test: does what the FPGA hold across the kick equal what
% the host wrote (= what the .dat says we should have written)?
%
% A PASS confirms three things together:
%   (a) the .h and .dat came from the same MATLAB run (data consistency)
%   (b) the AXI writes reached the slave registers (write path OK)
%   (c) the AXI reads return what the registers contain (read path OK)
%
% If this fails, the trajectory_data.h compiled into the app and the .dat
% files in MATLAB's working directory came from different MATLAB runs,
% OR the AXI bus has integrity problems.

ref.x   = read_dat(fullfile(DAT_DIR, DAT_SPEC.x{1}),   DAT_SPEC.x{2},   DAT_SPEC.x{3});
ref.y   = read_dat(fullfile(DAT_DIR, DAT_SPEC.y{1}),   DAT_SPEC.y{2},   DAT_SPEC.y{3});
ref.psi = read_dat(fullfile(DAT_DIR, DAT_SPEC.psi{1}), DAT_SPEC.psi{2}, DAT_SPEC.psi{3});
ref.v   = read_dat(fullfile(DAT_DIR, DAT_SPEC.v{1}),   DAT_SPEC.v{2},   DAT_SPEC.v{3});
ref.rx  = read_dat(fullfile(DAT_DIR, DAT_SPEC.rx{1}),  DAT_SPEC.rx{2},  DAT_SPEC.rx{3});
ref.ry  = read_dat(fullfile(DAT_DIR, DAT_SPEC.ry{1}),  DAT_SPEC.ry{2},  DAT_SPEC.ry{3});
ref.acc = read_dat(fullfile(DAT_DIR, DAT_SPEC.acc{1}), DAT_SPEC.acc{2}, DAT_SPEC.acc{3});
ref.str = read_dat(fullfile(DAT_DIR, DAT_SPEC.str{1}), DAT_SPEC.str{2}, DAT_SPEC.str{3});

N = numel(idx);
assert(numel(ref.x) >= N, 'log has more samples (%d) than .dat (%d)', N, numel(ref.x));

% AXI-bus input-echo: FPGA-read state vector == .dat ground truth
in_ok = all(gx==ref.x(1:N)) && all(gy==ref.y(1:N)) && ...
        all(gpsi==ref.psi(1:N)) && all(gv==ref.v(1:N)) && ...
        all(grx==ref.rx(1:N))  && all(gry==ref.ry(1:N));
fprintf('input-echo check (AXI bus integrity): %s\n', tern(in_ok, 'PASS', 'FAIL'));

% Per-field input-echo detail (helps localize a FAIL to a specific register)
if ~in_ok
    fprintf('  per-field detail:\n');
    fprintf('    x   : %s   (first diff @ idx=%d)\n', ...
            tern(all(gx==ref.x(1:N)),'OK','MISMATCH'), ...
            find_first_diff(gx, ref.x(1:N)));
    fprintf('    y   : %s   (first diff @ idx=%d)\n', ...
            tern(all(gy==ref.y(1:N)),'OK','MISMATCH'), ...
            find_first_diff(gy, ref.y(1:N)));
    fprintf('    psi : %s   (first diff @ idx=%d)\n', ...
            tern(all(gpsi==ref.psi(1:N)),'OK','MISMATCH'), ...
            find_first_diff(gpsi, ref.psi(1:N)));
    fprintf('    v   : %s   (first diff @ idx=%d)\n', ...
            tern(all(gv==ref.v(1:N)),'OK','MISMATCH'), ...
            find_first_diff(gv, ref.v(1:N)));
    fprintf('    rx  : %s   (first diff @ idx=%d)\n', ...
            tern(all(grx==ref.rx(1:N)),'OK','MISMATCH'), ...
            find_first_diff(grx, ref.rx(1:N)));
    fprintf('    ry  : %s   (first diff @ idx=%d)\n', ...
            tern(all(gry==ref.ry(1:N)),'OK','MISMATCH'), ...
            find_first_diff(gry, ref.ry(1:N)));
end

% Verify expected matches .dat (host-side .h <-> .dat consistency)
% This is independent of FPGA execution and only checks that the C app's
% trajectory_data.h was generated from the same MATLAB run as the .dat
% files.  A FAIL means re-run fcs_mpc_v2_tb3.m and rebuild the Vitis app.
exp_ok = all(eacc==ref.acc(1:N)) && all(estr==ref.str(1:N));
fprintf('exp-echo  check (.h <-> .dat consistency): %s\n', tern(exp_ok, 'PASS', 'FAIL'));

% --- 3. mismatch report ---
acc_diff = gacc - eacc;
str_diff = gstr - estr;
n_acc = nnz(acc_diff ~= 0);
n_str = nnz(str_diff ~= 0);
fprintf('\nFPGA vs MATLAB:\n');
fprintf('  matches      : %d / %d  (%.2f%%)\n', nnz(ok), N, 100*nnz(ok)/N);
fprintf('  accel diffs  : %d   max|err|=%d\n',  n_acc, max(abs(acc_diff)));
fprintf('  steer diffs  : %d   max|err|=%d\n',  n_str, max(abs(str_diff)));
if n_acc + n_str > 0
    bad = find(acc_diff ~= 0 | str_diff ~= 0);
    fprintf('  first mismatch idx = %d\n', idx(bad(1)));
end

% --- 4. latency ---
fprintf('\nPL-only latency (ns):  min=%d  avg=%.1f  max=%d\n', ...
        min(pl_ns), mean(pl_ns), max(pl_ns));
fprintf('End-to-end latency (ns): min=%d  avg=%.1f  max=%d\n', ...
        min(e2_ns), mean(e2_ns), max(e2_ns));

% --- 5. plots ---
t = (0:N-1).' * 0.1;

figure('Color','w','Position',[80 80 1100 800]);

subplot(3,2,1);
plot(t, gacc, 'b.-', t, eacc, 'r--', 'LineWidth', 1.0); grid on;
legend('FPGA','MATLAB','Location','best');
title('accel\_cmd (raw int)'); xlabel('t [s]'); ylabel('units');

subplot(3,2,2);
plot(t, gstr, 'b.-', t, estr, 'r--', 'LineWidth', 1.0); grid on;
legend('FPGA','MATLAB','Location','best');
title('steer\_cmd (raw int)'); xlabel('t [s]'); ylabel('units');

subplot(3,2,3);
plot(t, double(gacc)*ACC_DECODE/0.1, 'b.-', ...
     t, double(eacc)*ACC_DECODE/0.1, 'r--', 'LineWidth', 1.0); grid on;
legend('FPGA','MATLAB','Location','best');
title('decoded acceleration [m/s^2]'); xlabel('t [s]');

subplot(3,2,4);
plot(t, rad2deg(double(gstr)*STEER_SCALE), 'b.-', ...
     t, rad2deg(double(estr)*STEER_SCALE), 'r--', 'LineWidth', 1.0); grid on;
legend('FPGA','MATLAB','Location','best');
title('decoded steering [deg]'); xlabel('t [s]');

subplot(3,2,5);
stem(t, acc_diff, 'k', 'Marker', '.'); grid on;
title('FPGA - MATLAB  accel'); xlabel('t [s]'); ylabel('units');

subplot(3,2,6);
stem(t, str_diff, 'k', 'Marker', '.'); grid on;
title('FPGA - MATLAB  steer'); xlabel('t [s]'); ylabel('units');

% Path overlay --- forward-integrate the bicycle model twice from the same
% initial state (x=y=psi=v=0): once with MATLAB's expected commands, once
% with the FPGA's actual commands.  Plant model is the one in
% fcs_mpc_v2_tb2.m (lines 70-76):
%     acc = double(acc_int)/SF_V;  str = double(str_int)*0.025;
%     x  += v*cos(psi)*Ts;   y += v*sin(psi)*Ts;
%     psi += (v/L)*tan(str)*Ts;
%     v   = (v + acc) * 0.99;
L_phys = 2.5;
Ts     = 0.1;

[x_mat, y_mat] = sim_plant(double(eacc), double(estr), SF_V, L_phys, Ts);
[x_fpg, y_fpg] = sim_plant(double(gacc), double(gstr), SF_V, L_phys, Ts);

% The .dat-provided closed-loop MATLAB trajectory (gx/gy already are this).
x_dat = double(gx) / SF_POS;
y_dat = double(gy) / SF_POS;

% Build the geometric reference racetrack from fcs_mpc_v2_tb2.m so it shows
% up regardless of how chaotic ref_x/ref_y look sample-by-sample.
[ref_path_x, ref_path_y] = build_reference_track();

figure('Color','w','Position',[120 120 850 750]);
plot(ref_path_x, ref_path_y, 'k-',  'LineWidth', 1.2, 'DisplayName','reference track'); hold on;
plot(x_dat,  y_dat,  'b.-', 'LineWidth', 1.0, 'DisplayName','MATLAB (.dat closed-loop)');
plot(x_mat,  y_mat,  'g--', 'LineWidth', 1.2, 'DisplayName','MATLAB cmds re-integrated');
plot(x_fpg,  y_fpg,  'r.-', 'LineWidth', 1.0, 'DisplayName','FPGA cmds re-integrated');
plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor','y', 'DisplayName','start');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
legend('Location','best');
title(sprintf('Trajectory comparison  (N=%d,  on-board matches=%d/%d = %.1f%%)', ...
              N, nnz(ok), N, 100*nnz(ok)/N));

% =========================================================================
function v = read_dat(p, nbits, is_signed)
%READ_DAT  Load a hex- or decimal-encoded .dat column, sign-extend from `nbits`.
%
%   The MATLAB writer truncates each signal to its natural width and emits
%   the lower nbits as a left-padded hex string (no '0x' prefix), e.g. 4
%   chars for x (14-bit), 3 chars for psi (9-bit).  We have to recognise
%   bare-hex (no a-f digits in some columns, like accel_cmd_expected.dat
%   which only emits '00'/'0a'/'16') and sign-extend from the correct bit.

    fid = fopen(p, 'r');
    assert(fid > 0, 'cannot open %s', p);
    raw = textscan(fid, '%s', 'Delimiter', '\n', 'CommentStyle', '#');
    fclose(fid);
    raw = raw{1};

    % --- pass 1: clean the lines, detect format ---
    lines = cell(numel(raw),1);
    nlines = 0;
    has_letter = false;
    has_0x     = false;
    widths     = [];
    only_digits = true;
    for i = 1:numel(raw)
        s = strtrim(raw{i});
        if isempty(s), continue; end
        s = regexprep(s, '[,;]+$', '');
        nlines = nlines + 1;
        lines{nlines} = s;
        sl = lower(s);
        if startsWith(sl, '0x') || startsWith(sl, '-0x'), has_0x = true; end
        if any(ismember(sl, 'abcdef')), has_letter = true; end
        widths(end+1) = length(s); %#ok<AGROW>
        if any(~ismember(s, '0123456789-.')), only_digits = false; end
    end
    lines = lines(1:nlines);

    % bare hex if: any [a-f] anywhere, OR all lines same width all-digit
    % (a decimal column wouldn't be left-padded to a fixed width).
    is_bare_hex = has_letter || ...
                  (nlines > 1 && numel(unique(widths)) == 1 && only_digits && ~has_0x);

    v = zeros(nlines, 1, 'int32');     % int32 so big positives don't wrap pre-sign-extend
    msk = int32(bitshift(int32(1), nbits) - 1);
    sign_bit = int32(bitshift(int32(1), nbits - 1));
    full = int32(bitshift(int32(1), nbits));

    for i = 1:nlines
        s = lines{i};
        sl = lower(s);
        if startsWith(sl, '0x')
            x = int32(sscanf(s, '%x'));
        elseif startsWith(sl, '-0x')
            x = -int32(sscanf(s(2:end), '%x'));
        elseif is_bare_hex || has_0x
            x = int32(sscanf(s, '%x'));
        else
            x = int32(sscanf(s, '%d'));
        end
        x = bitand(x, msk);
        if is_signed && bitand(x, sign_bit)
            x = x - full;
        end
        v(i) = x;
    end
end

function s = tern(c, a, b)
    if c, s = a; else, s = b; end
end

function k = find_first_diff(a, b)
% Return the first index where a and b differ, or -1 if identical.
% Used by the input-echo detail-printer.
    d = find(a(:) ~= b(:), 1, 'first');
    if isempty(d), k = -1; else, k = d - 1; end   % 0-based for log readability
end

function [xs, ys] = sim_plant(acc_int, str_int, SF_V, L, Ts)
% Forward-integrate the bicycle model from x=y=psi=v=0 using integer
% commands acc_int, str_int. Matches the plant code in fcs_mpc_v2_tb2.m.
    N = numel(acc_int);
    xs = zeros(N, 1);  ys = zeros(N, 1);
    x = 0; y = 0; psi = 0; v = 0;
    for k = 1:N
        acc = acc_int(k) / SF_V;
        str = str_int(k) * 0.025;
        x   = x + v * cos(psi) * Ts;
        y   = y + v * sin(psi) * Ts;
        psi = psi + (v / L) * tan(str) * Ts;
        v   = (v + acc) * 0.99;
        xs(k) = x;  ys(k) = y;
    end
end

function [rx, ry] = build_reference_track()
% Same racetrack geometry as fcs_mpc_v2_tb2.m, for plotting context.
    seg1_x = 0:0.5:60;             seg1_y = zeros(size(seg1_x));
    th2 = linspace(-pi/2, pi/2, 60);
    seg2_x = 60 + 15*cos(th2);     seg2_y = 15 + 15*sin(th2);
    seg3a_x = 60:-0.5:45;          seg3a_y = 30 * ones(size(seg3a_x));
    seg3b_x = 44.5:-0.5:15.5;
    phase   = (45 - seg3b_x) / 30 * 2*pi;
    seg3b_y = 30 + 4.0 * sin(phase);
    seg3c_x = 15:-0.5:0;           seg3c_y = 30 * ones(size(seg3c_x));
    th4 = linspace(pi/2, 3*pi/2, 60);
    seg4_x = 0  + 15*cos(th4);     seg4_y = 15 + 15*sin(th4);
    rx = [seg1_x, seg2_x(2:end), seg3a_x(2:end), seg3b_x, seg3c_x, seg4_x(2:end-1)];
    ry = [seg1_y, seg2_y(2:end), seg3a_y(2:end), seg3b_y, seg3c_y, seg4_y(2:end-1)];
end