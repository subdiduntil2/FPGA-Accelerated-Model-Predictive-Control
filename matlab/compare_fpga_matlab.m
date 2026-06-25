%% compare_fpga_matlab.m -- closed-loop validation: C harness (SW or FPGA backend)
%% vs MATLAB golden. Both share an IEEE-754-only deterministic plant (sincos_det),
%% so a clean run is bit-identical, not just within tolerance. Build the C app with
%% -ffp-contract=off. Rename the on-board FPGA-run log to c_closed_loop.csv.
clear; clc; close all;

golden_csv = 'matlab_closed_loop.csv';
c_csv      = 'c_closed_loop.csv';

G = load_csv(golden_csv); C = load_csv(c_csv);
n = min(size(G,1), size(C,1)); G = G(1:n,:); C = C(1:n,:);

% cols: 1 step 2 x 3 y 4 psi 5 v 6 xq 7 yq 8 psiq 9 vq 10 rxq 11 ryq 12 accel 13 steer 14 err
intcols  = 6:13;
row_mm   = any(G(:,intcols) ~= C(:,intcols), 2);
first_mm = find(row_mm, 1);
posdiff  = sqrt((G(:,2)-C(:,2)).^2 + (G(:,3)-C(:,3)).^2);

fprintf('Closed-loop comparison: %s vs %s (%d steps)\n', c_csv, golden_csv, n);
fprintf('  int-exact rows (state+ref+cmd) : %d / %d\n', sum(~row_mm), n);
if isempty(first_mm), fprintf('  first integer mismatch         : none\n');
else,                 fprintf('  first integer mismatch         : step %d\n', G(first_mm,1)); end
fprintf('  accel / steer mismatches       : %d / %d\n', sum(G(:,12)~=C(:,12)), sum(G(:,13)~=C(:,13)));
fprintf('  max |position| diff            : %.3e m\n', max(posdiff));
fprintf('  max |tracking err| diff        : %.3e m\n', max(abs(G(:,14)-C(:,14))));
fprintf('  MATLAB peak err / C peak err   : %.3f / %.3f m\n', max(G(:,14)), max(C(:,14)));
if isempty(first_mm)
    disp('PASS: command sequences bit-identical; trajectories match (shared deterministic plant).');
else
    disp('Integer mismatch found. With the shared deterministic plant this is a real divergence:');
    disp('check the C app was built -ffp-contract=off and that reference_track.h matches this run.');
end

[rx, ry] = build_reference_track();
figure('Color','w', 'Position', [100 80 1000 800]);
subplot(2,2,[1 3]); plot(rx, ry, 'k--'); hold on;
plot(G(:,2), G(:,3), 'b-'); plot(C(:,2), C(:,3), 'r:');
axis equal; legend('track','MATLAB','C'); title('Closed-loop trajectory');
subplot(2,2,2);
plot(G(:,1), G(:,12), 'b-', C(:,1), C(:,12), 'r:'); hold on;
plot(G(:,1), G(:,13), 'b--', C(:,1), C(:,13), 'r-.');
legend('accel M','accel C','steer M','steer C'); title('Commands (int)'); xlabel('step');
subplot(2,2,4);
plot(G(:,1), G(:,14), 'b-', C(:,1), C(:,14), 'r:'); hold on; plot(C(:,1), posdiff, 'k-');
legend('err M','err C','|pos diff|'); title('Tracking error and divergence'); xlabel('step');


% =======================================================================
function M = load_csv(path)
    fid = fopen(path, 'r');
    if fid < 0, error('cannot open %s', path); end
    raw = textscan(fid, '%s', 'Delimiter', '\n', 'Whitespace', ''); fclose(fid);
    lines = raw{1};
    lines = lines(~cellfun('isempty', regexp(lines, '^\s*-?\d', 'once')));  % data rows only: drop banner, header, blanks, '#' footer
    M = nan(numel(lines), 14);
    for i = 1:numel(lines)
        v = sscanf(strrep(lines{i}, ',', ' '), '%f');
        if numel(v) >= 14, M(i,:) = v(1:14).'; end
    end
    M = M(~any(isnan(M), 2), :);                 % drop any incomplete trailing row
end

function [rx, ry] = build_reference_track()
    seg1_x = 0:0.5:60; seg1_y = zeros(size(seg1_x));
    th2 = linspace(-pi/2, pi/2, 60); seg2_x = 60 + 15*cos(th2); seg2_y = 15 + 15*sin(th2);
    seg3a_x = 60:-0.5:45; seg3a_y = 30*ones(size(seg3a_x));
    seg3b_x = 44.5:-0.5:15.5; phase = (45 - seg3b_x)/30*2*pi; seg3b_y = 30 + 4.0*sin(phase);
    seg3c_x = 15:-0.5:0; seg3c_y = 30*ones(size(seg3c_x));
    th4 = linspace(pi/2, 3*pi/2, 60); seg4_x = 0 + 15*cos(th4); seg4_y = 15 + 15*sin(th4);
    rx = [seg1_x, seg2_x(2:end), seg3a_x(2:end), seg3b_x, seg3c_x, seg4_x(2:end-1)];
    ry = [seg1_y, seg2_y(2:end), seg3a_y(2:end), seg3b_y, seg3c_y, seg4_y(2:end-1)];
end
