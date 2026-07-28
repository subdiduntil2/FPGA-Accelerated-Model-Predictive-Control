function fcs_mpc_v4_hdltb
% HDL Coder testbench: replays the 801 golden .dat vectors through fcs_mpc_v4.
d = @(f) typecast(uint16(sscanf(fileread(f), '%4x')), 'int16');
x = d('x.dat'); y = d('y.dat'); psi = d('psi.dat'); v = d('v.dat');
rx = d('ref_x.dat'); ry = d('ref_y.dat'); rv = d('ref_v.dat');
ea = d('accel_cmd_expected.dat'); es = d('steer_cmd_expected.dat');
ok = 0;
for k = 1:numel(x)
    [a, s] = fcs_mpc_v4(x(k), y(k), psi(k), v(k), rx(k), ry(k), rv(k));
    ok = ok + double(a == ea(k) && s == es(k));
end
fprintf('HDL TB vectors: %d / %d match\n', ok, numel(x));
end
