clearvars
ekf_functions

path = fullfile(currentdir(), '..', 'generated', 'ackermann');
strdump(char(f), fullfile(path, 'f.txt'))
strdump(char(F), fullfile(path, 'F.txt'))
strdump(char(h), fullfile(path, 'h.txt'))
strdump(char(H), fullfile(path, 'H.txt'))
strdump(char(state), fullfile(path, 'x.txt'))
strdump(char(observables), fullfile(path, 'z.txt'))
strdump(char(dt), fullfile(path, 'dt.txt'))
strdump(char(sensor_position), fullfile(path, 'sensor_position.txt'))