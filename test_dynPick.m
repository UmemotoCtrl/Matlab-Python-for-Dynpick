clear;
% # ls -l /dev/tty.* # for macOS
% dpick = py.DynPick.DynPick('/dev/tty.usbserial-AU02EQ8G');
% # See device manager for windows OS
% dpick = py.DynPick.DynPick('COM3');

% py.DynPick.DynPick.print_list_ports();
ports_cell = cell(py.serial.tools.list_ports.comports());
ports = cell(length(ports_cell), 2);
for ii=1:length(ports_cell)
    ports{ii,1} = ports_cell{ii}.device.char;
    ports{ii,2} = ports_cell{ii}.serial_number.char;
end
disp(ports);
dpick = py.DynPick.DynPick.open_ports_by_serial_number(ports{length(ports),2});
dpick.set_sensitivity();
disp({double(dpick.read_once()), "[N], [Nm]"});

dpick.start_continuous_read();
for ii=1:10
    pause(0.3);
    disp(double(dpick.read_continuous()));
end
dpick.stop_continuous_read();

clear dpick;

