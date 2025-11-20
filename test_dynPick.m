% clear;
% # ls -l /dev/tty.* # for macOS
% dpick = py.DynPick.DynPick('/dev/tty.usbserial-AU02EQ8G');
% # See device manager for windows OS
% dpick = py.DynPick.DynPick('COM3');

py.logging.basicConfig(pyargs('level', py.logging.INFO));
% py.logging.basicConfig(pyargs('level', py.logging.DEBUG));

DynPick = py.importlib.import_module('DynPick');
% py.importlib.reload(DynPick);

DynPick.DynPick.print_list_ports();

ports_cell = cell(py.serial.tools.list_ports.comports());
ports = cell(length(  ports_cell  ), 2);
for ii=1:length(ports_cell)
    ports{ii,1} = ports_cell{ii}.device.char;
    ports{ii,2} = ports_cell{ii}.serial_number.char;
end
dpick = py.DynPick.DynPick.open_ports_by_serial_number(ports{length(ports),2});

dpick.print_firmware_version();
dpick.set_sensitivity();
py.logging.info("%s [N], [Nm]", dpick.read_once());

dpick.start_continuous_read();
for ii=1:10
    pause(0.3);
    py.logging.info(dpick.read_continuous());
end
dpick.stop_continuous_read();

dpick.start_continuous_read();
num = 1000;
pause(0.1);
t1 = tic;
for ii=1:num
    dpick.read_continuous();
end
py.logging.info("%s sec (%d reads)", toc(t1), num);
dpick.stop_continuous_read();

clear dpick;
