clc;
clear;
close all;

% USE 'FS' MODE

% filename = "Futek_ADC-PCB";
filename = "DEMO_REC";

daqreset;
dev = daqlist("ni");
assert(height(dev)==1);
disp("Reading device:");
disp(dev.Description);

dq = daq("ni");
dq.Rate = 30; % Flir @ 8.7Hz
[ch1,idx1] = addinput(dq, dev.DeviceID, "ai0", "Voltage");
ch1.Range = [-5 5];

logData(dq, filename);

function logData(dq, filename)

    stopClean = @() stopSave(filename);
    cleanup = onCleanup(stopClean);

    daq = dq;
    
    figure(1);
    xlabel('Time (sec)');
    ylabel('Voltage (V)');
    
    start(daq, "continuous");
    
    data = [];
    while true

        pause(5);
        data = [data; read(daq, "all")];
        plot(data.Time, data.DAQ_ScrewTerm_ai0);
        drawnow;
    
    end
    
    function stopSave(filename)

        stop(daq);
        data = [data; read(daq, "all")];
        disp('Stopped DAQ');
        save(filename, 'data');
%         save("TESTTESTTEST.mat", 'data');
        disp('Saved Data');
    
    end

end