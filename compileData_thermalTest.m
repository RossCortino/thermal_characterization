function data = compileData_thermalTest(current_step_val, T_active_csv, T_inactive_csv, T_frame_csv, fahrenheit_flag, V_mat, interp_T)

    temp_lengths = [];
    if ~isempty(T_active_csv)
        temp.active = formatROIdata(T_active_csv, fahrenheit_flag);
        temp_lengths = [temp_lengths length(temp.active)];
    end
    if ~isempty(T_inactive_csv)
        temp.inactive = formatROIdata(T_inactive_csv, fahrenheit_flag);
        temp_lengths = [temp_lengths length(temp.inactive)];
    end
    if ~isempty(T_frame_csv)
        temp.frame = formatROIdata(T_frame_csv, fahrenheit_flag);
        temp_lengths = [temp_lengths length(temp.frame)];
    end

    if length(temp_lengths) > 1
        assert(range(temp_lengths) == 0); % ensure all temperature vectors are of the same length
    end

    temp_fns = fieldnames(temp);

    DAQ = load(V_mat, 'data');
    volt_label = DAQ.data.Properties.VariableNames{1};

    figure(1);
    for i = 1:length(temp_fns)
        plot(temp.(temp_fns{i}));
        hold on;
    end
    hold off;
    xlabel('Index');
    ylabel('Temperature (^oC)');

    [temp_idx_on, temp_idx_off] = pickBounds();
    temp_idx_on = round(temp_idx_on);
    temp_idx_off = round(temp_idx_off);

    plot(seconds(DAQ.data.Time), DAQ.data.(volt_label));
    xlabel('Time (sec)');
    ylabel('Voltage (V)');

    [DAQ_t_on, DAQ_t_off] = pickBounds();

    close 1;

    DAQ_t = seconds(DAQ.data.Time);
    voltage_on_idcs = DAQ_t >= DAQ_t_on & DAQ_t <= DAQ_t_off;
    voltage_on = DAQ.data.(volt_label)(voltage_on_idcs);
    t_DAQ_on = DAQ_t(voltage_on_idcs);
    t_DAQ_on = t_DAQ_on - t_DAQ_on(1);

    T_temp = t_DAQ_on(end)/(temp_idx_off - temp_idx_on + 1); % temp data sample period

    temp_start = [];
    for i = 1:length(temp_fns)
        temp_start = [temp_start mean(temp.(temp_fns{i})(1:temp_idx_on))];
        temp.(temp_fns{i}) = temp.(temp_fns{i})(temp_idx_on:end);
        if i == 1
            temp.t = (0:length(temp.(temp_fns{i}))-1)'*T_temp;
        end
    end
    data.ambient = mean(temp_start);
    
    t_f = temp.t(end);
    data.t = (0:interp_T:t_f)';
    interp_method = "pchip";

    for i = 1:length(temp_fns)
        data.(temp_fns{i}) = interp1(temp.t, temp.(temp_fns{i}), data.t, interp_method);
    end

    data.voltage = interp1(t_DAQ_on, voltage_on, data.t, interp_method, 0);
    data.resistance = data.voltage/current_step_val;
    data.q = data.voltage*current_step_val;

    function [x_on, y_off] = pickBounds()

        input('Scale plot to start of rise.');
        disp('Select on point.');
        [x_on, ~] = ginput(1);
        disp('Time On: ' + string(x_on));

        input('Scale plot to start of fall.');
        disp('Select off point.');
        [y_off, ~] = ginput(1);
        disp('Time Off: ' + string(y_off));

    end

    function data_avg = formatROIdata(filename, fahrenheit_flag)

        rawdata = readmatrix(filename);
        if fahrenheit_flag
            rawdata = (rawdata - 32)*5/9; % convert to Celsius
        end
        data_mat = permute(reshape(rawdata', 5, 5, []), [2 1 3]); % reshape exported thermal data
        data_avg = mean(data_mat, [1 2]); % average across each (5x5) ROI sample
        data_avg = data_avg(:);

    end

end