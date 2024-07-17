classdef thermalModel < handle

    properties

        Rwa % [C/W], winding-to-ambient resistance
        Rha % [C/W], housing-to-ambient resistance
        Rwh % [C/W], winding-to-housing resistance
        Cha % [J/C], housing capacitance
        Cwa % [J/C], winding capacitance

        t_train % [s], time vector used to train model
        q_train % [W], heat input vector (i^2*R) used to train model
        active_train % [C], active (hot) phase winding temperature used to train model
        inactive_train % [C], inactive (cool) phase winding temperature used to train model
        housing_train % [C], housing temperature used to train model
        T_ambient_train % [C], ambient temperature used to train model

        TW_data_train % [C], total winding temperature used to train model
        TH_data_train % [C], total housing temperature used to train model

        weight_W_train % normalized weight attributed to winding RMSE when training model

        train_error % weighted sum of winding and housing temperature model RMSE at training solution

    end

    methods

        function obj = thermalModel
            % thermalModel  model of motor thermal properties
            return

        end

        function setParams(self, x)
            % setParams  set model parameters
            %   Inputs:
            %       x: [Rwa Rha Rwh Cha Cwa]

            self.Rwa = x(1);
            self.Rha = x(2);
            self.Rwh = x(3);
            self.Cha = x(4);
            self.Cwa = x(5);

        end

        function [TW, TH] = simulate(self, t, q, T_ambient)
            % simulate thermal model
            %   Inputs:
            %       t: n-element time vector
            %       q: n-element heat energy vector (i^2*r)
            %       T_ambient: scalar ambient temperature [Celsius]
            %   Outputs:
            %       TW: n-element vector of winding temperature
            %       TH: n-element vector of housing temperature

            s = tf('s');

            % winding transfer function
            QsysW = 1/self.Rwa + 1/(self.Rwh + self.Rha/(self.Cha*self.Rha*s + 1)) + self.Cwa*s;
            TsysW = 1/QsysW;
        
            % housing transfer function
            QsysH = (self.Rwh + self.Rha*self.Rwh*self.Cha*s + self.Rha)/(self.Rwa*self.Rha) + ...
                1/self.Rha + self.Cha*s + ...
                (self.Rwh*self.Cwa*s + self.Rha*self.Rwh*self.Cha*self.Cwa*s^2 + self.Rha*self.Cwa*s)/self.Rha;
            TsysH = 1/QsysH;

            % simulate systems with provided heat input (q) and add ambient to get absolute temperature
            TW = lsim(TsysW, q, t) + T_ambient;
            TH = lsim(TsysH, q, t) + T_ambient;

        end

        function model_error = test(self, t, q, active, inactive, housing, T_ambient, weight_W, plot_bool)
            % test thermal model parameterization against measured temperatures
            %   Inputs:
            %       t: n-element time vector
            %       q: n-element heat energy vector (i^2*r)
            %       active: n-element active winding phase temperature vector
            %       inactive: n-element inactive winding phase temperature vector
            %       housing: n-element housing temperature vector
            %       T_ambient: scalar ambient temperature [Celsius]
            %       weight_W: normalized [0,1] weight of winding RMSE relative to housing RMSE in error calculation
            %   Outputs:
            %       model_error: weighted sum of weighted sum of winding and housing temperature model RMSE

            [TW_model, TH_model] = self.simulate(t, q, T_ambient); % simulate the model with current parameterization

            TW_data = (active + 2*inactive)/3; % average active/inactive phases into single winding temperature
            % TW_data = (2*active + inactive)/3; NOTE: above calculation is for Dela wound motors; this is for Wye

            TH_data = housing;

            % plot measured and modeled temperature trajectories
            if plot_bool
                figure(plot_bool);
                if ~isempty(TW_data)
                    plot(t, TW_data, 'Color', '#0072BD');
                    hold on;
                end
                if ~isempty(TH_data)
                    plot(t, TH_data, 'Color', '#0072BD');
                    hold on;
                end
                plot(t, TW_model, 'Color', '#D95319');
                hold on;
                plot(t, TH_model, 'Color', '#D95319');
                hold off;
                xlabel('Time (s)');
                ylabel('Temperature (C)');
            end

            model_error = self.model_RMSE(TW_model, TH_model, TW_data, TH_data, weight_W); % compute weighted sum RMSE

        end

        function [x, model_error] = fit_fmincon(self, x0, A, b, Aeq, beq, lb, ub, nonlcon, options, t, q, active, inactive, housing, T_ambient, weight_W, plot_bool)

            func = self.setup_train(t, q, active, inactive, housing, T_ambient, weight_W);

            [x, model_error] = fmincon(func, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

            self.setParams(x);
            self.train_error = model_error;

            if plot_bool
                if class(t) == "double"
                    self.test(t, q, active, inactive, housing, T_ambient, weight_W, 1);
                elseif class(t) == "cell"
                    for i = 1:length(t)
                        self.test(t{i}, q{i}, active{i}, inactive{i}, housing{i}, T_ambient{i}, weight_W, 1)
                    end
                end
            end
            
        end

        function [x, model_error] = fit_ga(self, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options, t, q, active, inactive, housing, T_ambient, weight_W, plot_bool)

            func = self.setup_train(t, q, active, inactive, housing, T_ambient, weight_W);

            [x, model_error] = ga(func, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);

            self.setParams(x);
            self.train_error = model_error;

            if plot_bool
                if class(t) == "double"
                    self.test(t, q, active, inactive, housing, T_ambient, weight_W, 1);
                elseif class(t) == "cell"
                    for i = 1:length(t)
                        self.test(t{i}, q{i}, active{i}, inactive{i}, housing{i}, T_ambient{i}, weight_W, 1)
                    end
                end
            end

        end

    end

    methods (Access = private)

        function model_error = model_RMSE(~, TW_model, TH_model, TW_data, TH_data, weight_W)

            if ~isempty(TW_data)
                errorW = sqrt(sum((TW_model - TW_data).^2)/length(TW_data));
            else
                errorW = 0;
            end
            if ~isempty(TH_data)
                errorH = sqrt(sum((TH_model - TH_data).^2)/length(TH_data));
            else
                errorH = 0;
            end
        
            model_error = weight_W*errorW + (1-weight_W)*errorH;

        end

        function model_error = train(self)

            if class(self.t_train) == "double"

                [TW_model, TH_model] = self.simulate(self.t_train, self.q_train, self.T_ambient_train);
    
                model_error = self.model_RMSE(TW_model, TH_model, self.TW_data_train, self.TH_data_train, self.weight_W_train);

            elseif class(self.t_train) == "cell"

                model_error = 0;

                for i = 1:length(self.t_train)

                    [TW_model, TH_model] = self.simulate(self.t_train{i}, self.q_train{i}, self.T_ambient_train{i});

                    model_error = model_error + self.model_RMSE(TW_model, TH_model, self.TW_data_train{i}, self.TH_data_train{i}, self.weight_W_train);

                end

            end

        end

        function model_error = optimFunc(self, x)

            self.setParams(x);

            model_error = self.train();

        end

        function func = setup_train(self, t, q, active, inactive, housing, T_ambient, weight_W)

            self.t_train = t;
            self.q_train = q;
            self.active_train = active;
            self.inactive_train = inactive;
            self.housing_train = housing;
            self.T_ambient_train = T_ambient;
            self.weight_W_train = weight_W;

            if class(self.t_train) == "double"

                self.TW_data_train = (active + 2*inactive)/3;
                self.TH_data_train = housing;

            elseif class(self.t_train) == "cell"

                trials = length(self.t_train);

                self.TW_data_train = cell(trials,1);
                self.TH_data_train = cell(trials,1);

                for i = 1:trials

                    self.TW_data_train{i} = (active{i} + 2*inactive{i})/3;
                    self.TH_data_train{i} = housing{i};

                end

            else

                error("Data must be formatted as vectors or cell array of vectors, not %s", class(self.t_train));

            end

            func = @(x) self.optimFunc(x);

        end

    end

end