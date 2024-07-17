classdef thermalModel < handle

    properties

        Rwa % [
        Rha
        Rwh
        Cha
        Cwa

        t_train
        q_train
        active_train
        inactive_train
        housing_train
        T_ambient_train

        TW_data_train
        TH_data_train

        weight_W_train

        train_error

    end

    methods

        function obj = thermalModel
            % thermalModel  model of motor thermal properties
            %   Rwa: winding-ambient resistance
            %   Rha: housing-ambient resistance
            %   Rwh: winding-housing resistance
            %   Cha: housing-ambient capacitance
            %   Cwa: winding-ambient capacitance

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
            % simulate  simulate thermal model
            %   Inputs:
            %       t: n-element time vector
            %       q: n-element heat energy vector (i^2*r)
            %       T_ambient: scalar ambient temperature [Celsius]
            %       train_test: indicator; 0=none, 1=train, 2=test
            %   Outputs:
            %       TW: n-element vector of winding temperature
            %       TH: n-element vector of housing temperature


            s = tf('s');
            QsysW = 1/self.Rwa + 1/(self.Rwh + self.Rha/(self.Cha*self.Rha*s + 1)) + self.Cwa*s;
            TsysW = 1/QsysW;
        
            QsysH = (self.Rwh + self.Rha*self.Rwh*self.Cha*s + self.Rha)/(self.Rwa*self.Rha) + ...
                1/self.Rha + self.Cha*s + ...
                (self.Rwh*self.Cwa*s + self.Rha*self.Rwh*self.Cha*self.Cwa*s^2 + self.Rha*self.Cwa*s)/self.Rha;
            TsysH = 1/QsysH;

            TW = lsim(TsysW, q, t) + T_ambient;
            TH = lsim(TsysH, q, t) + T_ambient;

        end

        function model_error = test(self, t, q, active, inactive, housing, T_ambient, weight_W, plot_bool)

            [TW_model, TH_model] = self.simulate(t, q, T_ambient);

            TW_data = (active + 2*inactive)/3;
            TH_data = housing;

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

            model_error = self.model_RMSE(TW_model, TH_model, TW_data, TH_data, weight_W);

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