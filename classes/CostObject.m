classdef CostObject < handle
    %ObservabilityCostObject
    %   Detailed explanation goes here
    
    properties
        Problem;
        Setup;
    end
    properties (Dependent)
        NPhases;
        NTimeStepsPerPhase;
        NOutputs;
        OutputNames;
        NStates;
        StateNames;
        NControls;
        ControlNames;
        Parameters;
        P_0;
        Q;
        R;
        ObserverSeed;
        StdPos;
        StdVel;
        CostScaling;
        TimeCostScaling;
        ObsCostScaling;
        
        %         CostScaling;
    end
    
    methods
        
        function ret = get.NPhases(obj)
            ret = length(obj.Problem.Phases);
        end
        
        function ret = get.NTimeStepsPerPhase(obj)
            ret = length(obj.Problem.RealTime);
        end
        
        function ret = get.NOutputs(obj)
            ret = length(obj.Problem.OutputNames);
        end
        
        function ret = get.OutputNames(obj)
            ret = obj.Problem.OutputNames;
        end
        
        function ret = get.NStates(obj)
            ret = length(obj.Problem.StateNames);
        end
        
        function ret = get.StateNames(obj)
            ret =  obj.Problem.StateNames;
        end
        
        function ret = get.NControls(obj)
            ret = length(obj.Problem.ControlNames);
        end
        
        function ret = get.ControlNames(obj)
            ret = obj.Problem.ControlNames;
        end
        
        function ret = get.Parameters(obj)
            ret = {
                obj.Problem.Parameters(1).Name
                obj.Problem.Parameters(2).Name
            };
        end
        
        function ret = get.P_0(obj)
            ret = obj.Setup.observerConfig.P_0;
        end
        
        function ret = get.Q(obj)
            ret = obj.Setup.observerConfig.Q;
        end
        
        function ret = get.R(obj)
            ret = obj.Setup.observerConfig.R;
        end
        
        function ret = get.ObserverSeed(obj)
            ret = obj.Setup.observerConfig.Seed;
        end
        
        function ret = get.StdPos(obj)
            ret = obj.Setup.observerConfig.StdPos;
        end
        
        function ret = get.StdVel(obj)
            ret = obj.Setup.observerConfig.StdVel;
        end
        
        function ret = get.CostScaling(obj)
            ret = obj.Setup.CostScaling;
        end
        
        function ret = get.TimeCostScaling(obj)
            ret = obj.Setup.TimeCostScaling;
        end
        
        function ret = get.ObsCostScaling(obj)
            ret = obj.Setup.ObsCostScaling;
        end
        
        function ret = stateJac_x(obj,dt)
            ret =  [
                1 , 0 , 0 , dt ,  0 , 0
                0 , 1 , 0 ,  0 , dt , 0
                0 , 0 , 1 ,  0 ,  0 , dt
                0 , 0 , 0 ,  1 ,  0 , 0
                0 , 0 , 0 ,  0 ,  1 , 0
                0 , 0 , 0 ,  0 ,  0 , 1
            ];
        end
        
        function ret = stateJac_w(obj,dt)
            ret =  [
                0.5*dt^2, 0         , 0
                0       , 0.5*dt^2  , 0
                0       , 0         , 0.5*dt^2
                dt      , 0         , 0
                0       , dt        , 0
                0       , 0         , dt
            ];
        end
        
        function ret = measFcn(obj, states)
            
            x = states(1);
            y = states(2);
            z = states(3);
            
            ret = [
                atan2(y,x)
                atan2(-z,sqrt(x^2 + y^2))
            ];
        end
        
        function ret = measJac(obj, states)
            
            x = states(1);
            y = states(2);
            z = states(3);
            
            ret = [
                -y/(x^2 + y^2)   , (x*z)/((x^2 + y^2)^(1/2)*(x^2 + y^2 + z^2))
                x/(x^2 + y^2)    , (y*z)/((x^2 + y^2)^(1/2)*(x^2 + y^2 + z^2))
                0                , -(x^2 + y^2)^(1/2)/(x^2 + y^2 + z^2)
                0                , 0
                0                , 0
                0                , 0
            ]';
        end
        
        function ret = stateFcn(obj, states, controls, dt)
            ret = [
                1 , 0 , 0 , dt ,  0 , 0
                0 , 1 , 0 ,  0 , dt , 0
                0 , 0 , 1 ,  0 ,  0 , dt
                0 , 0 , 0 ,  1 ,  0 , 0
                0 , 0 , 0 ,  0 ,  1 , 0
                0 , 0 , 0 ,  0 ,  0 , 1
            ] * states...
            - ...
            [
                0.5*dt^2 , 0        , 0
                0        , 0.5*dt^2 , 0
                0        , 0        , 0.5*dt^2
                dt       , 0        , 0
                0        , dt       , 0
                0        , 0        , dt
            ] * controls;
        end        
       
        function [j,j_jac] = observabilityCostFcn(obj, varargin)
            
            if numel(varargin) == 0 && nargout <= 1
                
                cnt = 1;
                
                nTimeStepsPerPhase = obj.NTimeStepsPerPhase;
                
                for iPhase=1:obj.NPhases
                    
                    % outputs
                    
                    str.input(cnt,1).m = obj.NOutputs;
                    
                    str.input(cnt,1).n = nTimeStepsPerPhase(iPhase);
                    
                    str.input(cnt,1).name = ['outputs_g' num2str(iPhase)];
                    
                    str.input(cnt,1).argnames = strcat(obj.OutputNames, ['_g' num2str(iPhase)]);
                    
                    str.input(cnt,1).type = 'OUTPUT';
                    
                    str.input(cnt,1).groupindex = iPhase;
                    
                    cnt = cnt + 1;
                    
                    
                    
                    % states
                    
                    str.input(cnt,1).m = obj.NStates;
                    
                    str.input(cnt,1).n = nTimeStepsPerPhase(iPhase);
                    
                    str.input(cnt,1).name = ['states_g' num2str(iPhase)];
                    
                    str.input(cnt,1).argnames = strcat(obj.StateNames, ['_g' num2str(iPhase)]);
                    
                    str.input(cnt,1).type = 'STATE';
                    
                    str.input(cnt,1).groupindex = iPhase;
                    
                    cnt = cnt + 1;
                    
                    
                    
                    % controls
                    
                    str.input(cnt,1).m = obj.NControls;
                    
                    str.input(cnt,1).n = nTimeStepsPerPhase(iPhase);
                    
                    str.input(cnt,1).name = ['controls_g' num2str(iPhase)];
                    
                    str.input(cnt,1).argnames = strcat(obj.ControlNames, ['_g' num2str(iPhase)]);
                    
                    str.input(cnt,1).type = 'CONTROL';
                    
                    str.input(cnt,1).groupindex = iPhase;
                    
                    cnt = cnt + 1;
                    
                end
                
                
                
                %         % parameters
                
                str.input(cnt,1).m = obj.NPhases+1;
                
                str.input(cnt,1).n = 1;
                
                str.input(cnt,1).name = 'parameters';
                
                str.input(cnt,1).argnames = obj.Parameters;
                
                str.input(cnt,1).type = 'PARAMETER';
                
                str.input(cnt,1).groupindex = 0;
                
                
                
                % constraintvalue
                
                str.output(1).m = 1;
                
                str.output(1).n = 1;
                
                str.output(1).name = 'constraintvalue';
                
                str.output(1).argnames = {'observability'};
                
                str.output(1).type = 'VALUE';
                
                str.output(1).jac_sparsity = ones(1,sum([str.input.m].*[str.input.n]));
                
                str.output(1).hess_sparsity = [];
                
                
                
                % Info
                
                str.info.Date = '20-Oct-2020';
                
                str.info.Computer = 'None';
                
                str.info.MATLAB = 'R2020 Update ?';
                
                str.info.Hessian = false;
                
                
                
                % Other Info
                
                str.name = 'observability';
                
                str.type = 'POINT_FUNCTION';
                
                
                j = str;
                
                return;
                
            end
            
            
            
            states = cell(obj.NPhases,1);
            
            controls = cell(obj.NPhases,1);
            
            time = cell(obj.NPhases,1);
            
            outputs = cell(obj.NPhases,1);
            
            
            
            % Parameters
            if size(varargin{end},1) ~= obj.NPhases+1
                
                error('Dimensions of parameters do not match');
                
            end
            
            if size(varargin{end},2) ~= 1
                
                error('Dimensions of parameters do not match');
                
            end
            
            
            
            cntPhase = 1;
            
            nTimeStepsPerPhase = obj.NTimeStepsPerPhase;
            
            for iInput=1:3:numel(varargin)-1
                
                % Outputs
                
                if size(varargin{iInput},1) ~= obj.NOutputs
                    
                    error('Dimensions do not match');
                    
                end
                
                if size(varargin{iInput},2) ~= obj.NTimeStepsPerPhase
                    
                    error('Dimensions do not match');
                    
                end
                
                outputs{cntPhase} = varargin{iInput};
                
                
                
                % States
                
                if size(varargin{iInput+1},1) ~= obj.NStates
                    
                    error('Dimensions do not match');
                    
                end
                
                if size(varargin{iInput+1},2) ~= obj.NTimeStepsPerPhase
                    
                    error('Dimensions do not match');
                    
                end
                
                states{cntPhase} = varargin{iInput+1};
                
                
                
                % controls
                
                if size(varargin{iInput+2},1) ~= obj.NControls
                    
                    error('Dimensions do not match');
                    
                end
                
                if size(varargin{iInput+2},2) ~= obj.NTimeStepsPerPhase
                    
                    error('Dimensions do not match');
                    
                end
                
                controls{cntPhase} = varargin{iInput+2};
                
                
                
                % time
                time{cntPhase} = linspace(varargin{end}(cntPhase),varargin{end}(cntPhase+1),nTimeStepsPerPhase(cntPhase));
                
                % increment phase counter
                cntPhase = cntPhase + 1;
                
            end
            
            %% Get States, Controls and Measurements
            
            % Time Handling
            N       = nTimeStepsPerPhase;
            dt      = diff(time{:}(1:2));
            
            % Defender States
            x = states{:}(strcmp(obj.StateNames,'x'), :);
            y = states{:}(strcmp(obj.StateNames,'y'), :);
            z = states{:}(strcmp(obj.StateNames,'z'), :);
            u = states{:}(strcmp(obj.StateNames,'u'), :);
            v = states{:}(strcmp(obj.StateNames,'v'), :);
            w = states{:}(strcmp(obj.StateNames,'w'), :);
            % Invader States
            x_inv = states{:}(strcmp(obj.StateNames    ,'x_inv'), :);
            y_inv = states{:}(strcmp(obj.StateNames    ,'y_inv'), :);
            z_inv = states{:}(strcmp(obj.StateNames    ,'z_inv'), :);
            u_inv = states{:}(strcmp(obj.OutputNames   ,'u_inv_out'), :);
            v_inv = states{:}(strcmp(obj.OutputNames   ,'v_inv_out'), :);
            w_inv = states{:}(strcmp(obj.OutputNames   ,'w_inv_out'), :);
            
            % True Observer States
            x_true = [
                x_inv - x
                y_inv - y
                z_inv - z
                u_inv - u
                v_inv - w
                w_inv - v
                ];
            
            % Pseudo Controls
            u_true  = [
                outputs{:}(strcmp(obj.OutputNames,'u1'), :)
                outputs{:}(strcmp(obj.OutputNames,'u2'), :)
                outputs{:}(strcmp(obj.OutputNames,'u3'), :)
                ];
            
            % True Measurements
            z_true  = [
                outputs{:}(strcmp(obj.OutputNames,'azimuth_true')  , :)
                outputs{:}(strcmp(obj.OutputNames,'elevation_true'), :)
                ];
            
            % Process Noise
            rng(2019);
            mu_p    = 0;
            std_p   = 0;
            w = normrnd(mu_p,std_p,size(u_true));
            u_obs = u_true + w;
            
            % Measurement Noise
            rng(2020);
            mu_m    = 0;
            std_m   = 0;
            v = normrnd(mu_m,std_m,size(z_true));
            z_obs = z_true + v;
            
            
            %% EKF Init
            
            % State jacobians
            F_x = obj.stateJac_x(dt);
            F_w = obj.stateJac_w(dt);            
            % Initial filter bias
            mu_b    = 0;
            rng(obj.ObserverSeed);
            b_pos   = normrnd(mu_b, obj.StdPos, [3 1]);
            b_x0    = [b_pos; -x_true(4:6,1)];            
            % Initialize state and covariane matrix
            x_0     = 	x_true(:,1) + b_x0;
            n_x     =   length(x_0);            
            % Allocate filter variables
            x_k_km1 =   nan(n_x,N);
            x_k_k   =   nan(n_x,N);
            P_k_km1 =   nan(n_x,n_x,N);
            P_k_k   =   nan(n_x,n_x,N);
            % Initialize filter variables
            x_k_km1(:,1)    = x_0;
            P_k_km1(:,:,1)  = obj.P_0;
            x_k_k(:,1)      = x_0;
            P_k_k(:,:,1)    = obj.P_0;
            % Allocate variables for post processing
            measurements    =   nan(2,N);
            std             =   nan(n_x,N);
            err_vec         =   nan(n_x,N);
            NEES            =   nan(1,N);
            NEES_pos        =   nan(1,N);
            NEES_vel        =   nan(1,N);
            P_trace         =   nan(1,N);
            P_trace_pos     =   nan(1,N);
            P_trace_vel     =   nan(1,N);
            % Initialize variables for post processing
            std(:,1)        =   sqrt(diag(obj.P_0));
            err_vec(:,1)    =   x_true(:,1) - x_0;
            P_trace(1)      =   trace(obj.P_0);
            P_trace_pos(1)  =   trace(obj.P_0(1:3,1:3));
            P_trace_vel(1)  =   trace(obj.P_0(4:6,4:6));
            
            %% EKF
            for k=2:N
                % Prediction step
                %                 x_k_km1(:,k)    =   ode4_step(@(x,u,dt)f(x,u,dt), dt, time{:}(k-1), x_k_k(:,k-1), u_obs(:,k-1), u_obs(:, k));
                x_k_km1(:,k)    = obj.stateFcn(x_k_k(:,k-1),u_obs(:,k-1),dt);
                P_k_km1(:,:,k)  = F_x * P_k_k(:,:,k-1) * F_x' + F_w * obj.Q * F_w';
                
                % Prediction measurement
                y_k_km1 = obj.measFcn(x_k_km1(:,k));
                
                % Gain
                H   =   obj.measJac(x_k_km1(:,k));
                K   =   P_k_km1(:,:,k) * H' / (H * P_k_km1(:,:,k) * H' + obj.R);
                
                % Update step
                x_k_k(:,k)      =   x_k_km1(:,k) + K * ( z_obs(:,k) - y_k_km1);
                P_k_k(:,:,k)    =   (eye(n_x) - K * H) * P_k_km1(:,:,k);
                
                
                %% Debug and Post Processing
                measurements(:,k)   =   y_k_km1;
                std(:,k)            =   sqrt(diag(P_k_k(:,:,k)));
                err_x               =   x_true(:,k) - x_k_k(:,k);
                err_vec(:,k)        =   err_x;
                NEES(k)             =   err_x' * P_k_k(:,:,k) * err_x;
                NEES_pos(k)         =   err_x(1:3)' * P_k_k(1:3,1:3,k) * err_x(1:3);
                NEES_vel(k)         =   err_x(4:6)' * P_k_k(4:6,4:6,k) * err_x(4:6);
                P_trace(k)          =   trace(P_k_k(:,:,k));
                P_trace_pos(k)      =   trace(P_k_k(1:3,1:3,k));
                P_trace_vel(k)      =   trace(P_k_k(4:6,4:6,k));
                
            end
            
            
            %% Post Processing
            %             plot(time{:},P_trace_pos);
            % Plot ture and estimated position
            %             figure(1);
            %             ax1 = subplot(3,1,1); hold on; grid on;
            %             ptrue   =   plot(time{:},x_true(1,:),'g','LineWidth',2);
            %             pest    =   plot(time{:},x_k_k(1,:),'.r','LineWidth',2);
            %             pstd    =   errorbar(time{:},x_k_k(1,:),std(1,:),'.r','LineWidth',.1);
            %             xlabel('T');ylabel('X');
            %             ax2 = subplot(3,1,2); hold on; grid on;
            %             plot(time{:},x_true(2,:),'g','LineWidth',2);
            %             plot(time{:},x_k_k(2,:),'.r','LineWidth',2);
            %             errorbar(time{:},x_k_k(2,:),std(2,:),'.r','LineWidth',.1);
            %             xlabel('T');ylabel('Y');
            %             ax3 = subplot(3,1,3); hold on; grid on;
            %             plot(time{:},x_true(3,:),'g','LineWidth',2);
            %             plot(time{:},x_k_k(3,:),'.r','LineWidth',2);
            %             errorbar(time{:},x_k_k(3,:),std(3,:),'.r','LineWidth',.1);
            %             xlabel('T');ylabel('Z');
            %             linkaxes([ax1,ax2, ax3],'x');
            %             sgtitle('True and Estimated Position');
            %             legend([ptrue(1) pest(1) pstd], {'True Position','Estimated Position','Standard Deviation'});
            
            
            
            %% Cost Functions            
            
            nTimeParameters = obj.NPhases + 1;
            nInputValues = sum(cellfun(@numel,[states;controls;outputs])) + nTimeParameters;
            
            % Allocate cost functions
            j_time  = 0;
            j_obs   = 0;
            
            % Allocate cost jacobians
            j_time_jac  = zeros(1,nInputValues);
            j_obs_jac   = zeros(1,nInputValues);
            
            
            % Cost functions
            j_time  = time{:}(end);
            j_obs   = sum(P_trace_pos);                        
            
            
            % Cost function jacobians
            j_time_jac(end) = 1;             
            
            
            % Total Cost Function
            j       = j_time + j_obs;
            j_jac   = j_time_jac; 
            
            
            
            
            
            
            
            %% OLD
            
            %     % get magnitudes and weights
            %
            %     scaleNoise = 1/obj.NoiseMagnitude;
            %
            %     scaleMass  = 1/obj.MassMagnitude;
            %
            %     scaleTime  = obj.Problem.Phases(end).FinalTime.Scaling;
            %
            %     scaleCost  = obj.CostScaling;
            %
            %     w          = obj.FuelNoiseRatio;
            %
            %
            %
            %     % get sizes and index for final mass
            %
            %     nTimeParameters = obj.NPhases + 1;
            %
            %     nInputValues = sum(cellfun(@numel,[x;u;out])) + nTimeParameters;
            %
            %     nControlValuesLastPhase = numel(u{end});
            %
            %     idxFinalMass = nInputValues - nTimeParameters - nControlValuesLastPhase;
            %
            %
            %
            %     % allocate cost
            %
            %     j_noise = 0;
            %
            %     j_mass  = 0;
            %
            %     j_time  = 0;
            %
            %
            %
            %     % allocate jacobians
            %
            %     j_noise_jac = zeros(1,nInputValues);
            %
            %     j_mass_jac  = zeros(1,nInputValues);
            %
            %     j_time_jac  = zeros(1,nInputValues);
            %
            %
            %
            %     if obj.OnlyMinimizeFinalTime
            %
            %         j_time          = t{end}(end);
            %
            %         j_time_jac(end) = 1;
            %
            %     else
            %
            %         % check if we need to compute the noise cost-function at all
            %
            %         if w>0
            %
            %             [j_noise,dj_noise_dx,dj_noise_du,dj_noise_dt,dj_noise_dout] = obj.MinNoise(x,u,t,out);
            %
            %             j_noise_jac = [dj_noise_dout; dj_noise_dx; dj_noise_du; dj_noise_dt];
            %
            %             j_noise_jac = cellfun(@(x)x(:).',j_noise_jac,'UniformOutput',false);
            %
            %             j_noise_jac = horzcat(j_noise_jac{:});
            %
            %         end
            %
            %
            %
            %         % compute mass objective (unscaled)
            %
            %         j_mass                   = - x{end}(end,end);
            %
            %         j_mass_jac(idxFinalMass) = - 1;
            %
            %     end
            %
            %
            %
            %     % compute objective including scaling factors
            %
            %     j     = scaleCost * (scaleNoise*j_noise*w     + scaleMass*j_mass*(1-w)     + scaleTime*j_time    );
            %
            %     j_jac = scaleCost * (scaleNoise*j_noise_jac*w + scaleMass*j_mass_jac*(1-w) + scaleTime*j_time_jac);
            
        end
        
        
        
        
        
        
        
    end
end

