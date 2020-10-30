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
    
    methods (Access = protected)
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
            
            if real(x)>0
                az = atan(y/x);
            elseif real(x)<0
                if real(y)>0
                    az = atan(y/x) + pi;
                else
                    az = pi;
                end
            else
                if real(y)>0
                    az = pi/2;
                else
                    y = -pi/2;
                end
            end
            
            el = atan(-z/sqrt(x^2 + y^2));
            
            ret = [
                az
                el];
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
             
      
        function [idxFilterValues] = getFilterIndices(obj)
            
            stateOffset = obj.NOutputs;
            
            idxFilterValues = [
                % Outputs
                find(strcmp(obj.OutputNames,'u_inv_out'))                   % #1
                find(strcmp(obj.OutputNames,'v_inv_out'))                   % #2
                find(strcmp(obj.OutputNames,'w_inv_out'))                   % #3
                find(strcmp(obj.OutputNames,'u1'))                          % #4
                find(strcmp(obj.OutputNames,'u2'))                          % #5
                find(strcmp(obj.OutputNames,'u3'))                          % #6
                
                % States                
                find(strcmp(obj.StateNames ,'x'))       + stateOffset       % #7
                find(strcmp(obj.StateNames ,'z'))       + stateOffset       % #8
                find(strcmp(obj.StateNames ,'y'))       + stateOffset       % #9
                find(strcmp(obj.StateNames ,'u'))       + stateOffset       % #10
                find(strcmp(obj.StateNames ,'v'))       + stateOffset       % #11
                find(strcmp(obj.StateNames ,'w'))       + stateOffset       % #12
                find(strcmp(obj.StateNames ,'x_inv'))   + stateOffset       % #13
                find(strcmp(obj.StateNames ,'y_inv'))   + stateOffset       % #14
                find(strcmp(obj.StateNames ,'z_inv'))   + stateOffset       % #15
                ];
        end
        
        function [x_true, u_obs] =...
                unwrapFilterInputs(obj, inputValues, idxFilterValues, N)
            
            
            inputs = reshape(inputValues,[],N);
            filterInputs = inputs(idxFilterValues,:);
            states_def = filterInputs(7:12,:);
            states_inv = [
                filterInputs(13:15,:)
                filterInputs(1:3,:)];            
            x_true = states_inv - states_def;
            u_true = filterInputs(4:6, :);
%             z_true = filterInputs(7:8, :);
            
            % Process Noise
%             rng(2019);
%             mu_p    = 0;
%             std_p   = 0;
%             w = normrnd(mu_p,std_p,size(u_true));
            u_obs = u_true;% + w;
            
            % Measurement Noise
%             rng(2020);
%             mu_m    = 0;
%             std_m   = 0;
%             v = normrnd(mu_m,std_m,size(z_true));
%             z_obs = z_true;% + v;
            
        end
        
        function [j_ekf] = EKF_run(obj,...
                inputValues, idxFilterValues, N, dt)
            
            % EKF Init
            [x_true, u_obs] = obj.unwrapFilterInputs(...
                inputValues, idxFilterValues, N);
            
            %  State Jacobians
            F_x = obj.stateJac_x(dt);
            F_w = obj.stateJac_w(dt);
            % Initial filter bias
            mu_b    = 0;
            rng(obj.ObserverSeed);
            b_pos   = normrnd(mu_b, obj.StdPos, [3 1]);
            b_x0    = [b_pos; -x_true(4:6,1)];
            % Initialize state and covariane matrix
            x_0     = 	x_true(:,1);% + b_x0;
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
            P_trace_pos =   zeros(1,N);
%             P_trace     =   zeros(1,N);
            % Initialize variables for post processing
            P_trace_pos(1)  =   trace(obj.P_0(1:3,1:3));
%             P_trace(1)      =   trace(obj.P_0);
            
            % EKF run
            for k=2:N
                % Prediction step
                x_k_km1(:,k)    = obj.stateFcn(x_k_k(:,k-1),u_obs(:,k-1),dt);
                P_k_km1(:,:,k)  = F_x * P_k_k(:,:,k-1) * F_x' + F_w * obj.Q * F_w';
                
                % Prediction measurement
                y_k_km1 = obj.measFcn(x_k_km1(:,k));
                z_obs   = obj.measFcn(x_true(:,k));
                
                % Gain
                H   =   obj.measJac(x_k_km1(:,k));
                K   =   P_k_km1(:,:,k) * H' / (H * P_k_km1(:,:,k) * H' + obj.R);
                
                % Update step
                x_k_k(:,k)      =   x_k_km1(:,k) + K * ( z_obs - y_k_km1);
                P_k_k(:,:,k)    =   (eye(n_x) - K * H) * P_k_km1(:,:,k);
                
                
                % Debug and Post Processing
                %                 measurements(:,k)   =   y_k_km1; std(:,k)
                %                 =   sqrt(diag(P_k_k(:,:,k))); err_x
                %                 =   x_true(:,k) - x_k_k(:,k);
                %                 err_vec(:,k)        =   err_x; NEES(k)
                %                 =   err_x' * P_k_k(:,:,k) * err_x;
                %                 NEES_pos(k)         =   err_x(1:3)' *
                %                 P_k_k(1:3,1:3,k) * err_x(1:3);
                %                 NEES_vel(k)         =   err_x(4:6)' *
                %                 P_k_k(4:6,4:6,k) * err_x(4:6); P_trace(k)
                %                 =   trace(P_k_k(:,:,k));
                P_trace_pos(k)  =   trace(P_k_k(1:3,1:3,k));
%                 P_trace(k)      =   trace(P_k_k(:,:,k));
                %                 P_trace_vel(k)      =
                %                 trace(P_k_k(4:6,4:6,k));
            end
            
            j_ekf = P_trace_pos(end);
            
            
%             figure;
%             plot(P_trace_pos);
            % Plot ture and estimated position
%             figure();
%             ax1 = subplot(3,1,1); hold on; grid on;
%             ptrue   =   plot(x_true(1,:),'g','LineWidth',2);
%             pest    =   plot(x_k_k(1,:),'.r','LineWidth',2);
% %             pstd    =   errorbar(x_k_k(1,:),std(1,:),'.r','LineWidth',.1);
%             xlabel('T');ylabel('X');
%             ax2 = subplot(3,1,2); hold on; grid on;
%             plot(x_true(2,:),'g','LineWidth',2);
%             plot(x_k_k(2,:),'.r','LineWidth',2);
% %             errorbar(x_k_k(2,:),std(2,:),'.r','LineWidth',.1);
%             xlabel('T');ylabel('Y');
%             ax3 = subplot(3,1,3); hold on; grid on;
%             plot(x_true(3,:),'g','LineWidth',2);
%             plot(x_k_k(3,:),'.r','LineWidth',2);
% %             errorbar(x_k_k(3,:),std(3,:),'.r','LineWidth',.1);
%             xlabel('T');ylabel('Z');
%             linkaxes([ax1,ax2, ax3],'x');
%             sgtitle('True and Estimated Position');
%             legend([ptrue(1) pest(1)], {'True Position','Estimated Position'});
            
        end
        
        function [jac_ekf] = ComplexStepDerivation(obj,...
                inputValues, idxFilterValues, N, dt)                   
      
            
            nInputValues    = numel(inputValues);
            nTimeParameters = obj.NPhases + 1;
            jac_ekf = zeros(1,nInputValues + nTimeParameters);
            
            % Complex Step Pertubation            
            h = 1e-50; %% ToDo integrate into setup.observabilityconfig            
            
            for i=1:nInputValues
                
                inputValues_pert    = inputValues;
                inputValues_pert(i) = inputValues_pert(i) + 1i*h;
                jac_ekf(i)          =...
                    imag(obj.EKF_run(...
                    inputValues_pert, idxFilterValues, N, dt)) / h;
            end
        end
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
            ret = obj.Setup.Solver.TimeCostScaling;
        end
        
        function ret = get.ObsCostScaling(obj)
            ret = obj.Setup.Solver.ObsCostScaling;
        end
        
        
        function [j,j_jac] = ObservabilityCostFcn(obj, varargin)
            
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
                
                
                % Parameters
                
                str.input(cnt,1).m = obj.NPhases+1;
                
                str.input(cnt,1).n = 1;
                
                str.input(cnt,1).name = 'parameters';
                
                str.input(cnt,1).argnames = obj.Parameters;
                
                str.input(cnt,1).type = 'PARAMETER';
                
                str.input(cnt,1).groupindex = 0;
                
                
                % Constraintvalue
                
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
                               
            
            %% Cost Function
            
            % Data preparation
            dt              = diff(time{:}(1:2));
            N               = nTimeStepsPerPhase;            
            nTimeParameters = obj.NPhases + 1;
            inputValues     = reshape([outputs{:};states{:};controls{:}],1,[]);
            nInputValues    = numel(inputValues) + nTimeParameters;            
            idxFilterValues = obj.getFilterIndices();
            
            % Cost functions
            j_time  = time{:}(end);
            j_obs   = obj.EKF_run(inputValues, idxFilterValues, N, dt);
            
%             % Allocate jacobians
%                 j_time_jac  = zeros(1,nInputValues);    

            j =...
                (j_time  * obj.TimeCostScaling + ...
                j_obs   * obj.ObsCostScaling) / 1000;
            
            % Compute jacobians
            if nargout>1
                j_jac = obj.der(j,varargin{:});
            end
            
            
%             
%             j_time_jac(end) = 1;
%             if nargout>1
%                 tic
%                 j_obs_jac = obj.ComplexStepDerivation(...
%                     inputValues, idxFilterValues, N, dt);
%                 toc
%             end
%             % Total Cost Function
            
%             if nargout>1
%                 j_jac = ...
%                     j_time_jac  * obj.TimeCostScaling + ...
%                     j_obs_jac   * obj.ObsCostScaling;
        end
        
        function jac = der(obj,j,varargin)
            nInputs = nargin-2;
            jac = cell(nInputs,1);
%             h = 1e-50;
            h = sqrt(eps);
            for i=1:nInputs
                nDer = numel(varargin{i});
                jac{i} = zeros(nDer,1);
                for k=1:nDer
%                     varargin{i}(k) = varargin{i}(k) + 1i*h;
                    varargin{i}(k) = varargin{i}(k) + max(1,abs(varargin{i}(k)))*h;
                    j_pert = obj.ObservabilityCostFcn(varargin{:});
%                     jac{i}(k) = imag(j_pert)/h;
                    jac{i}(k) = (j_pert-j)/(max(1,abs(varargin{i}(k)))*h);
%                     varargin{i}(k) = real(varargin{i}(k));
                    varargin{i}(k) = varargin{i}(k) - max(1,abs(varargin{i}(k)))*h;
                end
            end
            jac = vertcat(jac{:})';
        end
            
            end
            
       
            
        end        



