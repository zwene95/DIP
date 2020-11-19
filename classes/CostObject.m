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
        WeightTime
        WeightMiss
        WeightCov;
        WeightRMSE;
        Parallel;
        GPU;
    end
    
    methods % GetProperties
        
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
                obj.Problem.Parameters(2).Name};
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
        
        function ret = get.WeightTime(obj)
            ret = obj.Setup.Solver.CostWeightTime;
        end
        
        function ret = get.WeightMiss(obj)
            ret = obj.Setup.Solver.CostWeightMiss;
        end
        
        function ret = get.WeightCov(obj)
            ret = obj.Setup.Solver.CostWeightCov;
        end
        
        function ret = get.WeightRMSE(obj)
            ret = obj.Setup.Solver.CostWeightRMSE;
        end
        
        function ret = get.Parallel(obj)
            ret = obj.Setup.Solver.Parallel;
        end
        
        function ret = get.GPU(obj)
            ret = obj.Setup.Solver.GPU;
        end
        
        function [j,j_jac] = CostFunction(obj, varargin)
            
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
                
                str.output(1).argnames = {'CostParameter'};
                
                str.output(1).type = 'VALUE';
                
                str.output(1).jac_sparsity = ones(1,sum([str.input.m].*[str.input.n]));
                
                str.output(1).hess_sparsity = [];
                
                
                
                % Info
                
                str.info.Date = '20-Oct-2020';
                
                str.info.Computer = 'None';
                
                str.info.MATLAB = 'R2020 Update ?';
                
                str.info.Hessian = false;
                
                
                
                % Other Info
                
                str.name = 'CostParameter';
                
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
                
                
                % Controls
                
                if size(varargin{iInput+2},1) ~= obj.NControls
                    
                    error('Dimensions do not match');
                    
                end
                
                if size(varargin{iInput+2},2) ~= obj.NTimeStepsPerPhase
                    
                    error('Dimensions do not match');
                    
                end
                
                controls{cntPhase} = varargin{iInput+2};
                
                % Time
                time{cntPhase} = linspace(varargin{end}(cntPhase),varargin{end}(cntPhase+1),nTimeStepsPerPhase(cntPhase));
                
                % Increment phase counter
                cntPhase = cntPhase + 1;
            end
            
            %% Data preparation
            nOuputs             = obj.NOutputs;
            nStates             = obj.NStates;
            nTimeStepsPerPhase  = obj.NTimeStepsPerPhase;
            nInputs = nTimeStepsPerPhase * (...
                nOuputs + nStates + obj.NControls) +...
                obj.NPhases + 1;
            
            % Allocate costs
            j_time = 0;
            j_miss = 0;
            j_obs  = 0;
            % Allocate jacobians
            j_time_jac = zeros(1,nInputs);
            j_miss_jac = zeros(1,nInputs);
            j_obs_jac  = zeros(1,nInputs);
            
            % Defender States
            def_pos = [
                states{:}(strcmp(obj.StateNames,'x'), :)
                states{:}(strcmp(obj.StateNames,'y'), :)
                states{:}(strcmp(obj.StateNames,'z'), :)];
            
            % Invader States
            inv_pos = [
                states{:}(strcmp(obj.StateNames  ,'x_inv'), :)
                states{:}(strcmp(obj.StateNames  ,'y_inv'), :)
                states{:}(strcmp(obj.StateNames  ,'z_inv'), :)];
            
            %% Time Cost Function
            ScalingTime = 10;
            j_time(:) = time{end}(end)...
                / ScalingTime...
                * obj.WeightTime;
            j_time_jac(end) = 1 ... 
                / ScalingTime...
                * obj.WeightTime;
            
            %% Obsevability Cost Function
            if ( (obj.WeightCov > 0) || (obj.WeightRMSE > 0) )
                [j_obs(:), j_obs_jac(:)] = obj.ObservabilityCostFcn(varargin{:});
            end
            
            %% Missdistance Cost Function
            ScalingMiss = sum((inv_pos(:,1) - def_pos(:,1)).^2);
            j_miss(:)   = sum((inv_pos(:,end) - def_pos(:,end)).^2)...
                / ScalingMiss...
                * obj.WeightMiss;
            
            jac_mag = 2 * (inv_pos(:,end) - def_pos(:,end))...                
                / ScalingMiss...
                * obj.WeightMiss;
            
            idx_def = nTimeStepsPerPhase * nOuputs  + ...
                (nTimeStepsPerPhase -1) * nStates   + ...
                find(strcmp(obj.StateNames,'x'));
            idx_inv = nTimeStepsPerPhase * nOuputs  + ...
                (nTimeStepsPerPhase -1) * nStates   + ...
                find(strcmp(obj.StateNames,'x_inv'));
            
            for i=1:3
                j_miss_jac(idx_def + (i-1)) = -jac_mag(i);
                j_miss_jac(idx_inv + (i-1)) = +jac_mag(i);
            end
            
            
            %% Total cost function and jacobian
            j       = j_time + j_miss + j_obs;
            j_jac   = j_time_jac + j_miss_jac + j_obs_jac;
            %             j       = j_time + j_obs;
            %             j_jac   = j_time_jac + j_obs_jac;
            
        end % EoCostFunction
        
        
    end % EoMethods
    
    
    methods (Access = protected)
        function ret = stateJac_x(obj,dt)
            ret =  [
                1 , 0 , 0 , dt ,  0 , 0
                0 , 1 , 0 ,  0 , dt , 0
                0 , 0 , 1 ,  0 ,  0 , dt
                0 , 0 , 0 ,  1 ,  0 , 0
                0 , 0 , 0 ,  0 ,  1 , 0
                0 , 0 , 0 ,  0 ,  0 , 1];
        end
        
        function ret = stateJac_w(obj,dt)
            ret =  [
                0.5*dt^2, 0         , 0
                0       , 0.5*dt^2  , 0
                0       , 0         , 0.5*dt^2
                dt      , 0         , 0
                0       , dt        , 0
                0       , 0         , dt];
        end
        
        function ret = atan2_fcn(obj, y, x)
            
            if real(x)>0
                ret = atan(y/x);
            elseif real(x)<0
                if real(y)>0
                    ret = atan(y/x) + pi;
                elseif real(y) < 0
                    ret = atan(y/x) - pi;
                else
                    ret = pi;
                end
            else
                if real(y)>0
                    ret = pi/2;
                elseif real(y)<0
                    ret = -pi/2;
                else
                    ret = 0;
                end
            end
        end
        
        function ret = measFcn(obj, states)
            
            x = states(1);
            y = states(2);
            z = states(3);
            
            az = atan2(y,x);
            el = atan2(-z,sqrt(x^2 + y^2));
            
            %             az = obj.atan2_fcn(y,x);
            %             el = obj.atan2_fcn(-z,sqrt(x^2 + y^2));
            
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
        
        function jac = Jacobian(obj,j,varargin)
            nInputs = nargin-2;
            jac = cell(nInputs,1);
            h = sqrt(eps);
            for i=1:nInputs
                nDer = numel(varargin{i});
                jac{i} = zeros(nDer,1);
                %                 inputs_pert = varargin;
                %                 jac_col = zeros(nDer,1);
                %                 x = varargin;
                for k=1:nDer
                    % Parfor finite differences
                    %                     x{i}(k) = varargin{i}(k) + max(1,abs(varargin{i}(k)))*h;
                    %                     j_pert = obj.ObservabilityCostFcn(x{:});
                    %                     jac_col(k) = (j_pert-j)/(max(1,abs(varargin{i}(k)))*h);
                    %                     x{i}(k) = varargin{i}(k);
                    % Comlplex step
                    %                     varargin{i}(k) = varargin{i}(k) + 1i*h;
                    %                     j_pert = obj.ObservabilityCostFcn(varargin{:});
                    %                     jac{i}(k) = imag(j_pert)/h;
                    %                     varargin{i}(k) = real(varargin{i}(k));
                    % Finite differences
                    pert = max(1,abs(varargin{i}(k)))*h;
                    varargin{i}(k) = varargin{i}(k) + pert;
                    j_pert = obj.ObservabilityCostFcn(varargin{:});
                    jac{i}(k) = (j_pert-j)/(max(1,abs(varargin{i}(k)))*h);
                    %                     assignin('base','varargin_seq',varargin);
                    %                     assignin('base','j_seq',j_pert);
                    varargin{i}(k) = varargin{i}(k) - pert;
                end
                % Parfor
                %                 jac{i}(:) = jac_col;
            end
            jac = vertcat(jac{:})';
        end
        
        function data = unwrapInputs(obj,varargin)
            % Unwrap inputs for parallelized jacobian computation
            nInputs = nargin-1;
            nT = obj.NTimeStepsPerPhase;
            data = nan(1,(obj.NOutputs + obj.NStates + obj.NControls) * nT + obj.NPhases + 1);
            idx_start = 1;
            for i=1:nInputs
                tmp = reshape(varargin{i},1,[]);
                idx_end = idx_start + numel(tmp) - 1;
                data(idx_start:idx_end) = tmp;
                idx_start = idx_end + 1;
            end
        end
        
        function inputs = wrapInputs(obj,data)
        % Wrap inputs for parallelized jacobian computation
            nT = obj.NTimeStepsPerPhase;
            outputs     = cell(obj.NPhases,1);
            states      = cell(obj.NPhases,1);
            controls    = cell(obj.NPhases,1);
            parameters  = cell(obj.NPhases,1);
            
            idx_out_0   = 1;
            idx_out_f   = idx_out_0 + obj.NOutputs*nT - 1;
            idx_sta_0   = idx_out_f + 1;
            idx_sta_f   = idx_sta_0 + obj.NStates*nT - 1;
            idx_ctr_0   = idx_sta_f + 1;
            idx_ctr_f   = idx_ctr_0 + obj.NControls*nT - 1;
            
            outputs{:}   = reshape(data(idx_out_0:idx_out_f),obj.NOutputs,[]);
            states{:}    = reshape(data(idx_sta_0:idx_sta_f),obj.NStates,[]);
            controls{:}  = reshape(data(idx_ctr_0:idx_ctr_f),obj.NControls,[]);
            parameters{:}= data(end-1:end)';
            
            inputs = [outputs states controls parameters];
            
        end        
        
        function jac = Jacobian_par(obj,j,varargin)
        % Parallelized jacobian computation
            data = obj.unwrapInputs(varargin{:});
            n = numel(data);
            if obj.GPU
                jac = gpuArray(nan(1,n));
            else
                jac = zeros(1,n);
            end
            h = sqrt(eps);
            parfor k = 1:n
                if obj.GPU
                    data_pert = gpuArray(data);
                else
                    data_pert = data;
                end
                data_pert(k) = data(k) + max(1,abs(data(k))) * h;
                varargin_pert = obj.wrapInputs(data_pert);
                j_pert  = obj.ObservabilityCostFcn(varargin_pert{:});
                jac(k)  = (j_pert-j)/(max(1,abs(data_pert(k)))*h);
            end
            if obj.GPU
                jac = gather(jac);
            end
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
        
        function [j_obs, j_obs_jac] = ObservabilityCostFcn(obj,varargin)
            
            nTimeStepsPerPhase = obj.NTimeStepsPerPhase;
            time = linspace(varargin{end}(1),varargin{end}(2),nTimeStepsPerPhase);
            dt = diff(time(1:2));
            N  = nTimeStepsPerPhase - 0;
            
            % Defender States
            x_def = [
                varargin{2}(strcmp(obj.StateNames,'x'), :)
                varargin{2}(strcmp(obj.StateNames,'y'), :)
                varargin{2}(strcmp(obj.StateNames,'z'), :)
                varargin{2}(strcmp(obj.StateNames,'u'), :)
                varargin{2}(strcmp(obj.StateNames,'v'), :)
                varargin{2}(strcmp(obj.StateNames,'w'), :)];
            
            % Invader States
            x_inv = [
                varargin{2}(strcmp(obj.StateNames  ,'x_inv'), :)
                varargin{2}(strcmp(obj.StateNames  ,'y_inv'), :)
                varargin{2}(strcmp(obj.StateNames  ,'z_inv'), :)
                varargin{1}(strcmp(obj.OutputNames ,'u_inv_out'), :)
                varargin{1}(strcmp(obj.OutputNames ,'v_inv_out'), :)
                varargin{1}(strcmp(obj.OutputNames ,'w_inv_out'), :)];
            
            
            % Filter states
            x_true = x_inv - x_def;
            
            % Pseudo Controls
            u_true  = [
                varargin{1}(strcmp(obj.OutputNames,'u1'), :)
                varargin{1}(strcmp(obj.OutputNames,'u2'), :)
                varargin{1}(strcmp(obj.OutputNames,'u3'), :)];
            
            % Process Noise
            rng(2019);
            mu_p    = 0;
            std_p   = 0;
            w = normrnd(mu_p,std_p,size(u_true));
            u_obs = u_true + w;
            
            % Measurement Noise
            %             rng(2020);
            %             mu_m    = 0;
            %             std_m   = 0;
            %             v = normrnd(mu_m,std_m,size(z_true));
            %             z_obs = z_true + v;
            
            %  State Jacobians
            F_x = obj.stateJac_x(dt);
            F_w = obj.stateJac_w(dt);
            % Initial filter bias
            mu_b    = 0;
            rng(obj.ObserverSeed);
            b_pos   = normrnd(mu_b, obj.StdPos, [3 1]);
            b_x0    = [b_pos; -x_true(4:6,1)] * 0;
            % Initialize state and covariane matrix
            x_0     = 	x_true(:,1) + b_x0;
            n_x     =   length(x_0);
            % Allocate filter variables
            if obj.GPU
                x_k_km1 =   gpuArray(nan(n_x,N));
                x_k_k   =   gpuArray(nan(n_x,N));
                P_k_km1 =   gpuArray(nan(n_x,n_x,N));
                P_k_k   =   gpuArray(nan(n_x,n_x,N));
            else
                x_k_km1 =   nan(n_x,N);
                x_k_k   =   nan(n_x,N);
                P_k_km1 =   nan(n_x,n_x,N);
                P_k_k   =   nan(n_x,n_x,N);
            end
            % Initialize filter variables
            x_k_km1(:,1)    = x_0;
            P_k_km1(:,:,1)  = obj.P_0;
            x_k_k(:,1)      = x_0;
            P_k_k(:,:,1)    = obj.P_0;
            % Allocate variables for post processing
            if obj.GPU
                P_trace_pos = gpuArray(zeros(1,N));
            else
                P_trace_pos = zeros(1,N);
                P_trace     = zeros(1,N);
                NEES_pos    = zeros(1,N);
                posErr_vec  = zeros(3,N);
            end
            % Initialize variables for post processing
            P_trace_pos(1)  =   trace(obj.P_0(1:3,1:3));
            posErr_vec(:,1) =   x_true(1:3,1) - x_k_k(1:3,1);
            P_trace(1)      =   trace(obj.P_0);
            
            % Run EKF
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
                err_x           = x_true(:,k) - x_k_k(:,k);
                posErr_vec(:,k) = err_x(1:3)' * err_x(1:3);
                NEES_pos(k)  = err_x(1:3)' * P_k_k(1:3,1:3,k) * err_x(1:3);
                P_trace_pos(k)  =   trace(P_k_k(1:3,1:3,k));
                P_trace(k)      =   trace(P_k_k(:,:,k));
            end
            
            % Cost functions
            if obj.GPU
                error('GPU usage no longer supported');
                %                     j_obs = sum(gather(P_trace_pos));
            else
                % Observability cost
                j_obs(:) = sum(P_trace_pos) * obj.WeightCov ...
                    + sum(posErr_vec(:,end)) * obj.WeightRMSE;
                % Observability jacobian
                if nargout>1
                    tic
                    if obj.Parallel
                        j_obs_jac = obj.Jacobian_par(j_obs,varargin{:});
                    else
                        j_obs_jac = obj.Jacobian(j_obs,varargin{:});
                    end
                    toc
                end
            end
        end % EoObservabilityCostFcn        
        
        
    
        
        
    end % E0Methods
end % EoClass



%% Ablage

% function jac = der(obj,j,varargin)
% nInputs = nargin-2;
% jac = cell(nInputs,1);
% %             h = 1e-50;
% h = sqrt(eps);
% for i=1:nInputs
%     nDer = numel(varargin{i});
%     jac{i} = zeros(nDer,1);
%     for k=1:nDer
%         %                     varargin{i}(k) = varargin{i}(k) + 1i*h;
%         varargin{i}(k) = varargin{i}(k) + max(1,abs(varargin{i}(k)))*h;
%         j_pert = obj.ObservabilityCostFcn(varargin{:});
%         %                     jac{i}(k) = imag(j_pert)/h;
%         jac{i}(k) = (j_pert-j)/(max(1,abs(varargin{i}(k)))*h);
%         %                     varargin{i}(k) = real(varargin{i}(k));
%         varargin{i}(k) = varargin{i}(k) - max(1,abs(varargin{i}(k)))*h;
%     end
% end
% jac = vertcat(jac{:})';
% end




%                 j_jac = obj.Jacobian(j,varargin{:});
%                 data = obj.unwrapInputs(varargin{:});
%                 varargin_test = obj.wrapInputs(data);
%                 j_jac2 = obj.Jacobian(j,varargin_test{:});

%                 j_jac = obj.Jacobian(j,varargin{:});
%                 j_jac2 = obj.Jacobian2(j,varargin{:}); %%%%%%%%
%                 debug = max(abs(j_jac-j_jac2));
%                 fprintf('Max. error: %s\n',debug);
%                 numel(find(j_jac-j_jac2))





% function [jac_par,jac_seq] = debug_Jacobians(obj,varargin)
% data_par = varargin;
% data_seq = varargin;
%
% j = obj.ObservabilityCostFcn(varargin{:});
% %             jac_par = obj.Jacobian2(j,data_par{:});
% %             jac_seq = obj.Jacobian(j,data_seq{:});
%
% h = sqrt(eps);
% i = 4;
% k = 1;
%
% data_seq{i}(k) = data_seq{i}(k) + max(1,abs(data_seq{i}(k)))*h;
% j_seq = obj.ObservabilityCostFcn(data_seq{:});
% jac_seq = (j_seq-j)/(max(1,abs(data_seq{i}(k)))*h);
%
%
% data_pert = obj.unwrapInputs(data_par{:});
% data_pert(919) = data_pert(919) + max(1,abs(data_pert(919))) * h;
% varargin_pert = obj.wrapInputs(data_pert);
% j_par  = obj.ObservabilityCostFcn(varargin_pert{:});
% jac_par  = (j_par-j)/(max(1,abs(data_pert(919)))*h);
%
% end



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


