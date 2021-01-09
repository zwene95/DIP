classdef EKF_Object
    %EKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Time;
        States;
        Controls;
        Measurements;
        StepTime;
        Interpolate;
        Sigma_Q;
        Sigma_R;
        Sigma_w;
        Sigma_v;
        Sigma_x0;
        Sigma_P0_pos;
        Sigma_P0_vel;
    end
    
    properties (Dependent)
        Results;
    end
    
    methods
        
        function r = get.Results(obj)
            % State estimation with EKF            
            
            %% Preprocessing EKF
            
            nDat = length(obj.Time);
            iDat = linspace(1,nDat,nDat);            
            %Interpolation
            if obj.Interpolate                
                nEKF = ceil(obj.Time(end)/obj.StepTime);                
                iEKF = linspace(1,nDat,nEKF);
                % Interpolate input data
                r.Time  = interp1(iDat,obj.Time',iEKF)';
                r.x_true  = interp1(iDat,obj.States',iEKF)';            
                r.u_true  = interp1(iDat,obj.Controls',iEKF)';
                r.z_true   = interp1(iDat,obj.Measurements',iEKF)';
                dt      = obj.StepTime;
            else                                
                r.x_true  = obj.States;
                r.u_true  = obj.Controls;
                r.z_true   = obj.Measurements;
                r.nFil    = nDat;
                dt      = diff(obj.Time(1:2));
            end
            
            % Noise consideration
            % Define zero-mean Gaussian distibuted process and measurement 
            % noise
            rng(2019);
            w = normrnd(0,obj.Sigma_w,size(r.u_true));
            rng(2020);
            v = normrnd(0,obj.Sigma_v,size(r.z_true));            
            % Add noise to true controls and measurements
            u = r.u_true + w;
            r.z = r.z_true + v;
            
            %% Init EKF
            
            % Get number of states and outputs
            n_x = numel(r.x_true(:,1));
            n_y = numel(r.z_true(:,1));
            
            % Get constant state and disturbance jacobians
            F_x = obj.StateJac_x(dt);
            F_w = obj.StateJac_w(dt);   
            
            % Initial position error (bias)
            rng(9999);
            pos_bias = normrnd(0, obj.Sigma_x0, [3 1]);
            x0_bias  = [pos_bias; -r.x_true(4:6,1)];
            
            % Initial states and covariance
            x0  = r.x_true(:,1) + x0_bias;
            P0  = [
                eye(3)*obj.Sigma_P0_pos^2,zeros(3)
                zeros(3),eye(3)*obj.Sigma_P0_vel^2];
            % Setup filter variables
            x_k_km1 =   nan(n_x,nEKF);
            x_k_k   =   nan(n_x,nEKF);
            P_k_km1 =   nan(n_x,n_x,nEKF);
            P_k_k   =   nan(n_x,n_x,nEKF);
            % Initialize filter variables
            x_k_km1(:,1)    = x0;
            x_k_k(:,1)      = x0;
            P_k_km1(:,:,1)  = P0;
            P_k_k(:,:,1)    = P0;
            % Setup process and measurement covariances
            Q = eye(3)*obj.Sigma_Q^2;
            R = eye(2)*obj.Sigma_R^2;
            
            % Setup postprocessing variables
            r.Measurements  = nan(n_y, nEKF);
            r.Sigma         = nan(n_x, nEKF);
            r.Error         = nan(n_x, nEKF);                               % Filter estimation error
            r.NEES          = nan(1,nEKF);                                % Normalized estimation error squared combined
            r.NEES_pos      = nan(1,nEKF);                                % Normalized estimation eror squared position only
            r.NEES_vel      = nan(1,nEKF);                                % Normalized estimation error squared velocity only
            r.P_trace       = nan(1,nEKF);                                % Covariance trace
            r.P_trace_pos   = nan(1,nEKF);                                % Covariance trace position only
            r.P_trace_vel   = nan(1,nEKF);                                % Covariance trace velocity only
            % Initialize variables for post processing
            r.Sigma(:,1)     =   sqrt(diag(P0));
            r.Error(:,1)     =   -x0_bias;
            r.P_trace(1)     =   trace(P0);
            r.P_trace_pos(1) =   trace(P0(1:3,1:3));
            r.P_trace_vel(1) =   trace(P0(4:6,4:6));
            
            %% Run EKF
            tic
            for k = 2:nEKF
            
                % Prediction step                
                % State Propagation
                x_k_km1(:,k) = obj.StateFcn(x_k_k(:,k-1),u(:,k-1),dt);
                % Covariance Propagation
                P_k_km1(:,:,k) =...
                    F_x * P_k_k(:,:,k-1) * F_x' + F_w * Q * F_w';
                % Predict measurements
                y_k_km1 = obj.MeasFcn(x_k_km1(:,k));
                
                % Kalman Gain
                H = obj.MeasJac_x(x_k_km1(:,k));
                K = P_k_km1(:,:,k) * H' / (H * P_k_km1(:,:,k) * H' + R);
                
                % Correction step
                x_k_k(:,k)   =   x_k_km1(:,k) + K * ( r.z(:,k) - y_k_km1);
                P_k_k(:,:,k) =   (eye(n_x) - K * H) * P_k_km1(:,:,k);
                
                % Post processing variables
                r.Measurements(:,k) = y_k_km1;
                r.Sigma(:,k)        = sqrt(diag(P_k_k(:,:,k)));
                r.Error(:,k)        = r.x_true(:,k) - x_k_k(:,k);
                r.NEES(k)           = r.Error(:,k)' * P_k_k(:,:,k) * r.Error(:,k);
                r.NEES_pos(k)       = r.Error(1:3,k)' * P_k_k(1:3,1:3,k) * r.Error(1:3,k);
                r.NEES_vel(k)       = r.Error(4:6,k)' * P_k_k(4:6,4:6,k) * r.Error(4:6,k);
                r.P_trace(k)        = trace(P_k_k(:,:,k));
                r.P_trace_pos(k)    = trace(P_k_k(1:3,1:3,k));
                r.P_trace_vel(k)    = trace(P_k_k(4:6,4:6,k));
            end
            
            r.x_k_k = x_k_k;
            toc
            
        end
    end
    
    methods (Access=protected)
        
        function ret = StateFcn(obj,OldState,OldControl,dt)
            % Compute state prediction step x_k+1 = x_k * PHI + u_k * Gamma
            
            ret = [
                1 , 0 , 0 , dt ,  0 , 0
                0 , 1 , 0 ,  0 , dt , 0
                0 , 0 , 1 ,  0 ,  0 , dt
                0 , 0 , 0 ,  1 ,  0 , 0
                0 , 0 , 0 ,  0 ,  1 , 0
                0 , 0 , 0 ,  0 ,  0 , 1
                ] * OldState...
                - ...
                [
                0.5*dt^2 , 0        , 0
                0        , 0.5*dt^2 , 0
                0        , 0        , 0.5*dt^2
                dt       , 0        , 0
                0        , dt       , 0
                0        , 0        , dt
                ] * OldControl;
        end
        
        function ret = MeasFcn(obj,State)
            % Compute measurements from current states
            
            ret = [
                atan2(State(2),State(1))
                atan2(-State(3),sqrt(State(1)^2+State(2)^2))
                ];
            
        end
        
        function ret = StateJac_x(obj,dt)
            % Derivation of StateFcn with respect to x
            
            ret =  [
                1 , 0 , 0 , dt ,  0 , 0
                0 , 1 , 0 ,  0 , dt , 0
                0 , 0 , 1 ,  0 ,  0 , dt
                0 , 0 , 0 ,  1 ,  0 , 0
                0 , 0 , 0 ,  0 ,  1 , 0
                0 , 0 , 0 ,  0 ,  0 , 1
                ];
        end
        
        function ret = StateJac_w(obj,dt)
            % Derivation of StateFcn with respect to w
            
            ret =  [
                0.5*dt^2, 0         , 0
                0       , 0.5*dt^2  , 0
                0       , 0         , 0.5*dt^2
                dt      , 0         , 0
                0       , dt        , 0
                0       , 0         , dt
                ];
        end
        
        function ret = MeasJac_x(obj,State,dt)
            % Derivation of MeasFcn with respect to x
            
            x = State(1);
            y = State(2);
            z = State(3);
            
            ret = [
                -y/(x^2 + y^2)   , (x*z)/((x^2 + y^2)^(1/2)*(x^2 + y^2 + z^2))
                x/(x^2 + y^2)    , (y*z)/((x^2 + y^2)^(1/2)*(x^2 + y^2 + z^2))
                0                , -(x^2 + y^2)^(1/2)/(x^2 + y^2 + z^2)
                0                , 0
                0                , 0
                0                , 0
                ]';
            
        end
    end
end

