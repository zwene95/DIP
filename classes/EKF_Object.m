classdef EKF_Object
    %EKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Time;
        States;
        Controls;        
        StepTime;
        Q;
        R;
        P0;
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
            dt   = diff(obj.Time(1:2));
            
            % Compute measurements
            r.Measurements = obj.MeasFcn(obj.States);
            % Interpolation 
%             if (dt > obj.StepTime)
                nEKF = ceil(obj.Time(end)/obj.StepTime);                
                iEKF = linspace(1,nDat,nEKF);
                % Interpolate input data
                r.Time   = interp1(iDat,obj.Time',iEKF)';
                r.x_true = interp1(iDat,obj.States',iEKF)';            
                r.u_true = interp1(iDat,obj.Controls',iEKF)';
                r.z_true = interp1(iDat,r.Measurements',iEKF)';
                dt       = obj.StepTime;
%             else                
%                 r.x_true = obj.States;
%                 r.u_true = obj.Controls;
%                 r.z_true = r.Measurements;
%                 nEKF     = nDat;                
%             end
            
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
            
            % Initial state
            x0  = r.x_true(:,1) + x0_bias;
            % Setup filter variables
            x_k_km1 =   nan(n_x,nEKF);
            r.x_k_k   =   nan(n_x,nEKF);
            P_k_km1 =   nan(n_x,n_x,nEKF);
            P_k_k   =   nan(n_x,n_x,nEKF);
            % Initialize filter variables
            x_k_km1(:,1)    = x0;
            r.x_k_k(:,1)      = x0;
            P_k_km1(:,:,1)  = obj.P0;
            P_k_k(:,:,1)    = obj.P0;           
            
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
            r.Sigma(:,1)     =   sqrt(diag(obj.P0));
            r.Error(:,1)     =   -x0_bias;
            r.P_trace(1)     =   trace(obj.P0);
            r.P_trace_pos(1) =   trace(obj.P0(1:3,1:3));
            r.P_trace_vel(1) =   trace(obj.P0(4:6,4:6));
            
            %% Run EKF
%             tic
            for k = 2:nEKF
            
                % Prediction step                
                % State Propagation
                x_k_km1(:,k) = obj.StateFcn(r.x_k_k(:,k-1),u(:,k-1),dt);
                % Covariance Propagation
                P_k_km1(:,:,k) =...
                    F_x * P_k_k(:,:,k-1) * F_x' + F_w * obj.Q * F_w';
                % Predict measurements
                y_k_km1 = obj.MeasFcn(x_k_km1(:,k));
                
                % Kalman Gain
                H = obj.MeasJac_x(x_k_km1(:,k));
                K = P_k_km1(:,:,k) * H' / (H * P_k_km1(:,:,k) * H' + obj.R);
                
                % Correction step
                r.x_k_k(:,k)   =   x_k_km1(:,k) + K * (r.z(:,k) - y_k_km1);
                P_k_k(:,:,k) =   (eye(n_x) - K * H) * P_k_km1(:,:,k);
                
                % Post processing variables
                r.Measurements(:,k) = y_k_km1;
                r.Sigma(:,k)        = sqrt(diag(P_k_k(:,:,k)));
                r.Error(:,k)        = r.x_true(:,k) - r.x_k_k(:,k);
                r.NEES(k)           = r.Error(:,k)' * P_k_k(:,:,k) * r.Error(:,k);
                r.NEES_pos(k)       = r.Error(1:3,k)' * P_k_k(1:3,1:3,k) * r.Error(1:3,k);
                r.NEES_vel(k)       = r.Error(4:6,k)' * P_k_k(4:6,4:6,k) * r.Error(4:6,k);
                r.P_trace(k)        = trace(P_k_k(:,:,k));
                r.P_trace_pos(k)    = trace(P_k_k(1:3,1:3,k));
                r.P_trace_vel(k)    = trace(P_k_k(4:6,4:6,k));
            end
%             toc            
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
            
            x = State(1,:);
            y = State(2,:);
            z = State(3,:);            
            ret = [
                atan2(y,x)
                atan2(-z,sqrt(x.^2+y.^2))
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

