classdef EKF_Object < handle
    %EKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TimeHistory;
        StateHistory;
        ControlHistory;
        ObserverConfig;
    end
    
    properties (Dependent,Access=protected)
        StepTime;
        Std_r0;
        Std_r0_dot;
        Std_x0;
        Std_Qw;
        Std_Rv;
        Std_w;
        Std_v;
    end
    
    methods
        
        function ret = get.StepTime(obj)
            ret = obj.ObserverConfig.StepTime;
        end
        
        function ret = get.Std_r0(obj)
            ret = obj.ObserverConfig.Std_r0;
        end
        
        function ret = get.Std_r0_dot(obj)
            ret = obj.ObserverConfig.Std_r0_dot;
        end
        
        function ret = get.Std_Qw(obj)
            ret = obj.ObserverConfig.Std_Qw;
        end
        
        function ret = get.Std_Rv(obj)
            ret = obj.ObserverConfig.Std_Rv;
        end
        
        function ret = get.Std_w(obj)
            ret = obj.ObserverConfig.Std_w;
        end
        
        function ret = get.Std_v(obj)
            ret = obj.ObserverConfig.Std_v;
        end
        
        function [x0_est,P0_est] = initialEstimate(obj, x0)
            % Compute initial state estimate and covariance by performing
            % the unscented transform to spherical initial guess
            % Inputs:
            %   x0: True state vector (Cartesian)
            % Outputs:
            %   x0_est: Estimate state vector (Cartesian)
            %   P0_est: Estimate state covariance (Cartesian)
            
            f_SC = obj.f_SC;                                                % Nonlinear transformation from Cartesian in spherical system
            x0_S_true = f_SC(x0);                                           % Initial true state vector in spherical states
            rng(9999);
            x0_S_est = blkdiag(eye(3),zeros(3))* x0_S_true + ...            % Initial estimate of spherical states is true spherical states +
                [normrnd(0, obj.Std_r0)                                     % + range uncertainty
                normrnd(0, obj.Std_Rv)                                      % + azimuth ucertainty
                normrnd(0, obj.Std_Rv)                                      % + elevation ucertainty
                zeros(3,1)];                                                % Initial velocity is unknown by passive sensors and assumed to be zero
            P0_S = blkdiag(...                                              % Initial covaiance in spherical coordinates
                diag([obj.Std_r0^2;ones(2,1)*obj.Std_Rv^2]),...
                diag([obj.Std_r0_dot^2;ones(2,1)*sqrt(obj.Std_Rv^2)]));
            % Unscented trasform: spherical to Cartesian coordinates
            [x0_est,P0_est] = obj.UT(x0_S_est,P0_S,obj.f_CS);
        end
        
        function ret = runEKF(obj)
            % State estimation with EKF
            %% InitEKF
            % Initialization and preprocessing state estimation with EKF
            ret.nDat = length(obj.TimeHistory);
            ret.iDat = linspace(1,ret.nDat,ret.nDat);
            dtDat   = diff(obj.TimeHistory(1:2));
            
            % Compute measurements
            MeasurementHistory = obj.MeasFcn(obj.StateHistory);
            % Interpolation
            if (dtDat > obj.StepTime)
                ret.nEKF = ceil(obj.TimeHistory(end)/obj.StepTime);
                ret.iEKF = linspace(1,ret.nDat,ret.nEKF);
                % Interpolate input data
                ret.Time = ...
                    interp1(ret.iDat,obj.TimeHistory',ret.iEKF)';
                ret.x_true = ...
                    interp1(ret.iDat,obj.StateHistory',ret.iEKF)';
                ret.u_true = ...
                    interp1(ret.iDat,obj.ControlHistory',ret.iEKF)';
                ret.z_true = ...
                    interp1(ret.iDat,MeasurementHistory',ret.iEKF)';
                dt = obj.StepTime;
            else
                ret.Time    = obj.TimeHistory;
                ret.x_true  = obj.StateHistory;
                ret.u_true  = obj.ControlHistory;
                ret.z_true  = MeasurementHistory;
                ret.nEKF    = ret.nDat;
                ret.iEKF    = ret.iDat;
                dt          = dtDat;
            end
            
            % Zero-mean Gaussian distibuted process and measurement noise
            rng(2019);
            w = normrnd(0,obj.Std_w,size(ret.u_true));
            rng(2020);
            v = normrnd(0,obj.Std_v,size(ret.z_true));
            % Add noise to true controls and measurements
            ret.u = ret.u_true + w;
            ret.z = ret.z_true + v;
            % Build process and measurement covariances
            Qw = eye(numel(w(:,1))) * (obj.Std_Qw)^2;
            Rv = eye(numel(v(:,1))) * (obj.Std_Rv)^2;
            
            % Get number of states and outputs
            n_x = numel(ret.x_true(:,1));
            n_y = numel(ret.z_true(:,1));
            
            % Get constant state and disturbance jacobians
            F_x = obj.StateJac_x(dt);
            F_w = obj.StateJac_w(dt);
            
            % Initial estimate and covariance
            [x0,P0] = initialEstimate(obj, ret.x_true(:,1));
            
            % Setup filter variables
            x_k_km1     =   nan(n_x,ret.nEKF);
            ret.x_k_k   =   nan(n_x,ret.nEKF);
            P_k_km1     =   nan(n_x,n_x,ret.nEKF);
            P_k_k       =   nan(n_x,n_x,ret.nEKF);
            % Initialize filter variables
            x_k_km1(:,1)    = x0;
            ret.x_k_k(:,1)  = x0;
            P_k_km1(:,:,1)  = P0;
            P_k_k(:,:,1)    = P0;
            
            % Setup postprocessing variables and functions
            f_SC = obj.f_SC;                                                % Nonlinear transformation from Cartesian in spherical system
            ret.Measurements    = nan(n_y, ret.nEKF);                       % Estimated measurements
            ret.Std             = nan(n_x, ret.nEKF);                       % Standard deviation
            ret.P_trace_pos     = nan(1,ret.nEKF);                          % Covariance trace position
            ret.P_trace_vel     = nan(1,ret.nEKF);                          % Covariance trace velocity
            ret.x_S_true        = nan(n_x, ret.nEKF);                       % True state vector in spehrical coordinates
            ret.x_S_est         = nan(n_x, ret.nEKF);                       % Estimated state vector in spehrical coordinates
            ret.P_S             = nan(n_x,n_x,ret.nEKF);                    % Covariance in spherical coordinates
            ret.Std_S           = nan(n_x, ret.nEKF);                       % Standard deviation in spherical coordinates
            % Initialize postprocessing variables
            ret.Measurements    = obj.MeasFcn(x0);
            ret.Std(:,1)        = sqrt(diag(P0));
            ret.P_trace_pos(1)  = trace(P0(1:3,1:3));
            ret.P_trace_vel(1)  = trace(P0(4:6,4:6));
            ret.x_S_true(:,1)   = f_SC(ret.x_true(:,1));                         
            [ret.x_S_est(:,1),...
                ret.P_S(:,:,1)] = obj.UT(x0,P0,obj.f_SC);
            ret.Std_S(:,1)      = sqrt(diag(ret.P_S(:,:,1)));
                        
            %% Run EKF
            %             tic
            for k = 2:ret.nEKF
                
                % Prediction step
                % State Propagation
                x_k_km1(:,k) = obj.StateFcn(ret.x_k_k(:,k-1),ret.u(:,k-1),dt);
                % Covariance Propagation
                P_k_km1(:,:,k) =...
                    F_x * P_k_k(:,:,k-1) * F_x' + F_w * Qw * F_w';
                % Predict measurements
                y_k_km1 = obj.MeasFcn(x_k_km1(:,k));
                
                % Kalman Gain
                H = obj.MeasJac_x(x_k_km1(:,k));
                K = P_k_km1(:,:,k) * H' / (H * P_k_km1(:,:,k) * H' + Rv);
                
                % Correction step
                ret.x_k_k(:,k)   =   x_k_km1(:,k) + K * (ret.z(:,k) - y_k_km1);
                P_k_k(:,:,k) =   (eye(n_x) - K * H) * P_k_km1(:,:,k);
                
                % Post processing variables
                ret.Measurements(:,k) = y_k_km1;
                ret.Std(:,k)          = sqrt(diag(P_k_k(:,:,k)));
                ret.P_trace_pos(k)    = trace(P_k_k(1:3,1:3,k));
                ret.P_trace_vel(k)    = trace(P_k_k(4:6,4:6,k));
                ret.x_S_true(:,k)     = f_SC(ret.x_true(:,k));
                [ret.x_S_est(:,k),ret.P_S(:,:,k)] = ...
                    obj.UT(ret.x_k_k(:,k),P_k_k(:,:,k),obj.f_SC);
                ret.Std_S(:,k) = sqrt(diag(ret.P_S(:,:,k)));
                
            end
            
            % Compute estimator performance
            ret.Error     = ret.x_true - ret.x_k_k;                             % Filter estimation error
            ret.SE_pos    = sum(ret.Error(1:3,:).^2);                           % Squared position error
            ret.SE_vel    = sum(ret.Error(4:6,:).^2);                           % Squared velocity error
            ret.RMSE_pos  = sqrt(mean(ret.SE_pos));                             % Root mean squared positional error
            ret.RMSE_vel  = sqrt(mean(ret.SE_vel));                             % Root mean squared velocity error
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
        
        function [x_UT, P_UT] = UT(obj,x,P,g)
            % Unscented transform of states x and covariance P, defined by
            % nonlinear transformation g
            
            % Initialize UT
            n_x = numel(x);                                                 % numer of states
            alpha = 1e-3;                                                   % default, tunable
            ki = 0;                                                         % default, tunable
            beta = 2;                                                       % default, tunable
            lambda = alpha^2 * (n_x + ki) - n_x;                            % scaling factor
            c = n_x + lambda;                                               % scaling factor
            Wm = [lambda/c 0.5/c + zeros(1,2*n_x)];                       	% weights for means
            Wc = Wm;                                                        % weights for covariance
            Wc(1) = Wc(1) + (1 - alpha^2 + beta);
            
            % Compute Sigma-Points X
            A = sqrt(c)*chol(P)';
            X_UT = x(:,ones(1,numel(x)));
            X = [x X_UT + A X_UT - A];
            
            % Perform UT
            L = size(X,2);
            x_UT = zeros(n_x,1);
            X_UT = zeros(n_x,L);
            for k = 1:L
                X_UT(:,k) = g(X(:,k));
                x_UT = x_UT + Wm(k) * X_UT(:,k);
            end
            X_UT1 = X_UT - x_UT(:,ones(1,L));
            P_UT = X_UT1 * diag(Wc) * X_UT1';
        end
        
        function ret = f_CS(obj)
            % Nonlinear tansformation from spherical to Cartesian states
            %   Spherical state vector: x_S = [r b e r_dot b_dot e_dot]'
            %   Cartesian state vector: x_C = [x y z x_dot y_dot z_dot]'
            ret = @(x_S)[...
                x_S(1) * cos(x_S(3)) * cos(x_S(2))
                x_S(1) * cos(x_S(3)) * sin(x_S(2))
                -x_S(1) * sin(x_S(3))
                x_S(4) * cos(x_S(3)) * cos(x_S(2)) - x_S(1) * sin(x_S(3)) * cos(x_S(2)) * x_S(6) - x_S(1) * cos(x_S(3)) * sin(x_S(2)) * x_S(5)
                x_S(4) * cos(x_S(3)) * sin(x_S(2)) - x_S(1) * sin(x_S(3)) * sin(x_S(2)) * x_S(6) + x_S(1) * cos(x_S(3)) * cos(x_S(2)) * x_S(5)
                -x_S(4) * sin(x_S(3)) - x_S(1) * cos(x_S(3)) * x_S(6)];
        end
        
        function ret = f_SC(obj)
            % Nonlinear tansformation from Cartesian to spherical states
            %   Cartesian state vector: x_C = [x y z x_dot y_dot z_dot]'
            %   Spherical state vector: x_S = [r b e r_dot b_dot e_dot]'
            ret = @(x_C)[...
                sqrt(x_C(1)^2 + x_C(2)^2 + x_C(3)^2)
                atan2(x_C(2),x_C(1))
                atan2(-x_C(3), sqrt(x_C(1)^2 + x_C(2)^2))
                (x_C(1)*x_C(4) + x_C(2)*x_C(5) + x_C(3)*x_C(6)) / sqrt(x_C(1)^2 + x_C(2)^2 + x_C(3)^2)
                (x_C(1)*x_C(5) - x_C(2)*x_C(4)) / (x_C(1)^2 + x_C(2)^2)
                (x_C(1)*x_C(3)*x_C(4) + x_C(2)*x_C(3)*x_C(5) - (x_C(1)^2 + x_C(2)^2)*x_C(6)) / (sqrt(x_C(1)^2 + x_C(2)^2) * (x_C(1)^2 + x_C(2)^2 + x_C(3)^2))];
        end
        
        
    end
end

