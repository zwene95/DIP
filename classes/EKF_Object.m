classdef EKF_Object < handle
    %EKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        TimeHistory;
        StateHistory;
        ControlHistory;
        ObserverConfig;
    end
    
    properties (Dependent)
        StepTime;
        Std_r0;
        Std_r0_dot;
        Std_x0;
        Std_Qw;
        Std_Rv;
        Std_w;
        Std_v;
        Results;
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
        
        function ret = get.Std_v(obj)
            ret = obj.ObserverConfig.Std_v;
        end
        
        function ret = get.Std_w(obj)
            ret = obj.ObserverConfig.Std_w;
        end
        
        function ret = get.Std_Rv(obj)
            ret = obj.ObserverConfig.Std_Rv;
        end
        
        function ret = runEKF(obj)
            % State estimation with EKF
            
            %% Preprocessing EKF
            nDat = length(obj.TimeHistory);
            iDat = linspace(1,nDat,nDat);
            dt   = diff(obj.TimeHistory(1:2));
            
            % Compute measurements
            ret.Measurements = obj.MeasFcn(obj.StateHistory);
            % Interpolation
            if (dt > obj.StepTime)
                nEKF = ceil(obj.TimeHistory(end)/obj.StepTime);
                iEKF = linspace(1,nDat,nEKF);
                % Interpolate input data
                ret.TimeHistory = interp1(iDat,obj.TimeHistory',iEKF)';
                ret.x_true      = interp1(iDat,obj.StateHistory',iEKF)';
                ret.u_true      = interp1(iDat,obj.ControlHistory',iEKF)';
                ret.z_true      = interp1(iDat,ret.Measurements',iEKF)';
                dt              = obj.StepTime;
            else
                ret.Time    = obj.TimeHistory;
                ret.x_true  = obj.StateHistory;
                ret.u_true  = obj.ControlHistory;
                ret.z_true  = ret.Measurements;
                nEKF        = nDat;
            end            
            
            % Zero-mean Gaussian distibuted process and measurement noise            
            rng(2019);
            w = normrnd(0,obj.Std_w,size(ret.u_true));
            rng(2020);
            v = normrnd(0,obj.Std_v,size(ret.z_true));
            % Add noise to true controls and measurements
            u = ret.u_true + w;
            ret.z = ret.z_true + v;
            % Build process and measurement covariances
            Qw = eye(numel(w(:,1))) * (obj.Std_Qw)^2;
            Rv = eye(numel(v(:,1))) * (obj.Std_Rv)^2;
            
            % Initial state and covariance in spherical coordinates
            x0_S = obj.f_SC(ret.x_true(:,1));
            
%             x0_S = [...                                                     % Initial estimate of spherical relative inader position van velocity
%                 abs(normrnd(0, obj.Std_r0))
%                 ret.z_true(1,1) + normrnd(0, obj.Std_Rv)
%                 ret.z_true(2,1) + normrnd(0, obj.Std_Rv)
%                 normrnd(0, obj.Std_r0_dot)
%                 normrnd(0, sqrt(obj.Std_Rv))
%                 normrnd(0, sqrt(obj.Std_Rv))];
            P0_S = blkdiag(...                                              % Initial covaiance in spherical coordinates
                diag([obj.Std_r0^2;ones(2,1)*obj.Std_Rv^2]),...
                diag([obj.Std_r0_dot^2;ones(2,1)*sqrt(obj.Std_Rv^2)]));
            % Unscented trasform spherical to Cartesian coordinates
            [x0_C,P0_C] = obj.UT(x0_S,P0_S,obj.f_CS);
            
            % Get number of states and outputs
            n_x = numel(ret.x_true(:,1));
            n_y = numel(ret.z_true(:,1));
            
            % Get constant state and disturbance jacobians
            F_x = obj.StateJac_x(dt);
            F_w = obj.StateJac_w(dt);
            
            %% Init EKF            
            % Initial state estimate in Cartesian coordinates
            x0_pos  = [ret.x_true(1:3,1) + BiasPos_C(1:3); ret.x_true(4:6,1)];         % Initial relative velocity is conservatively estimated to be zero
            x0_vel = zeros(3,1);
            x0 = [x0_pos; x0_vel];
            % Setup filter variables
            x_k_km1     =   nan(n_x,nEKF);
            ret.x_k_k   =   nan(n_x,nEKF);
            P_k_km1     =   nan(n_x,n_x,nEKF);
            P_k_k       =   nan(n_x,n_x,nEKF);
            % Initialize filter variables
            x_k_km1(:,1)    = x0;
            ret.x_k_k(:,1)  = x0;
            P_k_km1(:,:,1)  = P0_C;
            P_k_k(:,:,1)    = P0_C;
            
            % Setup postprocessing variables
            ret.Measurements  = nan(n_y, nEKF);
            ret.Sigma         = nan(n_x, nEKF);
            ret.P_trace_pos   = nan(1,nEKF);                                  % Covariance trace position
            ret.P_trace_vel   = nan(1,nEKF);                                  % Covariance trace velocity
            % Initialize variables for post processing
            ret.Sigma(:,1)     = sqrt(diag(P0_C));
            ret.P_trace_pos(1) = trace(P0_C(1:3,1:3));
            ret.P_trace_vel(1) = trace(P0_C(4:6,4:6));
            
            %% Run EKF
            %             tic
            for k = 2:nEKF
                
                % Prediction step
                % State Propagation
                x_k_km1(:,k) = obj.StateFcn(ret.x_k_k(:,k-1),u(:,k-1),dt);
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
                ret.Sigma(:,k)        = sqrt(diag(P_k_k(:,:,k)));
                ret.P_trace_pos(k)    = trace(P_k_k(1:3,1:3,k));
                ret.P_trace_vel(k)    = trace(P_k_k(4:6,4:6,k));
            end
            
            % Compute estimator performance
            ret.Error     = ret.x_true - ret.x_k_k;                               % Filter estimation error
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

