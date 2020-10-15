function [results] = logResults(setup, problem)
% Write output results file
% [results] = logResults(setup, problem)

    results = struct();
    
    results.time = problem.RealTime;
    
    %% Defender 
    
    % Translational States
    
    % Position
    results.defender.states.pos = [ 
        problem.StateValues(find(ismember(problem.StateNames,'x'),1),:)
        problem.StateValues(find(ismember(problem.StateNames,'y'),1),:)
        problem.StateValues(find(ismember(problem.StateNames,'z'),1),:)
    ];
    % Velocity
    results.defender.states.vel = [ 
        problem.StateValues(find(ismember(problem.StateNames,'u'),1),:)
        problem.StateValues(find(ismember(problem.StateNames,'v'),1),:)
        problem.StateValues(find(ismember(problem.StateNames,'w'),1),:)
    ];
    % Accleration
    results.defender.states.acc = [
        problem.StateDotValues(find(ismember(problem.StateDotNames,'u_dot'),1),:)
        problem.StateDotValues(find(ismember(problem.StateDotNames,'v_dot'),1),:)
        problem.StateDotValues(find(ismember(problem.StateDotNames,'w_dot'),1),:)
    ];

    % Rotational States    
    if setup.modelOptions.defender.SixDoF
        
        % Attitude
        results.defender.states.att = [ 
            problem.StateValues(find(ismember(problem.StateNames,'phi'),1),:)   
            problem.StateValues(find(ismember(problem.StateNames,'theta'),1),:) 
            problem.StateValues(find(ismember(problem.StateNames,'psi'),1),:)   
        ];
        % Rotation
        results.defender.states.rot = [ 
            problem.StateValues(find(ismember(problem.StateNames,'p'),1),:) 
            problem.StateValues(find(ismember(problem.StateNames,'q'),1),:) 
            problem.StateValues(find(ismember(problem.StateNames,'r'),1),:) 
        ];
        % Motor States
        if setup.modelOptions.defender.MotorLag
            results.defender.states.motor.w1 = ...
                problem.StateValues(find(ismember(problem.StateNames,'w1'),1),:);
            results.defender.states.motor.w2 = ...
                problem.StateValues(find(ismember(problem.StateNames,'w2'),1),:);
            results.defender.states.motor.w3 = ...
                problem.StateValues(find(ismember(problem.StateNames,'w3'),1),:);
            results.defender.states.motor.w4 = ...
                problem.StateValues(find(ismember(problem.StateNames,'w4'),1),:);
            
        end
        
    end
    
    % Controls    
    if setup.modelOptions.defender.SixDoF
        
        if setup.modelOptions.defender.MotorLag
            
            results.defender.controls.w1 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w1_cmd'),1),:);
            results.defender.controls.w2 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w2_cmd'),1),:);
            results.defender.controls.w3 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w3_cmd'),1),:);
            results.defender.controls.w4 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w4_cmd'),1),:);
        else
            results.defender.controls.w1 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w1'),1),:);
            results.defender.controls.w2 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w2'),1),:);
            results.defender.controls.w3 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w3'),1),:);
            results.defender.controls.w4 = ...
                problem.ControlValues(find(ismember(problem.ControlNames,'w4'),1),:);
            
        end
        
    else
        
        results.defender.controls.Tx = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'T_x'),1),:);
        results.defender.controls.Ty = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'T_y'),1),:);
        results.defender.controls.Tz = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'T_z'),1),:);       
        
    end
    
    % Aero
    if setup.modelOptions.defender.Aero
        results.defender.aero = [   
            problem.OutputValues(find(ismember(problem.OutputNames,'FDAD_x'),1),:)   
            problem.OutputValues(find(ismember(problem.OutputNames,'FDAD_y'),1),:)
            problem.OutputValues(find(ismember(problem.OutputNames,'FDAD_z'),1),:)
        ];
    end
    
    %% Invader
    
    % States
    
    if strcmp(setup.modelOptions.invader.Type, 'Quad1')
        
        % Position
        results.invader.states.pos = [  
            problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
        ];  
        % Velocity
        results.invader.states.vel = [  
            problem.StateDotValues(find(ismember(problem.StateDotNames,'x_inv_dot'),1),:)
            problem.StateDotValues(find(ismember(problem.StateDotNames,'y_inv_dot'),1),:)
            problem.StateDotValues(find(ismember(problem.StateDotNames,'z_inv_dot'),1),:)
        ];  
        
    elseif strcmp(setup.modelOptions.invader.Type, 'Quad2')
        
        % Position
        results.invader.states.pos = [  
            problem.StateValues(find(ismember(problem.StateNames,'x_inv'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'y_inv'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'z_inv'),1),:)
        ];
        % Velocity
        results.invader.states.vel = [  
            problem.StateValues(find(ismember(problem.StateNames,'u_inv'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'v_inv'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'w_inv'),1),:)
        ];
    
    end    
    
    % Controls
    
    if strcmp(setup.modelOptions.invader.Type, 'Quad1')       
        
        results.invader.controls.vI_x = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'vI_x'),1),:);
        results.invader.controls.vI_y = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'vI_y'),1),:);
        results.invader.controls.vI_z = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'vI_z'),1),:);
        
    elseif strcmp(setup.modelOptions.invader.Type, 'Quad2')
        
        results.invader.controls.T_x_inv = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'T_x_inv'),1),:);
        results.invader.controls.T_y_inv = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'T_y_inv'),1),:);
        results.invader.controls.T_z_inv = ...
            problem.ControlValues(find(ismember(problem.ControlNames,'T_z_inv'),1),:);
        
    end
    
    
    %% LOS
    pDIO = results.invader.states.pos - results.defender.states.pos;
    results.LOS.elevation = atan2(-pDIO(3,:),sqrt(pDIO(1,:).^2 + pDIO(2,:).^2));
    results.LOS.azimuth = atan2(pDIO(2,:),pDIO(1,:));
    
    %% Seeker
    results.seeker.elevation = results.LOS.elevation - results.defender.states.att(2,:);
    results.seeker.azimuth = results.LOS.azimuth - results.defender.states.att(3,:);    
    
    
    %% Target Data

      
    %% Observer
    if setup.modelOptions.observer
        
        % Estimated relative position
        results.observer.states.pos = [ 
            problem.StateValues(find(ismember(problem.StateNames,'x_est'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'y_est'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'z_est'),1),:)
        ];
    
        % Estimated relative velocity
        results.observer.states.vel = [ 
            problem.StateValues(find(ismember(problem.StateNames,'u_est'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'v_est'),1),:)
            problem.StateValues(find(ismember(problem.StateNames,'w_est'),1),:)
        ];
    
        % Observed invader position
        results.observer.pIOO_obs = ...
            results.observer.states.pos + results.defender.states.pos;
        
        % Observed invader velocity
        results.observer.vIOO_obs.vel= ...
            results.observer.states.vel + results.defender.states.vel;
        
        % Covariane trace position only
        results.observer.covarianceTrace.pos = ...
            problem.StateValues(find(ismember(problem.StateNames,'P_11'),1),:) + ...
            problem.StateValues(find(ismember(problem.StateNames,'P_22'),1),:) + ...
            problem.StateValues(find(ismember(problem.StateNames,'P_33'),1),:);
        
        % Covariane trace velocity only
        results.observer.covarianceTrace.vel = ...
            problem.StateValues(find(ismember(problem.StateNames,'P_44'),1),:) + ...
            problem.StateValues(find(ismember(problem.StateNames,'P_55'),1),:) + ...
            problem.StateValues(find(ismember(problem.StateNames,'P_66'),1),:);
        
        % Covariane trace combined (pos+vel)
        results.observer.covarianceTrace.tot = ...
            results.observer.covarianceTrace.pos + ...
            results.observer.covarianceTrace.vel; 
        
        % True measurement
        results.observer.meas.true = [ 
            problem.OutputValues(find(ismember(problem.OutputNames,'azimuth_true'),1),:)
            problem.OutputValues(find(ismember(problem.OutputNames,'elevation_true'),1),:)            
        ];        
        % Estimated measurement
        results.observer.meas.est = [ 
            problem.OutputValues(find(ismember(problem.OutputNames,'azimuth_est'),1),:)
            problem.OutputValues(find(ismember(problem.OutputNames,'elevation_est'),1),:)            
        ];
        
    end                          


end
