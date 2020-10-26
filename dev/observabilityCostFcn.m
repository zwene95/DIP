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
% 
        str.input(cnt,1).m = obj.NPhases+1;

        str.input(cnt,1).n = 1;

        str.input(cnt,1).name = 'parameters';

        str.input(cnt,1).argnames = obj.TimeNames;

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



    x = cell(obj.NPhases,1);

    u = cell(obj.NPhases,1);

    t = cell(obj.NPhases,1);

    out = cell(obj.NPhases,1);



    if size(varargin{end},1) ~= obj.NPhases+1

        error('Dimensions of parameters do not match');

    end

    if size(varargin{end},2) ~= 1

        error('Dimensions of parameters do not match');

    end



    cntPhase = 1;

    nTimeStepsPerPhase = obj.NTimeStepsPerPhase;

    for iInput=1:3:numel(varargin)-1

        % outputs

        if size(varargin{iInput},1) ~= obj.NOutputs

            error('Dimensions do not match');

        end

        if size(varargin{iInput},2) ~= obj.NTimeStepsPerPhase

            error('Dimensions do not match');

        end

        out{cntPhase} = varargin{iInput};



        % states

        if size(varargin{iInput+1},1) ~= obj.NStates

            error('Dimensions do not match');

        end

        if size(varargin{iInput+1},2) ~= obj.NTimeStepsPerPhase

            error('Dimensions do not match');

        end

        x{cntPhase} = varargin{iInput+1};



        % controls

        if size(varargin{iInput+2},1) ~= obj.NControls

            error('Dimensions do not match');

        end

        if size(varargin{iInput+2},2) ~= obj.NTimeStepsPerPhase

            error('Dimensions do not match');

        end

        u{cntPhase} = varargin{iInput+2};



        % time
        t{cntPhase} = linspace(varargin{end}(cntPhase),varargin{end}(cntPhase+1),nTimeStepsPerPhase(cntPhase));

        % increment phase counter
        cntPhase = cntPhase + 1;

    end     
    
    
    %% EKF
    
%     N       = nTimeStepsPerPhase;
%     dt      = gradient(t(1:2)); 

    % States
    x = find(strcmp(obj.StateNames,'x'));
    y = find(strcmp(obj.StateNames,'y'));
    z = find(strcmp(obj.StateNames,'z'));
    x_inv = find(strcmp(obj.StateNames,'x_inv'));
    y_inv = find(strcmp(obj.StateNames,'y_inv'));
    z_inv = find(strcmp(obj.StateNames,'z_inv'));
    u = find(strcmp(obj.StateNames,'u'));
    v = find(strcmp(obj.StateNames,'v'));
    w = find(strcmp(obj.StateNames,'w'));
    x_true = 
%     x_true  = x;

    % Controls
    idx_u1 = find(strcmp(obj.OutputNames,'u1'));
    idx_u2 = find(strcmp(obj.OutputNames,'u2'));
    idx_u3 = find(strcmp(obj.OutputNames,'u3'));
    
    u_true  =  out([idx_u1 idx_u2 idx_u3], :);
    
%     z_true  =  

    
    
    
    
    
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