classdef reshape_varargin
    properties
        nT      = 51;
        nOut    = 6;
        nSta    = 9;
        nCtr    = 3;
        nPar    = 2;
    end
    properties (Dependent)
        data
    end
    methods
        function ret = get.data(obj)
            ret = cell(1,4);
            ret{1} = reshape(linspace(1,obj.nOut*obj.nT,obj.nOut*obj.nT),obj.nOut,obj.nT);
            ret{2} = reshape(linspace(obj.nOut*obj.nT+1,obj.nOut*obj.nT+obj.nSta*obj.nT,obj.nSta*obj.nT),obj.nSta,obj.nT);
            ret{3} = reshape(linspace((obj.nOut+obj.nSta)*obj.nT+1,(obj.nOut+obj.nSta+obj.nCtr)*obj.nT,obj.nCtr*obj.nT),obj.nCtr,obj.nT);
            ret{4} = reshape(linspace((obj.nOut+obj.nSta+obj.nCtr)*obj.nT+1,(obj.nOut+obj.nSta+obj.nCtr)*obj.nT+obj.nPar,obj.nPar),obj.nPar,1);
        end
        
        function ret = unwrapData(obj,data)
            ret = nan(1,(obj.nOut + obj.nSta + obj.nCtr) * obj.nT + obj.nPar);
            nInputs = numel(data);
            idx_start = 1;
            for i=1:nInputs
                tmp = reshape(data{i},1,[]);
                idx_end = idx_start + numel(tmp) - 1;
                ret(idx_start:idx_end) = tmp;
                idx_start = idx_end + 1;
            end
        end
        
        function ret = wrapData(obj,data)            
            idx_out_0   = 1;
            idx_out_f   = obj.nOut*obj.nT;
            idx_sta_0   = idx_out_f + 1;
            idx_sta_f   = idx_sta_0 + obj.nSta*obj.nT - 1;
            idx_ctr_0   = idx_sta_f + 1;
            idx_ctr_f    = idx_ctr_0 + obj.nCtr*obj.nT - 1;
            
            outputs     = cell(1,1);
            states      = cell(1,1);
            controls    = cell(1,1);
            parameter   = cell(1,1);
            
            outputs{:}   = reshape(data(idx_out_0:idx_out_f),obj.nOut,[]);
            states{:}    = reshape(data(idx_sta_0:idx_sta_f),obj.nSta,[]);
            controls{:}  = reshape(data(idx_ctr_0:idx_ctr_f),obj.nCtr,[]);
            parameter{:} = data(end-1:end)';
            
            ret = [outputs states controls parameter];
        end
    end    
end


% test = cell(1,4);
% test{1} = reshape(linspace(1,6*51,6*51),6,51);
% test{2} = reshape(linspace(6*51+1,6*51+9*51,9*51),9,51);
% test{3} = reshape(linspace((6+9)*51+1,(6+9+3)*51,3*51),3,51);
% test{4} = [919;920];
