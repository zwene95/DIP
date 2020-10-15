function [modelName] = getModelName(modelOptions)
% Extract unique modelname from modelOptions
    
    modelNameRaw = [];
%     struct = struct2cell(modelOptions);
    structs = {modelOptions};
    
    while 1
        
        struct = struct2cell(structs{1});
        
        i = cellfun(@(x) isstruct(x),struct);
        modelNameRaw = join([modelNameRaw; string(struct(~i))]);   
        structs = [structs(2:end); struct(i)];
        
        if isempty(structs)
            break;
        end
        
    end
    
    modelName = ...
        strrep(...
            strrep(...
                strrep(modelNameRaw,'true','1'),...
                'false','0'),...
                ' ','');
    
%EoF
end

