classdef OptimizationType < uint8
    % Define the type of optimization being run. This allows for different
    % settings in cases where we're optimizing a plan vs running online. 
    enumeration
        MS (1)
        MPC (2)
    end
end

