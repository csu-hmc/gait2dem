function [value,isterminal,direction] = walk_events(~,x)
global budget
    % this function defines an event that terminates the simulation
    height = 0.75;         		% uses hip height to indicate the model has fallen
    value(1) = (x(2) - height);	% simulation will stop when this becomes zero
    isterminal(1) = 1;
    direction(1) = -1;
    
%   effort budget
    value(2) = x(19) - budget;
    isterminal(2) = 1;
    direction(2) = 1;
end

