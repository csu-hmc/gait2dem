% Run the simulation and compute minus duration
function [cost] = walk(k)                     
global xinit duration Cstructure matrixeffort matrixwalktime matrixcost pfsign pfmag perturb W

    if pfsign > 0
        perturb.Force = pfmag.*rand(size(perturb.t));
    else
        perturb.Force = pfmag*perturb.t.*randn(size(perturb.t));
    end

    if (Cstructure == 0)
        options = odeset('Events',@walk_events);
        [~,x,walktime] = ode15s(@odefun, [0 duration], xinit, options);
        cost = walktime;
        matrixwalktime = [matrixwalktime; k(1) k(2) cost];
        fprintf('Gains: %8.3f %8.3f walktime %8.3f \n', k, walktime);
    else
        [~,x,~] = ode15s(@odefun, [0 duration], xinit);
        cost = W*x(end,19:20)';
        matrixcost = [matrixcost; k(1) k(2) cost];
        matrixeffort = [matrixeffort; k(1) k(2) x(end,19)];
        fprintf('Gains: %8.3f %8.3f conventional cost %8.3f effort %8.3f \n', k, W*x(end,19:20)', x(end,19));       
    end

end

