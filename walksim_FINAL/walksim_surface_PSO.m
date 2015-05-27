function [InitFunction, CostFunction] = walksim_surface_PSO(filename)

InitFunction = @Init;
CostFunction = @Cost;
return

%=====================================================================================
function [Population, OPTIONS] = Init(OPTIONS)
% global matrixwalktime matrixeffort matrixconstant GainStruc

% Initialize the population
Population = struct('chrom', cell([1 OPTIONS.popsize]), 'cost', cell([1 OPTIONS.popsize]));
load('result005')

%         if GainStruc == 2
%             matrixwalktime = zeros(3);
%             matrixeffort   = zeros(3);
%             matrixconstant = zeros(3);
%         else
%             matrixwalktime = zeros(7);
%             matrixeffort   = zeros(7);
%             matrixconstant = zeros(7);
%         end

for popindex = 1 : OPTIONS.popsize
    chrom = OPTIONS.MinDomain + (OPTIONS.MaxDomain - OPTIONS.MinDomain) .* rand(1,OPTIONS.numVar);
    Population(popindex).chrom = chrom;
end
return

%=====================================================================================
function [Population, walktime] = Cost(Population, OPTIONS)
global xinit duration matrixwalktime matrixeffort matrixconstant W Cstructure pfsign pfmag perturb
     % Compute the cost of each member in Population
    for popindex = 1 : length(Population)
            k = Population(popindex).chrom;
            
            if pfsign > 0
                perturb.Force = pfmag.*rand(size(perturb.t));
            else
                perturb.Force = pfmag*perturb.t.*randn(size(perturb.t));
            end
            
            options = odeset('Events',@walk_events);
            [~,x,walktime] = ode15s(@odefun, [0 duration], xinit, options);

            if (Cstructure == 0)
            Population(popindex).cost = -walktime;
            else
            Population(popindex).cost = W*x(end,19:20)';
            end
            
%             if (Cstructure == 0)
%                 matrixwalktime = [matrixwalktime; k(1) k(2) walktime];     
%                 fprintf('Gains: %8.3f %8.3f-- walk duration %8.3f effort %10.5f\n', k, walktime, x(end,19));
%             else
%                 matrixconstant = [matrixconstant; k(1) k(2) Population(popindex).cost];   
%                 fprintf('Gains: %8.3f %8.3f-- cost %8.3f integral of position error %10.5f\n', k, Population(popindex).cost, x(end,20));
%             end
%             matrixbudget = [matrixbudget; k(1) k(2) k(3) k(4) k(5) k(6) Population(popindex).cost];   
%             fprintf('Gains: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f-- cost %8.3f integral of position error %10.5f\n', k, Population(popindex).cost, x(end,20));
    end

return

