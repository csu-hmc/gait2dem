function [InitFunction, CostFunction] = walksim_surface_PSO(filename)
InitFunction = @Init;
CostFunction = @Cost;
return

%=====================================================================================
function [Population, OPTIONS] = Init(OPTIONS)
global control Result matrixwalktime matrixeffort matrixconstant

% Initialize the population
OPTIONS.numVar = 2; % number of dimensions in cost function
OPTIONS.MinDomain = [0 0];
OPTIONS.MaxDomain = [10000 200];
Population = struct('chrom', cell([1 OPTIONS.popsize]), 'cost', cell([1 OPTIONS.popsize]));
            if ~exist('filename', 'var') || isempty(filename)
                filename = 'result005';
            end
            load(filename)
            
            control.durcycle=2*Result.dur; 
            control.x0 = [Result.x';Result.x(Result.problem.vmx,:)'];
            control.u0 = [Result.u';Result.u(Result.problem.vmu,:)'];% 2 half gait cyclq                                                                                                                                                                                                                                                                                 
            control.u0 = control.u0(:,4:end);                                                            % throw away the first 3 columns
            control.u0 = [control.u0 ; control.u0(1,:)];                                                 % add data for the last missing data point
            control.x0 = [control.x0 ; control.x0(1,:)];                                                 % add data for the last missing data point
            control.tsamples = (0:2*Result.problem.N)*control.durcycle/(2*Result.problem.N);
          
            matrixwalktime = zeros(3);
            matrixeffort = zeros(3);
            matrixconstant = zeros(3);

for popindex = 1 : OPTIONS.popsize
    chrom = OPTIONS.MinDomain + (OPTIONS.MaxDomain - OPTIONS.MinDomain) .* rand(1,OPTIONS.numVar);
    Population(popindex).chrom = chrom;
end
return

%=====================================================================================
function [Population, walktime] = Cost(Population, OPTIONS)
global control kp kd xinit Result duration perturb budget matrixwalktime matrixeffort matrixconstant T
     % Compute the cost of each member in Population
    budget = 1000000;
    T = 0;             % Set T = length of walk duration 
    pfmag = 10;		    % how fast the perturbation grows (N/s)
    W(1) = 10;            % weighting factor for x(19)
    W(2) = .01;         % weighting factor for x(20)
    
    for popindex = 1 : length(Population)
            k = Population(popindex).chrom;
            
            duration = 5.0;		% allow for a maximum 10 second walk
            xinit = [Result.x(:,1);0;0];
            perturb.t = 0:0.1:duration;
            
            if (T>0)
            perturb.Force = pfmag.*rand(size(perturb.t));
            else            
            perturb.Force = pfmag*perturb.t.*rand(size(perturb.t));
            end 
            
            kp = k(1);                                      % first entry in k gain matrix
            kd = k(2);                                      % second entry in k gain matrix
            K1=[0 0 0 -kp 0 0 0 0 0 0 0 0 -kd 0 0 0 0 0];
            K2=[0 0 0 0 -kp 0 0 0 0 0 0 0 0 -kd 0 0 0 0];
            K3=[0 0 0 0 0 -kp 0 0 0 0 0 0 0 0 -kd 0 0 0];
            K4=[0 0 0 0 0 0 -kp 0 0 0 0 0 0 0 0 -kd 0 0];
            K5=[0 0 0 0 0 0 0 -kp 0 0 0 0 0 0 0 0 -kd 0];
            K6=[0 0 0 0 0 0 0 0 -kp 0 0 0 0 0 0 0 0 -kd];
            control.K=[K1;K2;K3;K4;K5;K6];              

            options = odeset('Events',@walk_events);
            [~,x,walktime] = ode15s(@odefun, [0 duration], xinit, options);
            
            if (T>0)
            Population(popindex).cost = W*x(end,19:20)';
            else
            Population(popindex).cost = -walktime;
            end
            
            if (T>0)
            matrixconstant = [matrixconstant; k(1) k(2) Population(popindex).cost];   
            fprintf('Gains: %8.3f %8.3f-- cost %8.3f integral of position error %10.5f\n', k, Population(popindex).cost, x(end,20));
            else
            matrixwalktime = [matrixwalktime; k(1) k(2) walktime]; 
            matrixeffort = [matrixeffort; k(1) k(2) x(end,19)];    
            fprintf('Gains: %8.3f %8.3f-- walk duration %8.3f effort %10.5f\n', k, walktime, x(end,19));
           
            end
    end

return

