function [MinCost] = PSO_walksim(ProblemFunction, GenLimit, DisplayFlag, phi1max, phi2max, phi3max, alpha)
global matrixwalktime matrixeffort matrixconstant T
% Particle swarm optimization for optimizing a general function.

% INPUTS: ProblemFunction = the handle of the function that returns the handles of the initialization and cost functions
%         DisplayFlag = whether or not to display information during iterations and plot results
%         phi1max = cognitive constant
%         phi2max = social constant for neighborhood interaction
%         phi3max = social constant for global interaction
%         alpha = constriction coefficient scale factor
%         GenLimit = generation limit

if ~exist('ProblemFunction', 'var') || isempty(ProblemFunction)
    ProblemFunction = @walksim_surface_PSO;
end
if ~exist('DisplayFlag', 'var') || isempty(DisplayFlag)
    DisplayFlag = true;
end
if ~exist('phi1max', 'var') || isempty(phi1max)
    phi1max = 2.1304;
end
if ~exist('phi2max', 'var') || isempty(phi2max)
    phi2max = 1.0575;
end
if ~exist('phi3max', 'var') || isempty(phi3max)
    phi3max = 2;
end
if ~exist('alpha', 'var') || isempty(alpha)
    alpha = 0.9;
end
KFactor = alpha * 2 / (phi1max + phi2max + phi3max - 2);
if ~exist('GenLimit', 'var') || isempty(GenLimit)
    GenLimit = 25;
end
OPTIONS.Maxgen = GenLimit;
OPTIONS.popsize = 10;
[OPTIONS, MinCost, AvgCost, Population] = Init(DisplayFlag, ProblemFunction, OPTIONS);
OPTIONS.neighbors = 5; % size of particle swarm neighborhood

for k = 1 : OPTIONS.popsize
    Population(k).vel(1 : OPTIONS.numVar) = 0; % initialize velocities
end
pbest = Population; % personal best of each particle
nbest = Population; % neighborhood best of each particle
gbest = Population(1); % global best

% Begin the optimization loop
for GenIndex = 1 : OPTIONS.Maxgen
    % Update the global best if needed
    if Population(1).cost < gbest.cost
        gbest = Population(1);
    end
    % Update the personal best and neighborhood best for each particle
    for i = 1 : OPTIONS.popsize 
        % Update each personal best as needed
        if Population(i).cost < pbest(i).cost
            pbest(i) = Population(i);
        end
        % Update each neighborhood best as needed
        Distance = zeros(OPTIONS.popsize, 1);
        for j = 1 : OPTIONS.popsize 
            Distance(j) = norm(Population(i).chrom - Population(j).chrom);
        end
        [~, indices] = sort(Distance);
        nbest(i).cost = inf;
        for j = 2 : OPTIONS.neighbors
            nindex = indices(j);
            if Population(nindex).cost < nbest(i).cost
                nbest(i) = Population(nindex);
            end
        end
    end
    % Update the position and velocity of each particle
    for i = 1 : OPTIONS.popsize
        r = rand(6, OPTIONS.numVar);
        deltaVpersonal = phi1max * r(1,:) .* (pbest(i).chrom - Population(i).chrom);
        deltaVneighborhood = phi2max * r(2,:) .* (nbest(i).chrom - Population(i).chrom);
        deltaVglobal = phi3max * r(3,:) .* (gbest.chrom - Population(i).chrom);
        Population(i).vel = KFactor * (Population(i).vel + deltaVpersonal + deltaVneighborhood + deltaVglobal);
        Population(i).chrom = Population(i).chrom + Population(i).vel;
        Population(i).chrom = max( min(Population(i).chrom, OPTIONS.MaxDomain), OPTIONS.MinDomain);
    end 
    % Calculate cost
    Population = OPTIONS.CostFunction(Population, OPTIONS);
    % Sort from best to worst
    Population = PopSort(Population);
    MinCost(GenIndex+1) = Population(1).cost;
    AvgCost(GenIndex+1) = mean([Population.cost]);
    
    if DisplayFlag
        disp(['Gen # ', num2str(GenIndex), ': min cost = ', num2str(MinCost(GenIndex+1)), ', ave cost = ', num2str(AvgCost(GenIndex+1))]);
    end
end
Conclude(DisplayFlag, OPTIONS, Population, MinCost, AvgCost);
    if (T>0)
        plotParetoconstant(matrixconstant,OPTIONS);
    else
        plotParetowalktime(matrixwalktime,OPTIONS);
        plotParetoeffort(matrixeffort,OPTIONS);
    end
return