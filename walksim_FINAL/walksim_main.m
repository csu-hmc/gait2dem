% This is the "control" panel for the walksim program.
% Turn functions such as animation, PSO, and surface on or off from here.
% Choose proportional and derivative gains from here
% Choose all other parameters from here such as type of cost function,
% weighting factors, effort budget, etc.

global control kp kd xinit Result duration perturb budget
global tsamples height an pfmag pfsign pl plforce plgrf pleff pltorque OPTIONS W 
global phi1max phi2max phi3max alpha Cstructure T gridsize
global matrixwalktime matrixeffort matrixcost

% Choose what functions to run
ani = 0;        % To run walksim_animation, set to 1
                % When ani = 1, the program will ignore commands for
                % Smethod and Cstructure
Smethod = 0;    % Search method
                % To run grid method, set to 0
                % To run PSO_walksim, set to 1
Cstructure = 1; % Cost structure
                % To run walking time cost function, set to 0
                % To run conventional cost function, set to 1
GainStruc = 2;  % Gain structure
                % For using the same gains on every joint, set = 2
                % For using separate gains on every joint type, set = 6

% Inputs for the simulation
% Time variables
T = 5;          % T = overall simulation time in seconds
ts = .01;       % sampling time for simulation result, (0.01 s is recommended)
tsp = .1;       % sampling time for perturbation force, (0.1 s is recommended)
tsamples = 0:ts:T;

% Walk events
height = .75;   % height of hip, below which the model registers a fall
budget = 5000;  % upper limit of allowed effort

% Perturbation Force
pfmag = 1;      % How fast the perturbation force grows (N/s)
pfsign = 2;     % Sign of perturbation function.
                % Set = 0 for pos and negative
                % Set = 1 for positive only, 
                % Set = 2 for positive and negative constant variance
%rng(0);        % reset the random number generator
duration = 100; % maximum perturbation time
perturb.t = 0:tsp:duration;

% Animation Specific Variables-----------------------------------------
an = 1;         % For animation, set to > 0
pl = 1;         % For plotting optimized x(2) and x(2), set to > 0
plforce = 1;    % For plotting the perturbation force, set to > 0
plgrf = 1;      % For plotting the ground reaction forces, set to > 0
pleff = 1;      % For plotting the effort, set to > 0
pltorque = 1;   % For plotting the joint torques, set to > 0

% Weighting Factors for Conventional Cost Function
W(1) = 1;       % weighting factor for x(19)
W(2) = 1;       % weighting factor for x(20)

% Note: The following gains can be controlled individually or globally
% from kp and kd inputs, or as a combination of the two.
if GainStruc == 2
kp=200;        % proportional gain on all joints (user input)
kd=10;          % derivative gain on all joints   (user input)
kphip   = kp;   % proportional right hip gain
kdhip   = kd;   % derivative right hip gain
kpknee  = kp;   % proportional right knee gain
kdknee  = kd;   % derivative right knee gain
kpankle = kp;   % proportional right ankle gain
kdankle = kd;   % derivative right ankle gain
k(1) = kp;      % 
k(2) = kd;      % 
elseif GainStruc == 6
kphip   = 1000; % proportional right hip gain      (user input)
kdhip   = 100;  % derivative right hip gain        (user input)
kpknee  = 500;  % proportional right knee gain     (user input)
kdknee  = 10;   % derivative right knee gain       (user input)        
kpankle = 50;   % proportional right ankle gain    (user input)
kdankle = 1;    % derivative right ankle gain      (user input)
k(1) = kphip;
k(2) = kdhip;
k(3) = kpknee;
k(4) = kdknee;
k(5) = kpankle;
k(6) = kdankle;
end

% load the optimization results
filename = 'result005';
load(filename);
xinit = [Result.x(:,1);0;0];  % = initial states
N=Result.problem.N;

% Gains Matrix K
control.K=zeros(6,18);
K1=[0 0 0 -kphip 0 0 0 0 0 0 0 0 -kdhip 0 0 0 0 0];
K2=[0 0 0 0 -kpknee 0 0 0 0 0 0 0 0 -kdknee 0 0 0 0];
K3=[0 0 0 0 0 -kpankle 0 0 0 0 0 0 0 0 -kdankle 0 0 0];
K4=[0 0 0 0 0 0 -kphip 0 0 0 0 0 0 0 0 -kdhip 0 0];
K5=[0 0 0 0 0 0 0 -kpknee 0 0 0 0 0 0 0 0 -kdknee 0];
K6=[0 0 0 0 0 0 0 0 -kpankle 0 0 0 0 0 0 0 0 -kdankle];
control.K=[K1;K2;K3;K4;K5;K6];
control.durcycle=2*Result.dur; % 2 half gait cycles + 1 data point
control.x0 = [Result.x';Result.x(Result.problem.vmx,:)'];
control.u0 = [Result.u';Result.u(Result.problem.vmu, :)'];
control.u0 = control.u0(:,4:end); % throw away the first 3 rows
control.u0 = [control.u0 ; control.u0(1,:)]; % add data for the last missing data point
control.x0 = [control.x0 ; control.x0(1,:)]; % add data for the last missing data point
control.tsamples = (0:2*N)*control.durcycle/(2*N);

% Grid Specific Variables----------------------------------------------
gridsize = [50 50];                              % number of grid divisions

% PSO Specific Variables-----------------------------------------------
phi1max = 2.1304;                           % cognitive constant
phi2max = 1.0575;                           % social constant for neighborhood interaction
phi3max = 2;                                % social constant for global interaction
alpha = 0.9;                                % constriction coefficient scale factor      
OPTIONS.Maxgen = 100;                       % generation limit
OPTIONS.popsize =100;                       % population size
OPTIONS.neighbors = 50;                     % size of particle swarm neighborhood
OPTIONS.numVar = 6;                         % number of dimensions in cost function

if GainStruc == 2
    OPTIONS.MinDomain = [0 0];                        % minimum search domain [p d]
    OPTIONS.MaxDomain = [2000 200];                   % maximum search domain [p d]
    matrixwalktime    = zeros(3);
    matrixcost        = zeros(3);
    matrixeffort      = zeros(3);
elseif GainStruc == 6
    OPTIONS.MinDomain = [0 0 0 0 0 0];                % minimum search domain [p d]
    OPTIONS.MaxDomain = [2000 200 2000 200 2000 200]; % maximum search domain [p d]
    matrixwalktime    = zeros(7);
    matrixcost        = zeros(7);
    matrixeffort      = zeros(7);
end

% Secondary Functions
if ani == 1
    walksim_animation
    Smethod = 2;
end
if Smethod == 0
    walksim_grid
elseif Smethod == 1
    ProblemFunction = @walksim_surface_PSO;
    PSO_walksim
end

