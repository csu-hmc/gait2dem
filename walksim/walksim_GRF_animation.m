function walksim_GRF_animation(filename)

% This is a simple program to simulate and plot the states of the gait2dem model.
% It is based on the code from the walksim_animation version of the program 
% which uses the ground reaction forces as input to the controller

	global control grf

	% Inputs for the simulation
	T = 4;       % T = overall simulation time in seconds
	ts = .01;    % ts = time step in seconds, (0.01 is recommended)
    height = .75; % height of hip, below which the model registers a fall
    kp=250;       % proportional gain on all joints
    kd=10;        % derivative gain on all joints
    % Note: The following gains can be controlled individually or globally
    % from kp and kd inputs above, or as a combination of the two. 
    kpRhip = kp;
    kdRhip = kd;
    kpRknee = kp;
    kdRknee = kd;
    kpRankle = kp;
    kdRankle = kd;
    kpLhip = kp;
    kdLhip = kd;
    kpLknee = kp;
    kdLknee = kd;
    kpLankle = kp;
    kdLankle = kd;
    
	% load the optimization resultsim
	load(filename);
   	xinit = Result.x(:,1);			% = initial states
	N=Result.problem.N;
	
    % Gains Matrix K
    control.K=zeros(6,18);    
    K1=[0 0 0 -kpRhip 0 0 0 0 0 0 0 0 -kdRhip 0 0 0 0 0];
    K2=[0 0 0 0 -kpRknee 0 0 0 0 0 0 0 0 -kdRknee 0 0 0 0]; 
    K3=[0 0 0 0 0 -kpRankle 0 0 0 0 0 0 0 0 -kdRankle 0 0 0]; 
    K4=[0 0 0 0 0 0 -kpLhip 0 0 0 0 0 0 0 0 -kdLhip 0 0];
    K5=[0 0 0 0 0 0 0 -kpLknee 0 0 0 0 0 0 0 0 -kdLknee 0];
    K6=[0 0 0 0 0 0 0 0 -kpLankle 0 0 0 0 0 0 0 0 -kdLankle];
    
    control.K=[K1;K2;K3;K4;K5;K6];
    control.durcycle=2*Result.dur; % 2 half gait cycles + 1 data point
    control.x0 = [Result.x';Result.x(Result.problem.vmx,:)'];  
    control.u0 = [Result.u';Result.u(Result.problem.vmu,:)'];
    control.u0 = control.u0(:,4:end); % throw away the first 3 rows
    control.u0 = [control.u0 ; control.u0(1,:)]; % add data for the last missing data point
    control.x0 = [control.x0 ; control.x0(1,:)]; % add data for the last missing data point
    control.tsamples = (0:2*N)*control.durcycle/(2*N); 
    
	% Run the simulation
    tsamples = ts:ts:T;
    [t,x] = ode15s(@odefun, tsamples, xinit);

% Plot hip height of perfect openloop walk with PD model  
    plot(Result.x(2,:),'g')
    hold on
    plot(x(:,2),'b--')
    xlabel('time step number (intervals of "ts")');
    ylabel('x(2) -hip height in meters')
    legend('Result.x(2)','x(2)')
    % anim(x')
end
%=====================================================================================
function [xdot] = odefun(t,x)
	tau = [zeros(3,1); controller(t,x)];		% six joint torques are generated by a controller
    xdot = gait2dem(x,tau);
end
%=====================================================================================
function [u] = controller(t,x)
	global control grf
    tper = mod(t,control.durcycle);
    x0t = interp1(control.tsamples,control.x0,tper)';
    u0t = interp1(control.tsamples,control.u0,tper)';
    
   % Compute the GRF
    tau = zeros(9, 1);
    [~, grf] = gait2dem(x, tau);

    wf = 0.001360544;  % weight factor - reciprocal of weight = 1/(735N = 75kg * 9.8 m/s^2)
    Rside = control.K(1:3,:)*grf(2,1);
    Lside = control.K(4:6,:)*grf(5,1);
    G = [Rside;Lside]*wf;
    
    u = u0t+G*(x-x0t);
    
end
%=====================================================================================