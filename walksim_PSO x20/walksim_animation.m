function walksim_animation(filename)

% This is a simple program to simulate and animate the gait2dem model.
% It is based on the code from the walksim_be_pd_M version of the program 
% which uses a pd controller to set the gains for each joint

	global control perturb budget
    
	% Inputs for the simulation
	T = 5;       % T = overall simulation time in seconds
	ts = .01;    % sampling time for simulation result, (0.01 s is recommended)
 	tsp = .1;    % sampling time for perturbation force, (0.1 s is recommended)
    height = .75; % height of hip, below which the model registers a fall
    kp=5000;       % proportional gain on all joints
    kd=500;        % derivative gain on all joints
    pfmag = 10; % How fast the perturbation force grows (newtons per second)
    pfsign = 0; % Sign of perturbation function.Set = 1 for positive only, set = 0 for pos and negative  
    an = 1; % For animation, set to < 0
    pl = 1; % For plotting optimized x(2) and x(2), set to > 0
    plforce = 1; % For plotting the perturbation force, set to > 0
    plgrf = 1; % For plotting the ground reaction forces, set to > 0
    pleff = 1; % For plotting the effort, set to > 0
    W(1) = 10;            % weighting factor for x(19)
    W(2) = .01;         % weighting factor for x(20)
    budget = 50000; %upper limit of effort allowed in walk_events function
    pltorque = 1; % For plotting the joint torques, set to > 0
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
	load(filename);  %filename is one of the 'result' files
    xinit = [Result.x(:,1);0;0];  % = initial states
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
    control.u0 = [Result.u';Result.u(Result.problem.vmu, :)'];
    control.u0 = control.u0(:,4:end); % throw away the first 3 rows
    control.u0 = [control.u0 ; control.u0(1,:)]; % add data for the last missing data point
    control.x0 = [control.x0 ; control.x0(1,:)]; % add data for the last missing data point
    control.tsamples = (0:2*N)*control.durcycle/(2*N); 
    
    % Perturbation Force Generation
    %rng(0);                 % reset the random number generator
    perturb.t = 0:tsp:T;
    if pfsign > 0 
        perturb.Force = pfmag.*rand(size(perturb.t));
    else
        perturb.Force = pfmag.*randn(size(perturb.t));
    end
	% Do the simulation and determine GRF
	tsamples = 0:ts:T;
    [t,x] = ode15s(@odefun, tsamples, xinit);
    fprintf(' cost %8.3f integral of position error %10.5f\n',  W*x(end,19:20)', x(end,20));
% 	[~,grf] = gait2dem(x(1:18,:)', zeros(9,size(x(1:18),1)));		% computes GRF for the entire state history x(t)   
    u = controller(t,x(:,1:18)')';
    budgetplot = budget.*ones(size(tsamples));
    % Animation
    if an > 0
    disp('simulation finished, creating animation...');
    anim(x(:,1:18)', interp1(perturb.t, perturb.Force, tsamples));
    end
    
    % Compare hip height of perfect openloop walk with PD model and pert force 
    figure
    if pl > 0
        subplot(2,2,1)
	    tper = mod(tsamples,control.durcycle);
		openloopx2 = interp1(control.tsamples,control.x0(:,2),tper);	
        plot(tsamples, openloopx2,'g')
        hold on
        plot(tsamples, x(:,2),'b--')
        title('Result.x(2) and x(2) vs. Time Steps')
        xlabel('time (s)');
        ylabel('x(2): hip height (m)')
        legend('unperturbed','perturbed')
    end
    if plforce > 0  
        subplot(2,2,2)
        plot(perturb.t,perturb.Force,'b')
        title('Perturbation Force')
        xlabel('time (s)');
        ylabel('Perturbation Force (N)')
    end
    if plgrf > 0
        subplot(2,2,3)
%         plot(tsamples, grf(2,:),'b')
%         hold on
%         plot(tsamples, grf(5,:),'g')
%         title('Ground Reaction Forces')
%         xlabel('time (s)');
%         ylabel('vertical GRF (N)')
%         legend('Right Foot','Left Foot')
    end
    if pleff > 0
        subplot(2,2,4)
        plot(tsamples, x(:,19),'b')
        hold on
        plot(tsamples,budgetplot,'g')
        title('Effort and Budget')
        xlabel('time (s)');
        ylabel('Magnitude (Nm)^3')
        legend('Effort','Budget')
        ylim([0 budget*1.1])
    end 
    if pltorque >0
        figure
        Rhip_torque = u(:,1);
            subplot (3,2,1)
            plot(tsamples,Rhip_torque); 
            title('Right Hip Torque')
            xlabel('time (s)');
            ylabel('Right Hip Torque (Nm)')
        Rknee_torque = u(:,2);
            subplot (3,2,3)
            plot(tsamples,Rknee_torque);
            title('Right Knee Torque')
            xlabel('time (s)');
            ylabel('Right Knee Torque (Nm)')
        Rankle_torque = u(:,3);
            subplot(3,2,5)
            plot(tsamples,Rankle_torque);
            title('Right Ankle Torque')
            xlabel('time (s)');
            ylabel('Rightt Ankle Torque (Nm)')
        Lhip_torque = u(:,4);
            subplot (3,2,2)
            plot(tsamples,Lhip_torque); 
            title('Left Hip Torque')
            xlabel('time (s)');
            ylabel('Left Hip Torque (Nm)')
        Lknee_torque = u(:,5);
            subplot (3,2,4)
            plot(tsamples,Lknee_torque);
            title('Left Knee Torque')
            xlabel('time (s)');
            ylabel('Left Knee Torque (Nm)')
        Lankle_torque = u(:,6);
            subplot(3,2,6)
            plot(tsamples,Lankle_torque);
            title('Left Ankle Torque')
            xlabel('time (s)');
            ylabel('Left Ankle Torque (Nm)')
    end 
end
