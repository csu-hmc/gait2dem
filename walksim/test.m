function [result] = test(command)

% This program runs various tests on the gait2de model

	close all

	if (nargin ~= 1)
		error('test.m needs one argument (command string)');
	end
		
	% Some model related constants
	ndof = 9;
	nstates = 2*ndof;
	joints = {'Hip' 'Knee' 'Ankle' 'LHip' 'LKnee' 'LAnkle'};
	njoints = size(joints,2);
	
	dofnames = {'trunk x','trunk y','trunk angle', ...
			'Rhip angle','Rknee angle','Rankle angle', ...
			'Lhip angle','Lknee angle','Lankle angle'};
			
	% construct a free fall state
	xff = zeros(nstates,1);
	xff(2) = 1.2;					% put model some distance above origin
	xff(4) = 15*pi/180;				% right hip flexed
	xff(5) = -15*pi/180;			% right knee flexed
	xff(7) = -15*pi/180;			% left hip extended
	xff(8) = -25*pi/180;			% left knee flexed

	% ======================= do the stickfigure test
	if strcmp(command, 'stick')
		disp('Stick figure test...');
		clf;
		stick(xff);
		return
	end
	
	% ======================= do the computation speed test
	if strcmp(command, 'speed')
		disp('Speed test...');
		tic
		Neval = 10000;
		for i=1:Neval
			x = rand(nstates,1);
			tau = rand(ndof,1);
			[xdot] = gait2dem(x,tau);
		end
		fprintf('Computation time for non-vectorized dynamics evaluation: %8.5f ms\n',1000*toc/Neval);
		x = rand(nstates,Neval);
		tau = rand(ndof,Neval);
		tic;
		[xdot] = gait2dem(x,tau);
		fprintf('Computation time for vectorized dynamics evaluation:     %8.5f ms\n',1000*toc/Neval);		
		return
	end
	
	% ======================= do the free fall dynamics test
	if strcmp(command, 'freefall')	
		disp('Freefall dynamics test...');
		
		% we use the freefall state xff constructed above
		
		% test the dynamics
		tau = zeros(ndof,1);
		xdot = gait2dem(xff,tau);
		
		% print the derivatives
		fprintf('STATE DERIVATIVES dx/dt:\n');
		fprintf('------------------------------\n');
		fprintf('velocities    accelerations   \n');
		fprintf('------------------------------\n');
		for i=1:ndof
			fprintf(' %9.4f    %9.4f\n',xdot(i),xdot(ndof+i));
		end
		fprintf('------------------------------\n');
		return
	end
	
	% ======================= do the GRF test
	if strcmp(command, 'grf')
		figure(1);clf;
		disp('GRF test...');
		x = xff;
		x(1:9) = zeros(9,1);		% put model in upright position
		x([6 9]) = 0.3;				% tilt the feet up so only heel touches the ground

		% vertical force-deformation curves
		y = 0.945:0.0001:1.0;
		ny = size(y,2);
		vy = [-1 -.5 0 .5 1];
		nvy = size(vy,2);
		Fy = zeros(ny,nvy);
		for i = 1:nvy
			x(11) = vy(i);				% vertical speed
			for j = 1:ny
				x(2) = y(j);			% vertical position
				[xdot,GRF] = gait2dem(x,zeros(ndof,1));
				Fy(j,i) = GRF(2);		% vertical GRF of right foot (heel)
			end
		end
		subplot(1,2,1);
		plot(y,Fy);
		legend([repmat('vertical speed ',nvy,1) num2str(vy')]);
		xlabel('hip height (m)');
		ylabel('Fy (N)');
		
		% horizontal force-velocity curves
		x(11) = 0;		% reset vertical velocity to zero
		vx = -.2:0.001:.2;
		nvx = size(vx,2);
		y = 0.945:0.008:0.961;
		ny = size(y,2);
		Fx = zeros(nvx,ny);
		for i = 1:ny
			x(2) = y(i);				% vertical position
			for j = 1:nvx
				x(10) = vx(j);			% horizontal speed
				[xdot,GRF] = gait2dem(x,zeros(ndof,1));
				Fx(j,i) = GRF(1);		% horizontal GRF of right foot (heel)
			end
		end
		subplot(1,2,2);
		plot(vx,Fx);
		legend([repmat('hip height ',ny,1) num2str(y')]);
		xlabel('horizontal speed (m/s)');
		ylabel('Fx (N)');
		
		return
	end
	
	% ====================== simulate freefall movement, first passive then active ==============
	if strcmp(command, 'simulate')
		disp('Simulating freefall movements...');
		close all
				
		% set simulation parameters
		duration = 4.0;
		nout = round(25*duration);
		tstep = duration/nout;
		t = (0:nout)*tstep;
		xinitial = xff;
		
		% run passive simulation
		tau = zeros(ndof,1);
		disp('Running passive freefall simulation...');
		neval = 0;
		tic;
		[tout1,xout1] = ode23(@odefun, [0 duration], xinitial);
		speed = toc/duration;
		fprintf('Number of function evaluations: %d\n', neval);
		fprintf('Number of integration steps:    %d\n', size(tout1,1));
		fprintf('Processor time / real time: %8.4f\n', speed);
		x = interp1(tout1,xout1,t);
		figure(1);
		stick(x');
		title('Passive simulation');
		save passive t x tau
		
		% run active simulation
		tau = [zeros(3,1) ; 10*ones(6,1)];
		disp('Running active freefall simulation...');
		neval = 0;
		tic;
		[tout1,xout1] = ode23(@odefun, [0 duration], xinitial);
		speed = toc/duration;
		fprintf('Number of function evaluations: %d\n', neval);
		fprintf('Number of integration steps:    %d\n', size(tout1,1));
		fprintf('Processor time / real time: %8.4f\n', speed);
		x = interp1(tout1,xout1,t);
		figure(2);
		stick(x');
		title('Active simulation');
		save active t x tau
		
		return

	end
	
	error('Command not recognized');
	
	%===================== Start of embedded functions ============================================
	function [xdot] = odefun(t,x)
		[xdot,grf,stk,mom] = gait2dem(x,tau);
		neval = neval+1;
	end
	
end
	
