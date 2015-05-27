function walksim_animation(filename)

% This is a simple program to simulate and animate the gait2dem model.
% It is based on the code from the walksim_be_pd_M version of the program 
% which uses a pd controller to set the gains for each joint

	global control perturb budget tsamples an pl plforce plgrf pleff pltorque xinit 
	
    [t,x] = ode15s(@odefun, tsamples, xinit);
   
 	%[~,grf] = gait2dem(x(1:18,:)', zeros(9,size(x(1:18),1)));		% computes GRF for the entire state history x(t)   
    u = controller(t,x(:,1:18)')';
    budgetplot = budget.*ones(size(tsamples));
    
    if pfsign > 0
        perturb.Force = pfmag.*rand(size(perturb.t));
    else
        perturb.Force = pfmag*perturb.t.*randn(size(perturb.t));
    end
    
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
    if pleff > 0
        subplot(2,2,3)
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
