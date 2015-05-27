    function [xdot] = odefun(t,x)                          % function to calculate perturb force,
        % joint torques, and state derivatives
        global perturb

        Force = interp1(perturb.t,perturb.Force,t);        % Perturbation force acting on x1 (hip)
   
        [u,uf] = controller(t,x(1:18));
        tau = [Force; 0; 0; u];
        xdot = zeros(19,1);
        xdot(1:18) = gait2dem(x(1:18),tau);
        xdot(19) = sum(uf.^2);
    return


