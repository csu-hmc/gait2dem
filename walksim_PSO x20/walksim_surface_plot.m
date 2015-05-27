function walksim_surface_plot

% This is a simple program to optimize the gait2dem model controller gains.

%=====================================================================================

global control perturb kp kd Result

lb = [1 1];             % lower bound of kd/kp for optimization
ub = [2000 100];        % upper bound of kd/dp for optimization
gridsize = OPTIONS.popsize*GenLimit*[1 1];		% 

% map the performance as a function of k
k1 = zeros(gridsize);
k2 = zeros(gridsize);
f = zeros(gridsize);
for ik1 = 1:gridsize(1)
    for ik2 = 1:gridsize(2)
        k1(ik1,ik2) = lb(1) + (ik1-1)*(ub(1)-lb(1))/(gridsize(1)-1);
        k2(ik1,ik2) = lb(2) + (ik2-1)*(ub(2)-lb(2))/(gridsize(2)-1);
		perturb.t = 0:0.1:duration;
		perturb.Force = pfmag*perturb.t.*rand(size(perturb.t));
        f(ik1,ik2) = walk([k1(ik1,ik2) k2(ik1,ik2)]);
    end
end
surfc(k1,k2,f);
xlabel('Kp (N*m/radian)')
ylabel('Kd (N*m*s/radian)');
zlabel('f (s)');
keyboard

% smooothing the response surface
kernel = [.05 .1 .05; .1 .4 .1; .05 .1 .05];
f = conv2(f,kernel,'same');
surfc(k1,k2,f);
xlabel('Kp (N*m/radian)')
ylabel('Kd (N*m*s/radian)');
zlabel('f (s)');
