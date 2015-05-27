function walksim_grid

% This is a simple program to optimize the gait2dem model controller gains.

%=====================================================================================

global OPTIONS gridsize Cstructure 
     
lb = OPTIONS.MinDomain;             % lower bound of kd/kp for optimization
ub = OPTIONS.MaxDomain;             % upper bound of kd/dp for optimization		% 

% map the performance as a function of k
k1 = zeros(gridsize(1));
k2 = zeros(gridsize(2));
f = zeros(gridsize(1));
for ik1 = 1:gridsize(1)
    for ik2 = 1:gridsize(2)
        k1(ik1,ik2) = lb(1) + (ik1-1)*(ub(1)-lb(1))/(gridsize(1)-1);
        k2(ik1,ik2) = lb(2) + (ik2-1)*(ub(2)-lb(2))/(gridsize(2)-1);    
        f(ik1,ik2) = walk([k1(ik1,ik2) k2(ik1,ik2)]);
    end
end
subplot(1,2,1)
surfc(k1,k2,f);
xlabel('Kp (N*m/radian)')
ylabel('Kd (N*m*s/radian)');
if (Cstructure == 0)
    zlabel('walktime (s)');
else
    zlabel('cost');
end

% smooothing the response surface
subplot(1,2,2)
kernel = [.05 .1 .05; .1 .4 .1; .05 .1 .05];
f = conv2(f,kernel,'same');
surfc(k1,k2,f);
xlabel('Kp (N*m/radian)')
ylabel('Kd (N*m*s/radian)');
if (Cstructure == 0)
    zlabel('walktime (s)');
else
    zlabel('cost');
end

