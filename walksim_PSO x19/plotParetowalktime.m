function plotParetowalktime(matrixwalktime, OPTIONS)

V = scatteredInterpolant(matrixwalktime(:,1), matrixwalktime(:,2), matrixwalktime(:,3));

step = OPTIONS.MaxDomain(1)/10000;
xx = 0:step:OPTIONS.MaxDomain(1);
yy = 0:step:OPTIONS.MaxDomain(2);

[xq,yq] = meshgrid(xx,yy);
zq = V(xq,yq);

figure;
mesh(xq,yq,zq)
xlabel('kp (Nm/deg)')
ylabel('kd (Nm/deg)')
zlabel('walktime')
hold on

scatter3(matrixwalktime(:,1), matrixwalktime(:,2), matrixwalktime(:,3))