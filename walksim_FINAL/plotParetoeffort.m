function plotParetoeffort(matrixeffort,OPTIONS)

V = scatteredInterpolant(matrixeffort(:,1), matrixeffort(:,2), matrixeffort(:,3));

step = OPTIONS.MaxDomain(1)/10000;
xx = 0:step:OPTIONS.MaxDomain(1);
yy = 0:step:OPTIONS.MaxDomain(2);

[xq,yq] = meshgrid(xx,yy);
zq = V(xq,yq);

figure;
mesh(xq,yq,zq)
xlabel('kp (Nm/deg)')
ylabel('kd (Nm/deg)')
zlabel('effort')
hold on

scatter3(matrixeffort(:,1), matrixeffort(:,2), matrixeffort(:,3))