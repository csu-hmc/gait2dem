function plotParetoconstant(matrixconstant,OPTIONS)

V = scatteredInterpolant(matrixconstant(:,1), matrixconstant(:,2), matrixconstant(:,3));

step = OPTIONS.MaxDomain(1)/10000;
xx = 0:step:OPTIONS.MaxDomain(1);
yy = 0:step:OPTIONS.MaxDomain(2);

[xq,yq] = meshgrid(xx,yy);
zq = V(xq,yq);

figure;
mesh(xq,yq,zq)
xlabel('kp (Nm/deg)')
ylabel('kd (Nm/deg)')
zlabel('cost')
hold on

scatter3(matrixconstant(:,1), matrixconstant(:,2), matrixconstant(:,3))