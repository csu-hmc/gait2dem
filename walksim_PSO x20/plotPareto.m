function plotPareto(matrixeffort)
global OPTIONS
V = scatteredInterpolant(matrixeffort(:,1), matrixeffort(:,2), matrixeffort(:,3));

% maxjes = ceil(100*max(matrix))/100;
% minjes = floor(100*min(matrix))/100;
% OPTIONS.MinDomain = [1 1];
% OPTIONS.MaxDomain = [2000 100];
% xx = minjes(1):0.01:maxjes(1);
% yy = minjes(2):0.01:maxjes(2);
% xx = OPTIONS.MinDomain(1):0.01:OPTIONS.MaxDomain(1);
% yy = OPTIONS.MinDomain(2):0.01:OPTIONS.MaxDomain(2);
xx = 0:01:10000;
yy = 0:01:200;

[xq,yq] = meshgrid(xx,yy);
zq = V(xq,yq);

figure;
mesh(xq,yq,zq)
xlabel('kp (Nm/deg)')
ylabel('kd (Nm/deg)')
zlabel('walktime')
hold on
% scatter3(matrix(:,1), matrix(:,2), matrix(:,3))
scatter3(matrixeffort(:,1), matrixeffort(:,2), matrixeffort(:,3))