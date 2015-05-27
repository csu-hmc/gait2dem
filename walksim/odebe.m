function [tout,xout] = odebe(func,t,x0)
%Backwards eu
nsamples = length(t);
nstates = length(x0);
xout = zeros(nsamples,nstates);
tout = t;
x1 = x0;
t1 = t(1);
xout(1,:) = x1';

        for i=2:nsamples
              t2=t(i);
              x2 = fsolve(@befun,x1);
              xout(i,:) = x2';
              t1 = t2;
              x1 = x2;
        end
function z = befun(x)
        z=func(t2,x)-(x-x1)/(t2-t1);
end

end

