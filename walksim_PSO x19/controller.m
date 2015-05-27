function [u,uf] = controller(t,x)                         % controller input 'u' as a function of time and state
    global control 
    
    tper = mod(t,control.durcycle);                         % modulo of t/control.durcycle
    x0t = interp1(control.tsamples,control.x0,tper)';       % initial position
    u0t = interp1(control.tsamples,control.u0,tper)';       % open loop control
    uf = control.K*(x-x0t);                                 % closed loop control
    u = u0t + uf;                                           % control law
    
end

