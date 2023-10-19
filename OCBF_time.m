function [rt,c1_org,c1_CBF,c2_org,c2_CBF] = OCBF_time(i, one, que, ip, index, position, ilc) % for vehicles in the different lane
global dt
L = 300;
vMax = 15;

if (one.state(1) > L)
    v = vMax;
    u1 = 0;
    x = one.state(1) + vMax * dt;
    rt = [x, v, u1];
    c1_org=0;
    c1_CBF=0;
    c2_org=[];
    c2_CBF=[];
    return;
end

global u cnt noise1 noise2
noise1 = 1*(rand() - 0.5);
noise2 = 1*(rand() - 0.5);
% noise1 = 0;
% noise2 = 0;

x0 = one.state; 
c = double(one.ocpar); 
t = 0.1*i;  
eps = 10; 
psc = 0.1; 
c1_org=0;
c1_CBF=0;
c2_org=[];
c2_CBF=[];

phiRearEnd = 1.8;
phiLateral = 1.8;
deltaSafetyDistance = 0;
l = 7-3.5*sqrt(3); % Suppose that the vehicle change lanes along with the line whose slope is 30 degree

%% physical limitations on control inputs
umax = 3;
umin = -3;
A = [1 0; -1 0];    b = [umax; -umin];

%% reference trajectory
vd = 0.5*c(1)*t^2 + c(2)*t + c(3);
u_ref = c(1)*t + c(2);


%% CLF
phi0 = -eps * (x0(2) - vd)^2;
phi1 = 2*(x0(2) - vd);
A = [A; [phi1 -1]];   b = [b; phi0];

%% rear-end safety constraints
if(ip ~= -1)
    s0 = que{ip}.state(1);
    c1_org = s0 - x0(1) - phiRearEnd * x0(2);
    c1_CBF =que{ip}.state(2) - x0(2) - phiRearEnd * x0(3) + s0 - x0(1) - phiRearEnd * x0(2);
    v0 = que{ip}.state(2);
    
    LfB= v0 - x0(2) + s0 - x0(1) - phiRearEnd * x0(2) - deltaSafetyDistance;
    LgB=phiRearEnd;
    A = [A; [LgB 0]];  b = [b; LfB];

end


%% lateral safety constraints
for k = 1 : length(index)

    if (index(k) == -1)
       continue;
    else
        d1 = que{index(k)}.metric(position(k)+4) - que{index(k)}.state(1);  % distance to the merging point
        d2 = one.metric(k+4) - x0(1); % distance to the merging point
    end
    
    L = one.metric(k+4); % total distance to the merging point


    bigPhi = phiLateral * x0(1) / L;

    c2_org =   d2 - d1- bigPhi * x0(2);
    c2_CBF = que{index(k)}.state(2) - x0(2) - phiLateral*x0(2)*x0(2)/L - bigPhi*x0(3) +  d2 - d1 - bigPhi*x0(2);
    v0 = que{index(k)}.state(2); 
    
    h = d2 - d1 - bigPhi * x0(2);
        LgB = bigPhi;
        LfB = v0 - x0(2) - phiLateral*x0(2)*x0(2)/L;
        if (LgB ~= 0)
            A = [A;[LgB 0]]; b = [b; LfB + h];
        end
end

%% physical limitations on velocity
vmax = 15;
vmin = 0;
A_vmax = [1,0];  %CBF for max and min speed
b_vmax = vmax - x0(2);
A_vmin = [-1,0];
b_vmin = x0(2) - vmin;
A = [A; A_vmax; A_vmin];
b = [b; b_vmax; b_vmin];
            

    H = [1 0; 0 psc];
    %F = [0; 0];
    F = [-u_ref; 0];  %% the objective is to let dv/dt be close to uref and let v be close to vref
    options = optimoptions('quadprog',...
        'Algorithm','interior-point-convex','Display','off');
    [u,fval,~,~,~] = ...
       quadprog(H,F,A,b,[],[],[],[],[],options);
    if (numel(u) == 0)
       %cnt = cnt + 1
       u = [0, 0];%[-cd*m*g 0];
    end
    a = u(1);
    t=[0 dt];
    [~,xx]=ode45('second_order_model',t,x0(1:2));
    rt = [xx(end, 1), xx(end, 2), a];
    
    if (rt(2) < 0)
        rt(2) = 0;
    end
    
    if (rt(1) < 0)
        pause(1);
    end
end