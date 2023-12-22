function [rt,RearEnd_CBF,Lateral_CBF,infeasiblity] = StrategicAttacker(i, one, que, ip, index, position, geometry_road) % for vehicles in the different lane
global dt u cnt  

vMax = 15;
infeasiblity = 0;
Lateral_CBF = [];
RearEnd_CBF = [];

if (one.state(1) > one.metric(4))
    v = one.state(2);
    u1 = 0;
    x = one.state(1) + v * dt;
    rt = [x, v, u1];
    return;
end

x0 = one.state; 
c = double(one.ocpar); 
t = dt*i;  
eps = 10; 
psc = 0.1; 
%   Fr = f0 + f1*x0(2) + f2*x0(2)^2;

phiRearEnd = 1.8;
phiLateral = 1.8;
deltaSafetyDistance = 3.4;

l = 7-3.5*sqrt(3); % Suppose that the vehicle change lanes along with the line whose slope is 30 degree

%% physical limitations on control inputs
umax = 5;
umin = -5;
A = [1 0; -1 0];    b = [umax; -umin];

%% reference trajector
%     if i <= 150
%         vd = max(0.5*c(1)*t^2 + c(2)*t + c(3),vMax);
%         u_ref = min(c(1)*t + c(2),0);
%     else
        vd = 1;
        u_ref = 0;
%     end

%% CLF
phi0 = -eps * (x0(2) - vd)^2;
phi1 = 2*(x0(2) - vd);
A = [A; [phi1 -1]];   b = [b; phi0];


%% From Coordinator
% if(ip ~= -1)
%     k_rear = one.k_rear;
%     s0 = que{ip}.state(1);
%     CAV_t.rear_end(i,one.id(2)) = s0 - x0(1) - phiRearEnd * x0(2) - deltaSafetyDistance;
%     CAV_t.rear_end_CBF(i,one.id(2)) = que{ip}.state(2) - x0(2) - phiRearEnd * x0(3) + k_rear * (s0 - x0(1) - phiRearEnd * x0(2) - deltaSafetyDistance);
%     v0 = que{ip}.state(2);
%     
%     LfB= v0 - x0(2) + k_rear * (s0 - x0(1) - phiRearEnd * x0(2) - deltaSafetyDistance);
%     LgB=phiRearEnd;
%     A = [A; [LgB 0]];  b = [b; LfB];
% 
% end

    
%% lateral safety constraints

% for k = 1 : length(index)
% 
% 
%     if (index(k) == -1)
%        continue;
%     else
%         d1 = que{index(k)}.metric(position(k)+4) - que{index(k)}.state(1);  % distance to the merging point
%         d2 = one.metric(k+4) - x0(1); % distance to the merging point
%     end
% 
%     k_lateral = one.k_lateral(k);
%     L = one.metric(k+4); % total distance to the merging point
% 
% 
%     bigPhi = phiLateral * x0(1) / L;
% 
%     CAV_t.lateral(i,one.id(2),k) =   d2 - d1- bigPhi * x0(2) - deltaSafetyDistance;
%     CAV_t.lateral_CBF(i,one.id(2),k) = que{index(k)}.state(2) - x0(2) - phiLateral*x0(2)*x0(2)/L - bigPhi*x0(3) +  k_lateral * (d2 - d1 - bigPhi*x0(2) - deltaSafetyDistance);
%     v0 = que{index(k)}.state(2); 
% 
%     h = d2 - d1 - bigPhi * x0(2);
%         LgB = bigPhi;
%         LfB = v0 - x0(2) - k_lateral * (phiLateral*x0(2)*x0(2)/L- deltaSafetyDistance);
%         if (LgB ~= 0)
%             A = [A;[LgB 0]]; b = [b; LfB + k_lateral * h];
%         end
% end

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
       cnt = cnt + 1       
       if (x0(2) >= 0)
          u = [umin 0];
       else
           u = [0 0];
       end
    end
    a = u(1);
    t=[0 0.1];
    [~,xx]=ode45('second_order_model',t,x0(1:2));
     rt = [xx(end, 1) , xx(end, 2), a];
        
    if (rt(2) < 0)
        rt(2) = 0;
    end
    
    if (rt(1) < 0)
        pause(1);
    end
end