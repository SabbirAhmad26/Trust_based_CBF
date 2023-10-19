function [rt,c1_org,c1_CBF,c2_org,c2_CBF] = OCBF_event(i, one, que, ip, index, position, ilc) % for vehicles in the different lane
global dt s1 s2 s3 u cnt noise1 noise2
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

k_lateral = one.trust(1);

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

if(ip ~= -1)
         %% find the minimum terms
%      x_ip-x_i - phi*v_i+d > 0
%      v_i - s1 < v_i < v_i + s1
%      x_i - s2 < x_i < x_i + s2
%      v_ip - s1 < v_ip < v_ip + s1
%      x_ip - s2 < x_ip < x_ip + s2


     k_rear = que{ip}.trust(1)- s3;

     s0 = que{ip}.state(1);
     c1_org = s0 - x0(1) - phiRearEnd * x0(2) - deltaSafetyDistance;
     c1_CBF = que{ip}.state(2) - x0(2) - phiRearEnd * x0(3) + k_rear * (s0 - x0(1) - phiRearEnd * x0(2) - deltaSafetyDistance);

     
     % v_i, x_i, v_ip, x_ip,
     v_tk = x0(2);
     x_tk = x0(1);
     vp_tk= que{ip}.state(2);
     xp_tk= que{ip}.state(1);
     C1_a = [phiRearEnd, +1, 0, -1];
     C1_b = -deltaSafetyDistance;
     v_a = [1, 0, 0, 0; -1, 0, 0, 0];
     v_b = [v_tk + s1; s1 - v_tk];
     x_a = [0, 1, 0, 0; 0, -1, 0, 0];
     x_b = [x_tk + s2; s2 - x_tk];
     vp_a= [0, 0, 1, 0; 0, 0, -1, 0];
     vp_b= [vp_tk + s1; s1 - vp_tk];
     xp_a= [0, 0, 0, 1; 0, 0, 0, -1];
     xp_b= [xp_tk + s2; s2 - xp_tk];
          
     A_lin = [C1_a; v_a; x_a; vp_a; xp_a];
     b_lin = [C1_b; v_b; x_b; vp_b; xp_b];
     f_lin = [-k_rear * phiRearEnd-1, -k_rear * 1, 1, k_rear* 1];
     options = optimoptions('linprog','Algorithm','interior-point','Display','off');
     [var,fval_lin,~,~,~] = linprog(f_lin,A_lin,b_lin,[],[],[],[],options);
     Lf_terms = fval_lin-deltaSafetyDistance;
     Lg_term = phiRearEnd;

     
     A = [A; [Lg_term 0]];  b = [b; Lf_terms];
 end


for k = 1 : length(index)
    if (index(k) == -1)
       continue;
    else
        v_tk = x0(2);
        x_tk = x0(1);
        vl_tk = que{index(k)}.state(2);
        xl_tk = que{index(k)}.state(1);
        d1 = que{index(k)}.metric(position(k)+4) - xl_tk;  % distance to the merging point
        d2 = one.metric(k+4) - x_tk; % distance to the merging point
    end
         k_lateral = que{index(k)}.trust(1) - s3 ;
         bigPhi = phiLateral * x0(1) / L;
         c2_org=que{index(k)}.state(1)-x0(1)-phiLateral/L*x0(1)*x0(2);
         c2_CBF=que{index(k)}.state(2) - x0(2) - phiLateral*x0(2)*x0(2)/L - bigPhi*x0(3) + k_lateral * (d2 - d1 - bigPhi*x0(2));
         x_init=[v_tk,x_tk,vl_tk,xl_tk];
         fun = @(x)k_lateral * (-phiLateral/L*x(2)*x(1)+(one.metric(k+4) - x(2))-(que{index(k)}.metric(position(k)+4)-x(4)))+x(3)-x(1)-phiLateral/L*x(1)^2;
         nonlcon = @constraint;
         c = @(x)-(one.metric(k+4) - x(2)) + (que{index(k)}.metric(position(k)+4) - x(4))+phiLateral/L*x(2)*x(1);
         ceq = @(x) x(1);
         nonlinfcn = @(x)deal(c(x),ceq(x));
         Aeq=[];
         beq=[];
         A_quad=[];
         b_quad=[];
         lb=[v_tk - s1,x_tk - s2,vl_tk - s1,xl_tk - s2];
         ub=[v_tk + s1,x_tk + s2,vl_tk + s1,xl_tk + s2];


         % [rt_slack,c1_slack,c1_CBF_slack,c2_slack,c2_CBF_slack] = OCBF_time(i, one, que, ip, i1);
         [rt_slack,~,~,~,~] = OCBF_time(i, one, que, ip, index, position, ilc); 
         rt_slack(3);
         x = fmincon(fun,x_init,A_quad,b_quad,Aeq,beq,lb,ub,nonlinfcn);
         fval_quad=-phiLateral/L*x(2)*x(1)-x(1)+ (one.metric(k+4) - x(2))-(que{index(k)}.metric(position(k)+4)-x(4))+x(3)-phiLateral/L*x(1)^2;
         if(rt_slack(3)>=0)         
            Lf_terms = fval_quad;
            Lg_term = phiLateral/L*x(2);
            A = [A; [Lg_term 0]];  b = [b; Lf_terms]; 
         else
            Lf_terms = fval_quad;
            Lg_term = phiLateral/L*(x_tk+s2);
            A = [A; [Lg_term 0]];  b = [b; Lf_terms]; 
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