function [rt,CAV_e,infeasiblity] = OCBF_event(i, one, que, ip, index, position, ilc,CAV_e,geometry_road) % for vehicles in the different lane
global dt s1 s2 s3 u cnt noise1 noise2 const1 const2 const3 const4
vMax = 20;
infeasiblity = 0;

if i >= 465 && one.id(2) == 6
    stop = 1;
end

if (one.state(1) > one.metric(4))
    v = 10;
    u1 = 0;
    x = one.state(1) + v * dt;
    rt = [x, v, u1];
    return;
end

noise1 = const1*(rand() - const2);
noise2 = const3*(rand() - const4);
x0 = one.state; 
c = double(one.ocpar); 
t = dt*i;  
eps = 10; 
psc = 0.1; 


deltaSafetyDistance = one.carlength;
l = 7-3.5*sqrt(3); % Suppose that the vehicle change lanes along with the line whose slope is 30 degree

%% physical limitations on control inputs
umax = 3;
umin = -6;
A = [1 0; -1 0];    b = [umax; -umin];

%% reference trajectory
% if one.id(2) == 1 && i >= 25
%     vd = initialvelocity;
%     u_ref = 0;
% else
    vd = max(0.5*c(1)*t^2 + c(2)*t + c(3),15);
    u_ref = min(c(1)*t + c(2),umax);
% end



%% CLF
phi0 = -eps * (x0(2) - vd)^2;
phi1 = 2*(x0(2) - vd);
A = [A; [phi1 -1]];   b = [b; phi0];

for k = 1 : length(ip) 

         %% find the minimum terms
%      x_ip-x_i - phi*v_i+d > 0
%      v_i - s1 < v_i < v_i + s1
%      x_i - s2 < x_i < x_i + s2
%      v_ip - s1 < v_ip < v_ip + s1
%      x_ip - s2 < x_ip < x_ip + s2

     k_rear =  one.k_rear;
     phiRearEnd = one.phiRearEnd;
     
     % v_i, x_i, v_ip, x_ip,
     v_tk = x0(2);
     x_tk = x0(1);
     vp_tk= que{ip(k)}.state(2);
     xp_tk= que{ip(k)}.state(1);
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
     f_lin = [-k_rear * phiRearEnd-1- k_rear * deltaSafetyDistance, -k_rear * 1, 1, k_rear* 1];
     options = optimoptions('linprog','Algorithm','interior-point','Display','off');
     [var,fval_lin,~,~,~] = linprog(f_lin,A_lin,b_lin,[],[],[],[],options);

     if numel(fval_lin)
        Lf_terms = fval_lin;
        Lg_term = phiRearEnd;
     else    
        Lg_term = phiRearEnd;
        Lf_terms = vp_tk - v_tk + k_rear * (xp_tk - x_tk - phiRearEnd * v_tk - deltaSafetyDistance);
     end
     A = [A; [Lg_term 0]];  b = [b; Lf_terms];
 end


% for k = 1 : length(one.see) 
% %         if numel(intersect(one.see(k),ip)) 
%           if (que{one.see(k)}.id(3)== one.id(3)) && que{one.see(k)}.state(1) >= one.state(1)
%             k_rear =  one.k_rear;
%             phiRearEnd = one.phiRearEnd;
%             v_tk = x0(2);
%             x_tk = x0(1);
%             xp_tk = que{one.see(k)}.state(1); %position of cav ip
%             vp_tk = que{one.see(k)}.state(2); % velocity of cav ip
%             C1_a = [phiRearEnd, +1, 0, -1];
%             C1_b = -deltaSafetyDistance;
%             v_a = [1, 0, 0, 0; -1, 0, 0, 0];
%             v_b = [v_tk + s1; s1 - v_tk];
%             x_a = [0, 1, 0, 0; 0, -1, 0, 0];
%             x_b = [x_tk + s2; s2 - x_tk];
%             vp_a= [0, 0, 1, 0; 0, 0, -1, 0];
%             vp_b= [vp_tk + s1; s1 - vp_tk];
%             xp_a= [0, 0, 0, 1; 0, 0, 0, -1];
%             xp_b= [xp_tk + s2; s2 - xp_tk];
% 
%             A_lin = [C1_a; v_a; x_a; vp_a; xp_a];
%             b_lin = [C1_b; v_b; x_b; vp_b; xp_b];
%             f_lin = [-k_rear * phiRearEnd-1- k_rear * deltaSafetyDistance, -k_rear * 1, 1, k_rear* 1];
%             options = optimoptions('linprog','Algorithm','interior-point','Display','off');
%             [var,fval_lin,~,~,~] = linprog(f_lin,A_lin,b_lin,[],[],[],[],options);
%              if numel(fval_lin)
%                 Lf_terms = fval_lin;
%                 Lg_term = phiRearEnd;
%              else    
%                 Lg_term = phiRearEnd;
%                 Lf_terms = vp_tk - v_tk + k_rear * (xp_tk - x_tk - phiRearEnd * v_tk - deltaSafetyDistance);
%              end
%              A = [A; [Lg_term 0]];  b = [b; Lf_terms];
% 
%             %if que{one.see(k)}.MUSTleave == 1
% %             if (PositionIp - x0(1)>= 0)
% %                 disp('')
% %             else
% %                 indi = find(car.order == one_index);
% %                 inds = find(car.order == one.see(k));
% %                 temp = car.order(indi);
% %                 car.order(indi) = car.order(inds);
% %                 car.order(inds) = temp;
% %                 car = update_table(car); 
% %                 %end   
% %             end     
% 
%         end
% end


for k = 1 : length(index)
    for j = 1:1:length(index{k})
    if (index{k}(j) == -1)
       continue;
    else
        v_tk = x0(2);
        x_tk = x0(1);
        vl_tk = que{index{k}(j)}.state(2);
        xl_tk = que{index{k}(j)}.state(1);
        d1 = que{index{k}(j)}.metric(position{k}(j)+4) - xl_tk;  % distance to the merging point
        d2 = one.metric(k+4) - x_tk; % distance to the merging point
    end
         k_lateral = one.k_lateral(k);
         phiLateral = one.phiLateral;
         L = one.metric(4);

         x_init=[v_tk,x_tk,vl_tk,xl_tk];
         fun = @(x)x(3)-x(1)-phiLateral/L*x(1)^2 -phiLateral/L*x(1)*deltaSafetyDistance + k_lateral * ((one.metric(k+4) -x(2))-(que{index{k}(j)}.metric(position{k}(j)+4)-x(4)) ...
             -phiLateral/L*x(2)*(x(1)+deltaSafetyDistance));

%          fun = @(x)x(3)-x(1)-phiLateral/L*x(1)^2 + k_lateral * ((one.metric(k+4) -x(2))-(que{index{k}(j)}.metric(position{k}(j)+4)-x(4)) ...
%              -phiLateral/L*x(2)*x(1)-deltaSafetyDistance);

         nonlcon = @constraint;
%          c = @(x)-(one.metric(k+4) - x(2)) + (que{index{k}(j)}.metric(position{k}(j)+4) - x(4))+ phiLateral/L*x(2)*x(1) + deltaSafetyDistance;
         c = @(x)-(one.metric(k+4) - x(2)) + (que{index{k}(j)}.metric(position{k}(j)+4) - x(4)) + phiLateral/L*x(2)*(x(1) + deltaSafetyDistance);
         ceq = @(x) 0;
         nonlinfcn = @(x)deal(c(x),ceq(x));
         Aeq=[];
         beq=[];
         A_quad=[];
         b_quad=[];
         lb=[v_tk - s1,x_tk - s2,vl_tk - s1,xl_tk - s2];
         ub=[v_tk + s1,x_tk + s2,vl_tk + s1,xl_tk + s2];


         index_1 = index{k}(j);
         position_1 = position{k}(j);
         [rt_slack] = OCBF_time(i, one, que, ip, index_1, position_1, ilc,geometry_road); 
          
         [x,fval] = fmincon(fun,x_init,A_quad,b_quad,Aeq,beq,lb,ub,nonlinfcn);
         
         if(rt_slack(3)>=0)         
            Lf_terms = fval;
            Lg_term = phiLateral/L*(x_tk-s2);
            A = [A; [Lg_term 0]];  b = [b; Lf_terms]; 
         else
            Lf_terms = fval;
            Lg_term = phiLateral/L*(x_tk+s2);
            A = [A; [Lg_term 0]];  b = [b; Lf_terms]; 
         end

    end
 end



%% physical limitations on velocity

vmin = 0;
A_vmax = [1,0];  %CBF for max and min speed
b_vmax = vMax - x0(2);
A_vmin = [-1,0];
b_vmin = x0(2) - vmin;
A = [A; A_vmax; A_vmin];
b = [b; b_vmax; b_vmin];
            

H = [1 0; 0 psc];
F = [-u_ref; 0];  %% the objective is to let dv/dt be close to uref and let v be close to vref
options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
[u,fval,~,~,~] = ...
   quadprog(H,F,A,b,[],[],[],[],[],options);
if (numel(u) == 0)
   cnt = cnt + 1
   u = [-6 0];
   infeasiblity = 1;

end
a = u(1);
t=[0 dt];
[~,xx]=ode45('second_order_model',t,x0(1:2));
rt = [xx(end, 1), xx(end, 2), a];

if (rt(2) < 0)
    rt(2) = 0;
    rt(1) = one.state(1);
end

if (rt(1) - one.state(1) < 0)
    pause(1);
end
end