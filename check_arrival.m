function [car, pen, CAV_oc, maxTime, CAV_et] = check_arrival(i, init_queue, car, pen, pointer, CAV_oc, mod, maxTime,CAV_et)

global computationTime dt;
x = -50;
y = 0;

l = 7-3.5*sqrt(3); % Suppose that the vehicle change lanes along with the line whose slope is 30 degree
L = 300; % The length of control zone, which is L1 in the paper
w = 3.5; % Lane width
r = 4.0; % The raidus of the small circle for right-turn
L1 = 300;
alpha1 = asind((r+0.5*w)/(r+2.5*w));
alpha2 = asind((r+1.5*w)/(r+2.5*w));

ratio1 = alpha1 / 360;
ratio2 = alpha2 / 360;
ratio3 = 0.25 - ratio2;
ratio4 = 0.25 - ratio1;

lowerLaneChanging = 57 + l;
upperLaneChanging = 250 + l;

car.cars1 = car.cars1 + 1;
len = length(car.que1)+1;
new.state = [0 init_queue(4) 0]; % pos, speed, acc
new.prestate = [-1,-1,-1];
L_end = L * 2 + 4 * w + 2* r;
%%Suppose that 6th is the action
% action 1: go straight, 2: turn left, 3: turn right
% lane 1
if (init_queue(2) == 1 && init_queue(6) == 1)
    new.lane = 1; % original_lane
    new.id = [1 pen 1 1 18 19 20 21 22];  %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2*w, L+r+2.5*w, L+r+3.5*w];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [-1+init_queue(7),L1+1.5*w+r];
    new.prerealpose = [-2,L1+1.5*w+r];
elseif (init_queue(2) == 1 && init_queue(6) == 2)
    new.lane = 1;
    new.id = [2 pen 1 3 18 16 13 9];  %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+2.5*w)*ratio1, L+2*pi*(r+2.5*w)*ratio2, L+2*pi*(r+2.5*w)*ratio3, L+2*pi*(r+2.5*w)*ratio4];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [-1+init_queue(7),L1+1.5*w+r];
    new.prerealpose = [-2,L1+1.5*w+r];
elseif (init_queue(2) == 1 && init_queue(6) == 3)
    new.lane = 1;
    new.id = [3 pen 1 8 1 28]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+0.5*pi*(r+0.5*w)+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [-1+init_queue(7),L1+1.5*w+r];
    new.prerealpose = [-2,L1+1.5*w+r];
% lane 2
elseif (init_queue(2) == 2 && init_queue(6) == 1)
    new.lane = 2;
    new.id = [1 pen 2 2 23 25 26 24 27]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2.5*w, L+r+3.5*w L+4*w+2*r];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [-1+init_queue(7) ,L1+0.5*w+r]; 
   new.prerealpose = [-2 ,L1+0.5*w+r]; 
elseif (init_queue(2) == 2 && init_queue(6) == 2)
    new.lane = 2;
    new.id = [2 pen 2 3 1 18 16 13 9]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+2*pi*(r+2.5*w)*ratio1+l, L+2*pi*(r+2.5*w)*ratio2+l, L+2*pi*(r+2.5*w)*ratio3+l, L+2*pi*(r+2.5*w)*ratio4+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [-1+init_queue(7) ,L1+0.5*w+r]; 
    new.prerealpose = [-2 ,L1+0.5*w+r]; 
elseif (init_queue(2) == 2 && init_queue(6) == 3)
    new.lane = 2;
    new.id = [3 pen 2 8 28]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+0.5*w)*0.25];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [-1+init_queue(7) ,L1+0.5*w+r]; 
    new.prerealpose = [-2 ,L1+0.5*w+r]; 
% lane 3
elseif (init_queue(2) == 3 && init_queue(6) == 1)
    new.lane = 3;
    %new.state = [100 init_queue(4) 0];
    new.id = [1 pen 3 3 26 21 17 14 9];  %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2*w, L+r+2.5*w, L+r+3.5*w];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+2.5*w+r,-1+init_queue(7)];
    new.prerealpose = [L1+2.5*w+r,-2];
elseif (init_queue(2) == 3 && init_queue(6) == 2)
    new.lane = 3;
    %new.state = [100 init_queue(4) 0];
    new.id = [2 pen 3 5 26 20 16 11]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+2.5*w)*ratio1, L+2*pi*(r+2.5*w)*ratio2, L+2*pi*(r+2.5*w)*ratio3, L+2*pi*(r+2.5*w)*ratio4];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+2.5*w+r,-1+init_queue(7)];
    new.prerealpose = [L1+2.5*w+r,-2];
elseif (init_queue(2) == 3 && init_queue(6) == 3)
    new.lane = 3;
    %new.state = [100 init_queue(4) 0];
    new.id = [3 pen 3 2 2 27]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+0.5*pi*(r+0.5*w)+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+2.5*w+r,-1+init_queue(7)];
    new.prerealpose = [L1+2.5*w+r,-2];
% lane 4
elseif (init_queue(2) == 4 && init_queue(6) == 1)
    new.lane = 4;
    %new.state = [100 init_queue(4) 0];
    new.id = [1 pen 4 4 24 22 15 10 5];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2.5*w, L+r+3.5*w L+4*w+2*r];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+3.5*w+r, -1+init_queue(7)];
    new.prerealpose = [L1+3.5*w+r, -2];
elseif (init_queue(2) == 4 && init_queue(6) == 2)
    new.lane = 4;
    %new.state = [100 init_queue(4) 0];
    new.id = [2 pen 4 5 2 26 20 16 11];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+2*pi*(r+2.5*w)*ratio1+l, L+2*pi*(r+2.5*w)*ratio2+l, L+2*pi*(r+2.5*w)*ratio3+l, L+2*pi*(r+2.5*w)*ratio4+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+3.5*w+r, -1+init_queue(7)];
    new.prerealpose = [L1+3.5*w+r, -2];
elseif (init_queue(2) == 4 && init_queue(6) == 3)
    new.lane = 4;
    %new.state = [100 init_queue(4) 0];
    new.id = [3 pen 4 2 27];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+0.5*w)*0.25];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+3.5*w+r, -1+init_queue(7)];
    new.prerealpose = [L1+3.5*w+r, -2];
% lane 5
elseif (init_queue(2) == 5 && init_queue(6) == 1)
    new.lane = 5;
    %new.state = [200 init_queue(4) 0];
    new.id = [1 pen 5 5 15 14 13 12 11];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2*w, L+r+2.5*w, L+r+3.5*w];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [2*L1+4*w+2*r-(-1+init_queue(7)),L1+2.5*w+r];
    new.prerealpose = [2*L1+4*w+2*r-(-2),L1+2.5*w+r];
elseif (init_queue(2) == 5 && init_queue(6) == 2)
    new.lane = 5;
    %new.state = [200 init_queue(4) 0];
    new.id = [2 pen 5 7 15 17 20 25];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+2.5*w)*ratio1, L+2*pi*(r+2.5*w)*ratio2, L+2*pi*(r+2.5*w)*ratio3, L+2*pi*(r+2.5*w)*ratio4];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [2*L1+4*w+2*r-(-1+init_queue(7)),L1+2.5*w+r];
    new.prerealpose = [2*L1+4*w+2*r-(-2),L1+2.5*w+r];    
elseif (init_queue(2) == 5 && init_queue(6) == 3)
    new.lane = 5;
    %new.state = [200 init_queue(4) 0];
    new.id = [3 pen 5 4 3 5];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+0.5*pi*(r+0.5*w)+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [2*L1+4*w+2*r-(-1+init_queue(7)),L1+2.5*w+r];
    new.prerealpose = [2*L1+4*w+2*r-(-2),L1+2.5*w+r];
% lane 6
elseif (init_queue(2) == 6 && init_queue(6) == 1)
    new.lane = 6;
    %new.state = [200 init_queue(4) 0];
    new.id = [1 pen 6 6 10 9 8 7 6];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2.5*w, L+r+3.5*w L+4*w+2*r];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [2*L1+4*w+2*r-(-1+init_queue(7)), L1+3.5*w+r];
    new.prerealpose = [2*L1+4*w+2*r-(-2), L1+3.5*w+r];
elseif (init_queue(2) == 6 && init_queue(6) == 2)
    new.lane = 6;
    %new.state = [200 init_queue(4) 0];
    new.id = [2 pen 6 7 3 15 17 20 25];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+2*pi*(r+2.5*w)*ratio1+l, L+2*pi*(r+2.5*w)*ratio2+l, L+2*pi*(r+2.5*w)*ratio3+l, L+2*pi*(r+2.5*w)*ratio4+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [2*L1+4*w+2*r-(-1+init_queue(7)), L1+3.5*w+r];
    new.prerealpose = [2*L1+4*w+2*r-(-2), L1+3.5*w+r];
elseif (init_queue(2) == 6 && init_queue(6) == 3)
    new.lane = 6;
    %new.state = [200 init_queue(4) 0];
    new.id = [3 pen 6 4 5];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+0.5*w)*0.25];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [2*L1+4*w+2*r-(-1+init_queue(7)), L1+3.5*w+r];
    new.prerealpose = [2*L1+4*w+2*r-(-2), L1+3.5*w+r];
% lane 7
elseif (init_queue(2) == 7 && init_queue(6) == 1)
    new.lane = 7;
    new.id = [1 pen 7 7 8 12 16 19 25];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2*w, L+r+2.5*w, L+r+3.5*w];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+1.5*w+r, 2*L1+4*w+2*r-(-1+init_queue(7))];
    new.prerealpose = [L1+1.5*w+r, 2*L1+4*w+2*r-(-2)];
elseif (init_queue(2) == 7 && init_queue(6) == 2)
    new.lane = 7;
    new.id = [2 pen 7 1 8 13 17 22];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+2.5*w)*ratio1, L+2*pi*(r+2.5*w)*ratio2, L+2*pi*(r+2.5*w)*ratio3, L+2*pi*(r+2.5*w)*ratio4];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+1.5*w+r, 2*L1+4*w+2*r-(-1+init_queue(7))];
    new.prerealpose = [L1+1.5*w+r, 2*L1+4*w+2*r-(-2)];    
elseif (init_queue(2) == 7 && init_queue(6) == 3)
    new.lane = 7;
    new.id = [3 pen 7 6 4 6];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+0.5*pi*(r+0.5*w)+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+1.5*w+r, 2*L1+4*w+2*r-(-1+init_queue(7))];
    new.prerealpose = [L1+1.5*w+r, 2*L1+4*w+2*r-(-2)];    
% lane 8
elseif (init_queue(2) == 8 && init_queue(6) == 1)
    new.lane = 8;
    new.id = [1 pen 8 8 7 11 18 23 28];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+r+0.5*w, L+r+1.5*w, L+r+2.5*w, L+r+3.5*w L+4*w+2*r];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-(-1+init_queue(7))];
    new.prerealpose = [L1+0.5*w+r,2*L1+4*w+2*r-(-2)];
elseif (init_queue(2) == 8 && init_queue(6) == 2)
    new.lane = 8;
    new.id = [2 pen 8 1 4 8 13 17 22];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, -1, L+2*pi*(r+2.5*w)*ratio1+l, L+2*pi*(r+2.5*w)*ratio2+l, L+2*pi*(r+2.5*w)*ratio3+l, L+2*pi*(r+2.5*w)*ratio4+l];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-(-1+init_queue(7))];
    new.prerealpose = [L1+0.5*w+r,2*L1+4*w+2*r-(-2)];    
elseif (init_queue(2) == 8 && init_queue(6) == 3)
    new.lane = 8;
    new.id = [3 pen 8 6 6];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0, L_end, L+2*pi*(r+0.5*w)*0.25];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    new.realpose = [L1+0.5*w+r,2*L1+4*w+2*r-(-1+init_queue(7))];
    new.prerealpose = [L1+0.5*w+r,2*L1+4*w+2*r-(-2)];    
end

% For OC method, update the desired arrival time to the conflict zone
timeSet = zeros(1, 6);
deltaStraight = 4;
deltaLeft = 4;
deltaRight = 2;
deltaSafety = 1.8;
if (new.id(1) == 1)
    new.deltaTurn = deltaStraight;
elseif (new.id(1) == 2)
    new.deltaTurn = deltaLeft;
else
    new.deltaTurn = deltaRight;
end
    
if (mod == 2)
    timeSet(1) = access_time_min(L, new.state(2)) + i * dt;
    ip = [];
    
    flag = [0 0 0 0];
    for k = length(car.que1) : -1 : 1
        A = car.que1{k}.id(5:end);
        B = new.id(5:end);
        if (car.que1{k}.id(3) == new.id(3))
            if (~flag(2))
                timeSet(4) = car.que1{k}.tf - new.deltaTurn;
                timeSet(3) = car.que1{k}.tm + deltaSafety;
                flag(2) = 1;
                ip = k;
            end
        elseif (isempty(intersect(A,B)))
            if (~flag(1))
                timeSet(6) = car.que1{k}.tf - new.deltaTurn;
                flag(1) = 1;
            end
        elseif (car.que1{k}.id(4) == new.id(4))
            if (~flag(3))
                timeSet(2) = car.que1{k}.tf + deltaSafety - new.deltaTurn;
                flag(3) = 1;
            end
        elseif (~flag(4))
            timeSet(5) = car.que1{k}.tf;
            flag(4) = 1;
        end
        
        if (sum(flag) == 4)
            break;
        end
    end
    
    new.tm = max(timeSet);
    new.tf = new.tm + new.deltaTurn;
    
end

tic
if (mod == 1)
    new.ocpar = OCT1(dt*i, init_queue(4), new.metric(4)); %unconstrained oc sol.   a,b,c,d
elseif (mod == 2)
    if (~isempty(ip))
        [new.ocpar, new.violation] = OCT4(dt*i, init_queue(4), L, new.tm, 1000000, car.que1{ip});
    else
        [new.ocpar, new.violation] = OCT4(dt*i, init_queue(4), L, new.tm, 1000000, []);
    end
    %new.ocpar = OCT2(0.1*i, init_queue(4), tm, L, 0);
    
    parameter = double(new.ocpar);
    new.tm = parameter(6);
    new.tf = new.tm + new.deltaTurn;
else
    new.ocpar = CAV_oc(pointer).ocp;
end
recordTime = toc;
computationTime = [computationTime, recordTime];

if (mod == 1 || mod == 2)
    CAV_oc(pointer).ocp = new.ocpar;
end

if (new.metric(5) == -1)
    index = search_i_p(car.que1, new);
    if(numel(index) == 0)
        new.metric(5) = upperLaneChanging;
    else
        new.metric(5) = get_L_1(dt*i, new.ocpar, car.que1{index}.ocpar);
    end
    if (new.metric(5) < lowerLaneChanging)
        new.metric(5) = lowerLaneChanging;
    end
    if (new.metric(5) > upperLaneChanging)
        new.metric(5) = upperLaneChanging;
    end
end

new.trust = [0,0];
new.see= [];
new.rearendconstraint = [];
new.lateralconstraint = [];

new.p = plot(x,y,'o','EraseMode','background');
new.p.MarkerEdgeColor = 'red';
new.p.MarkerSize = 5;
new.p.MarkerFaceColor = 'red';
new.scores = [nan,nan,nan,nan];
new.reward = 0;
new.infeasiblity = 0;
new.regret= 0;
new.MUSTleave = 0;
% new.agent = init_queue(8);
new.Warning = 0;
new.NewWarning = 0;

car.cars1 = car.cars1 + 1;
len = length(car.que1)+1;
car.que1{len} = new;
car.cars = car.cars + 1;
CAV_et.arrivalexit(pen,1) = i * dt;
pen = pen + 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%for OCBF
% if(CAV_oc.veh(12) == 3)
%     rto = OCT1(0.1*i, init_queue(4)); 
%     que.ocpar(car.cars, :) = [rto(1), rto(2), rto(3), rto(4)];
% else
%     que.ocpar(car.cars, :) = CAV_oc.par;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%end
if (pen >= 100)
    pen = 1;
end