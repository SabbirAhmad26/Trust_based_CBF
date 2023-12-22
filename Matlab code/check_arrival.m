function [car, pen, CAV_oc, maxTime, CAV_et,init_queue_carla_compatible] = check_arrival(i, init_queue, car, pen, pointer, CAV_oc, mod, maxTime,CAV_et,init_queue_carla_compatible,trajs)

global computationTime dt;
x = -50;
y = 0;

righta = find(trajs == 'Right A:');
lefta = find(trajs == 'Left A:');
straighta = find(trajs == 'straight A:');

originb = find(trajs == 'Origin Lane B:');
rightb = find(trajs == 'Right B:');
leftb = find(trajs == 'Left B:');
straightb = find(trajs == "Straight B: ");

originc = find(trajs == 'Origin Lane C:');
rightc = find(trajs == 'Right C:');
leftc = find(trajs == 'Left C:');
straightc =find (trajs == 'Straight C:');

origind = find(trajs == "Origin Lane D: ");
rightd = find(trajs == "Right D: ");
leftd = find(trajs == 'Left D:');
straightd = find(trajs == "Straight D");   


CAV_et.arrivalexit(pen,1) = i * dt;
car.cars1 = car.cars1 + 1;
len = length(car.que1)+1;
new.state = [0 init_queue(4) 0]; % pos, speed, acc
new.prestate = [-1,-1,-1];
new.phiLateral = 0.9;
new.phiRearEnd = 0.9;
new.k_lateral = 1 * ones(5,1);
new.k_rear = 1;
new.carlength = 3.47;

%%Suppose that 6th is the action
% action 1: go straight, 2: turn left, 3: turn right
% lane 1

if (init_queue(2) == 1 && init_queue(6) == 2)
    new.jindex = 31;
    new.lane = 1;
    new.id = [2 pen 1 3 6 4];  %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(leftb+1:straightb-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor+init_queue(7),initial_ycoor];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["0.0.00","25.0.00"];
    
% lane 2
elseif (init_queue(2) == 2 && init_queue(6) == 1)
    new.jindex = 31;
    new.lane = 2;
    new.id = [1 pen 2 2 8 9 10 11]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(straightb+1:originc-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor+init_queue(7),initial_ycoor];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["0.0.00","39.0.00"];
elseif (init_queue(2) == 2 && init_queue(6) == 3)
    new.jindex = 31;
    new.lane = 2;
    new.id = [3 pen 2 8 12]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(rightb+1:leftb-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor+init_queue(7),initial_ycoor];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["0.0.00","-26.0.00"];
% lane 3
elseif (init_queue(2) == 3 && init_queue(6) == 2)
    new.jindex = 31;
    new.lane = 3;
    new.id = [2 pen 3 5 9 6]; %action, vehicle_id, current_lane, destination_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(leftc+1:rightc-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor,initial_ycoor+init_queue(7)];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-29.0.00","-0.0.00"];
% lane 4
elseif (init_queue(2) == 4 && init_queue(6) == 1)
    new.jindex = 31;
    new.lane = 4;
    new.id = [1 pen 4 4 10 7 5 1];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(straightc+1:origind-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor,initial_ycoor+init_queue(7)];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-29.0.00","25.0.00"];
elseif (init_queue(2) == 4 && init_queue(6) == 3)
    new.jindex = 31;
    new.lane = 4;
    new.id = [3 pen 4 2 11];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(rightc+1:straightc-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor,initial_ycoor+init_queue(7)];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-29.0.00","39.0.00"];

% lane 5
elseif (init_queue(2) == 5 && init_queue(6) == 2)
    new.jindex = 1;
    new.lane = 5;
    new.id = [2 pen 5 7 7 9];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(leftd+1:straightd-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor-(init_queue(7)),initial_ycoor];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];  
    new.edges = ["-39.0.00","-26.0.00"];
% lane 6
elseif (init_queue(2) == 6 && init_queue(6) == 1)
    new.jindex = 1;
    new.lane = 6;
    %new.state = [200 init_queue(4) 0];
    new.id = [1 pen 6 6 5 4 3 2];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(straightd+1:end);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor-(init_queue(7)),initial_ycoor];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-39.0.00","-0.0.00"];
elseif (init_queue(2) == 6 && init_queue(6) == 3)
    new.jindex = 1;
    new.lane = 6;
    %new.state = [200 init_queue(4) 0];
    new.id = [3 pen 6 4 1];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(rightd+1:leftd-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor-(init_queue(7)),initial_ycoor];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-39.0.00","25.0.00"];
% lane 7
elseif (init_queue(2) == 7 && init_queue(6) == 2)
    new.jindex = 45;
    new.lane = 7;
    new.id = [2 pen 7 1 4 7];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(lefta+1:straighta-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor,initial_ycoor-init_queue(7)];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-16.0.00","39.0.00"];
% lane 8
elseif (init_queue(2) == 8 && init_queue(6) == 1)
    new.jindex = 45;
    new.lane = 8;
    new.id = [1 pen 8 8 3 6 8 12];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(straighta+1:originb-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor,initial_ycoor-init_queue(7)];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-16.0.00","-26.0.00"];
elseif (init_queue(2) == 8 && init_queue(6) == 3)
    new.jindex = 45;
    new.lane = 8;
    new.id = [3 pen 8 6 2];  %queue_id, vehicle_id, origin_lane, cur_lane, MP1, MP2.....MPn
    new.metric = [0, 0, 0];  %time, fuel, energy, distance, MP1_dis, MP2_dis
    str = trajs(righta+1:lefta-1);
    preinitial_xycoor = strsplit(str(new.jindex),' ');
    initial_xycoor = strsplit(str(new.jindex + 1),' ');
    initial_xcoor = str2num(replace(initial_xycoor(1), "x=",''));
    preinitial_xcoor = str2num(replace(preinitial_xycoor(1), "x=",''));
    initial_ycoor = str2num(replace(initial_xycoor(2), "y=",''));
    preinitial_ycoor = str2num(replace(preinitial_xycoor(2), "y=",''));
    new.realpose = [initial_xcoor,initial_ycoor-init_queue(7)];
    new.prerealpose = [preinitial_xcoor,preinitial_ycoor];
    new.edges = ["-16.0.00","-0.0.00"];
end
lengths = find_MPs(init_queue(2),init_queue(6),trajs,new.jindex);
new.metric = [new.metric,lengths(end),lengths'];
new.vehtype = "vehicle.micro.microlino";
new.departslane = "best";


if (mod == 1)
    new.ocpar = OCT1(dt*i, init_queue(4), new.metric(4)); %unconstrained oc sol.   a,b,c,d
elseif (mod == 2)
    if (~isempty(ip))
        [new.ocpar, new.violation] = OCT4(dt*i, init_queue(4), new.metric(4), new.tm, 1000000, car.que1{ip});
    else
        [new.ocpar, new.violation] = OCT4(dt*i, init_queue(4), new.metric(4), new.tm, 1000000, []);
    end
    
    parameter = double(new.ocpar);
    new.tm = parameter(6);
    new.tf = new.tm + new.deltaTurn;
else
    new.ocpar = CAV_oc(pointer).ocp;
end

if (mod == 1 || mod == 2)
    CAV_oc(pointer).ocp = new.ocpar;
end


row = pen;
cols = 11;
for col = 1:1:cols
    switch col
        case 1
            temp = pen;
        case 2
            temp = new.vehtype;
        case 3
            temp = CAV_et.arrivalexit(pen,1);
        case 4
            temp = new.departslane;
        case 5
            temp = new.edges(1);
        case 6
            temp = new.edges(2);
        case 7
            temp = new.state(2);
        case 8
            temp = init_queue(7);
        case 9
            temp = init_queue(8);
        case 10
            temp = init_queue(2);
        case 11
            temp = init_queue(6);
    end
    init_queue_carla_compatible{row+1,col} = temp;
end





new.trust = [0,0];
new.see= [];
new.rearendconstraint = [];
new.lateralconstraint = cell(1,5);

new.p = plot(x,y,'o','EraseMode','background');
new.p.MarkerEdgeColor = 'red';
new.p.MarkerSize = 5;
new.p.MarkerFaceColor = 'red';
new.txt = text(new.prerealpose(1), new.prerealpose(2),"" + new.id(2));
new.scores = [nan,nan,nan,nan];
new.reward = 0;
new.infeasiblity = 0;
new.regret= 0;
new.MUSTleave = 0;
new.agent = init_queue(8);
new.Warning = 0;
new.NewWarning = 0;
new.overtake = 0;

car.cars1 = car.cars1 + 1;
len = length(car.que1)+1;



car.que1{len} = new;
car.cars = car.cars + 1;

pen = pen + 1;
if (pen >= 100)
    pen = 1;
end