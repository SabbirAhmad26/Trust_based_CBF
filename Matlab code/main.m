clc
close all
clear all
%% 

set(0, 'DefaultTextInterpreter', 'latex')
load('sumoAsymmetricalTwoLane3.mat');  
trajs = fileread('Position Values for ABCD.txt');
trajs = string(trajs);
trajs = splitlines(trajs);
TF = (trajs == '');
trajs(TF) = [];

[total, ~] = size(init_queue); %total = total; init_queue = case1;  load('CAV_oc.mat');

Excel_output = 1;

pyversion('C:\Users\esabo\anaconda3\python.exe')

init_queue(:,7)= 0;
init_queue(:,8)= 0;
init_queue(:,4)= 4/10 * init_queue(:,4);
% init_queue(5,8)= 1;
rng("shuffle");
Plot = 0;
% load from vissim data
% rng(1, 'twister');
sc = 4;
[init_queue,total] = ScenarioBuilder(sc);
% total = 14;
pen = 1; pointer =1;
warning('off','Matlab:hg:EraseModeIgnored');
global beta s1 s2 s3 const1 const2 const3 const4 cnt dt initialvelocity
beta = 0.1;
cnt = 0;  mode = 1;  % 1 - running in real time, 2 - running after obtaining OC for all CAVs (to record video)
dt = 0.1; % discretization time for time driven & sensor sampling time for event driven
s1 = 0.5;
s2 = 0.5;
s3 = 0.1;
const1 = 0;
const2 = 0;
const3 = 0;
const4 = 0;
initialvelocity = 3;

geometry_road.L = 52;
geometry_road.w = 4;
geometry_road.r = 2.5;


queue_cbf = zeros(total,3);  %for plot
if(mode == 1 || mode == 2)
    CAV_oc(1).ocp = [];  
end
range = 3000;
% [lg, frame1] = draw_intersection_twolane(geometry_road); 
[lg, frame1] = Copy_of_draw_intersection_twolane(trajs); 
 pause(3);

order = [];
identityOrder = [];
% if we further divide the conflict zone into subzones or merging points
% maxTime is a vector containing the max time to each component.
% if we regard the conflict zone as a whole, then it is a scalar.
maxTime = -10; 

% range = init_queue(end, 3)*10;


MultipleConstraints = 0;
trust_thereshold.high = 0.8;
trust_thereshold.low = 0.4;
trust_thereshold.lowpercentage1 = 0.01;
trust_thereshold.lowpercentage2 = 0;


[car, metric,CAV_e,init_queue_carla_compatible] = init_e(total,range);


for i = 0 : range  %2000

    
    lg.step.String = num2str(i);
    
    lastCars = car.cars;
    %% From if to while, if there are multiple vehicles entering into the control at the same time
    while(pointer <= total && i == init_queue(pointer,3)*fix(1/dt))
        % FIFO
        [ car, pen, CAV_oc, maxTime, CAV_e,init_queue_carla_compatible] = check_arrival(i, init_queue(pointer,:), car, pen, pointer, CAV_oc, mode, maxTime, CAV_e,init_queue_carla_compatible,trajs);
        pointer = pointer + 1;
        len = car.cars;
        car.order(len) = len;
        car = update_table(car);
    end
    
    %tic %%%%%%%%%%%%%CBF  %toc
    if (car.cars ~= 0)
        [car,CAV_e] = main_OCBF_event(i, car, mode, trust_thereshold, MultipleConstraints,CAV_e,geometry_road);  % calculate and update
    end
    if (car.cars1 > 0)
        [index,order,CAV_e] = check_leave(i,car, car.que1,CAV_e); % check CAVs that leave the CZ in queue1
        if(numel(index))
            [car, metric] = main_CBF_update(car, metric, lg, index, 1);
            queue_cbf(car.Car_leaves,:) = [car.Car_leaves, metric.Ave_time, metric.Ave_u2];
            car.order = order;
            car = update_table(car); 
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(car.cars ~= 0)
   [car,CAV_e] = update_positions_twolane(i,car,frame1,CAV_e,trajs); % update plot
end
    
end
collisions = sign(CAV_e.numcollisions);




if Plot == 1
%% Event triggered
    figure;
    for i = 1:1:total
        subplot(3,1,1)
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        plot(traveltime,CAV_e.acc(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,1)/dt + length(traveltime)-1,i))
        hold on
    end
    title("Event triggered metrics")
    xlabel("time")
    ylabel("Acceleration")
    % Velocity
    figure
    for i = 1:1:total
        subplot(3,1,2)
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        plot(traveltime,CAV_e.vel(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,1)/dt + length(traveltime)-1,i))
        hold on
    end
    xlabel("time")
    ylabel("Velocity")
    % Position
    for i = 1:1:total
        subplot(3,1,3)
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        plot(traveltime,CAV_e.pos(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,1)/dt + length(traveltime)-1,i))
        hold on
    end
    xlabel("time")
    ylabel("Position")

    figure;
    for i = 1:1:total
        subplot(2,1,1)
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        plot(traveltime, CAV_e.rear_end(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,1)/dt + length(traveltime)-1,i))
        hold on
        ylabel("Value")
        xlabel("time")
        title("Rear-end Original Constraint")
        subplot(2,1,2)
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        plot(traveltime, CAV_e.rear_end_CBF(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,1)/dt + length(traveltime)-1,i))
        hold on
        ylabel("Value")
        xlabel("time")
        title("Rear-end CBF Constraint")
    end
    figure;
    for i = 1:1:total
        subplot(2,1,1)
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        plot(traveltime, CAV_e.lateral(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,1)/dt + length(traveltime)-1,i))
        hold on
        ylabel("Value")
        xlabel("time")
        title("Lateral Original Constraint")
        subplot(2,1,2)
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        plot(traveltime, CAV_e.lateral_CBF(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,1)/dt + length(traveltime)-1,i))
        hold on
        ylabel("Value")
        xlabel("time")
        title("Lateral CBF Constraint")
    end
end

if Excel_output
    xlswrite('dataset_init.xlsx',init_queue_carla_compatible)
    xlswrite('dataset_xcoor.xlsx',CAV_e.posx(1:870,:)')
    xlswrite('dataset_ycoor.xlsx',CAV_e.posy(1:870,:)')
    xlswrite('dataset_angle.xlsx',CAV_e.angle(1:870,:)')
end