clc; 
clear; 
close all;
%% 
set(0, 'DefaultTextInterpreter', 'latex')
%load('inter_queue.mat');  
%load('matlab23.mat');  
load('sumoAsymmetricalTwoLane.mat');  
[total, ~] = size(init_queue); %total = total; init_queue = case1;  load('CAV_oc.mat');

global computationTime;
global computationTime1;

init_queue(:,7)= 0;
init_queue(:,8)= 0;

run_time_driven=1; 
run_event_driven=1;
Plot = 0;
% load from vissim data
rng(1, 'twister');

 
warning('off','Matlab:hg:EraseModeIgnored');
global beta s1 s2 s3 noise1 noise2          % 0.0606 0.6666 2, 4, 6, 9
% beta = 2*(0.4*10)^2/12; % trade-off 
beta = 1;
global cnt dt;  % infeasible cnts for CBF
cnt = 0;  L = 300;  mode = 1;  % 1 - running in real time, 2 - running after obtaining OC for all CAVs (to record video)
dt = 0.1; % discretization time for time driven & sensor sampling time for event driven
s1 = 2.5;
s2 = 2.5;
s3 = 0.1;


queue_cbf = zeros(total,3);  %for plot
if(mode == 1 || mode == 2)
    CAV_oc(1).ocp = [];  
%else
 %   load('CAV_oc.mat');
end

[car, metric] = init(); % OCBF para. initialize
pen = 1; 
pointer = 1;   %77
% [p, p_s, lg, frame1, p2, p_s2, frame2] = draw_intersection_twolane();
[lg, frame1] = draw_intersection_twolane(); 
% pause(15); ready = 'ready'  
%  pause(3);
total = 5;
order = [];
identityOrder = [];
% if we further divide the conflict zone into subzones or merging points
% maxTime is a vector containing the max time to each component.
% if we regard the conflict zone as a whole, then it is a scalar.
maxTime = -10; 

range = init_queue(end, 3)*10;

CAV_e.acc=zeros(range,total);
CAV_e.arrivalexit=zeros(total,2);
CAV_e.vel=zeros(range,total);
CAV_e.pos=zeros(range,total);
CAV_e.rear_end=zeros(range,total);
CAV_e.rear_end_CBF=zeros(range,total);
CAV_e.lateral=zeros(range,total);
CAV_e.lateral_CBF=zeros(range,total);
CAV_e.time=zeros(total,1);
CAV_e.fuel=zeros(total,1);
CAV_e.energy=zeros(total,1);
CAV_e.x_tk=zeros(total,7);
CAV_e.v_tk=zeros(total,7);
CAV_e.trust_tk=zeros(total,6);
CAV_e.counter=zeros(total,1);
CAV_e.trust = zeros(range,total);
CAV_e.buffer=zeros(total,3);
        


CAV_t.acc=zeros(range,1);
CAV_t.arrivalexit=zeros(total,2);
CAV_t.vel=zeros(range,1);
CAV_t.pos=zeros(range,1);
CAV_t.rear_end=zeros(range,total);
CAV_t.rear_end_CBF=zeros(range,total);
CAV_t.lateral=zeros(range,total);
CAV_t.lateral_CBF=zeros(range,total);
CAV_t.fuel=zeros(total,1);
CAV_t.energy=zeros(total,1);
CAV_t.time=zeros(total,1);
CAV_t.counter=zeros(total,1);
CAV_t.buffer=zeros(total,3);
CAV_t.trust = zeros(range,total);

MultipleConstraints = 0;

trust_thereshold.high = 0.8;
trust_thereshold.low = 0.2;
% trust_thereshold.lowpercentage1 = 0.15;
trust_thereshold.lowpercentage1 = 0.01;
trust_thereshold.lowpercentage2 = 0;


if(run_time_driven==1) 
    for i = 0 : init_queue(end, 3)*10  %2000
   
        
        lg.step.String = num2str(i);
        
        lastCars = car.cars;
        %% From if to while, if there are multiple vehicles entering into the control at the same time
        while(pointer <= total && i == init_queue(pointer,3)*10)
            % FIFO
            [car, pen, CAV_oc, maxTime, CAV_t] = check_arrival(i, init_queue(pointer,:), car, pen, pointer, CAV_oc, mode, maxTime, CAV_t);
            pointer = pointer + 1;
        end
        
        %%%%% FIFO
        if (car.cars > 0)
            order = zeros(1, length(car.que1));
            for j = 1 : length(car.que1)
                order(j) = j;
            end
            car = update_table(car, order);
        end
        
        %tic %%%%%%%%%%%%%CBF  %toc
        if (car.cars ~= 0)
            [car,CAV_t] = main_OCBF_time(i, car, 1, trust_thereshold,MultipleConstraints, CAV_t);  % calculate and update
        end
        if (car.cars1 > 0)
            index = check_leave(car.que1); % check CAVs that leave the CZ in queue1
            if(numel(index))
                CAV_t.arrivalexit(car.que1{index}.id(2),2) = i * dt;
                for kkk = 1 : 1 : length(car.table)
                    if (car.table{kkk}(31) == index)
                        car.table(kkk) = [];
                        break;
                    end
                end
                for tt = 1 : 1 : length(car.table)
                    if (car.table{tt}(31) > index)
                        car.table{tt}(31) = car.table{tt}(31) - 1;
                    end
                end
                [car, metric] = main_CBF_update(car, metric, lg, index, 1);
                queue_cbf(car.Car_leaves,:) = [car.Car_leaves, metric.Ave_time, metric.Ave_u2];
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(car.cars ~= 0)
        car = update_positions_twolane(car, frame1); % update plot
    end
        
    end
        sumt_t=0;
        sumf_t=0;
        sume_t=0;
%         sum_counter_t=0;
        for i=1:1:total
            sumt_t=sumt_t+CAV_t.time(i,1);
            sumf_t=sumf_t+CAV_t.fuel(i,1);
            sume_t=sume_t+CAV_t.energy(i,1);
%             sum_counter_t=sum_counter_t+CAV_t.counter(i,1);
        end
        avgt_t=sumt_t/total;
        avge_t=sume_t/total;
        avgf_t=sumf_t/total;
end

if(run_event_driven==1)
    [car, metric] = init();
    pen = 1; pointer =1;
    for i = 0 : init_queue(end, 3)*10  %2000
    
        
        lg.step.String = num2str(i);
        
        lastCars = car.cars;
        %% From if to while, if there are multiple vehicles entering into the control at the same time
        while(pointer <= total && i == init_queue(pointer,3)*10)
            % FIFO
            [ car, pen, CAV_oc, maxTime, CAV_e] = check_arrival(i, init_queue(pointer,:), car, pen, pointer, CAV_oc, mode, maxTime, CAV_e);
            pointer = pointer + 1;
        end
        
        %%%%% FIFO
        if (car.cars > 0)
            order = zeros(1, length(car.que1));
            for j = 1 : length(car.que1)
                order(j) = j;
            end
            car = update_table(car, order);
        end
        
        %tic %%%%%%%%%%%%%CBF  %toc
        if (car.cars ~= 0)
            [car,CAV_e] = main_OCBF_event(i, car, mode, trust_thereshold,MultipleConstraints, CAV_e);  % calculate and update
        end
        if (car.cars1 > 0)
            index = check_leave(car.que1); % check CAVs that leave the CZ in queue1
            if(numel(index))
                CAV_e.arrivalexit(car.que1{index}.id(2),2) = i * dt;
                for kkk = 1 : 1 : length(car.table)
                    if (car.table{kkk}(31) == index)
                        car.table(kkk) = [];
                        break;
                    end
                end
                for tt = 1 : 1 : length(car.table)
                    if (car.table{tt}(31) > index)
                        car.table{tt}(31) = car.table{tt}(31) - 1;
                    end
                end
                [car, metric] = main_CBF_update(car, metric, lg, index, 1);
                queue_cbf(car.Car_leaves,:) = [car.Car_leaves, metric.Ave_time, metric.Ave_u2];
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(car.cars ~= 0)
       car = update_positions_twolane(car, frame1); % update plot
    end
        
    end
        sumt_e=0;
        sumf_e=0;
        sume_e=0;
%         sum_counter_t=0;
        for i=1:1:total
            sumt_e=sumt_e+CAV_e.time(i,1);
            sumf_e=sumf_e+CAV_e.fuel(i,1);
            sume_e=sume_e+CAV_e.energy(i,1);
%             sum_counter_t=sum_counter_t+CAV_t.counter(i,1);
        end
        avgt_t=sumt_e/total;
        avge_t=sume_e/total;
        avgf_t=sumf_e/total;
end

if Plot == 1
    %% Time triggered
    % Acceleration
    figure;
    for i = 1:1:total
        traveltime = CAV_t.arrivalexit(i,1):0.1:CAV_t.arrivalexit(i,2);
        subplot(3,1,1)
        plot(traveltime,CAV_t.acc(CAV_t.arrivalexit(i,1)/dt:CAV_t.arrivalexit(i,2)/dt,i))
        hold on
    end
    title("Time driven metrics")
    xlabel("time")
    ylabel("Acceleration")
    % Velocity
    for i = 1:1:total
        traveltime = CAV_t.arrivalexit(i,1):0.1:CAV_t.arrivalexit(i,2);
        subplot(3,1,2)
        plot(traveltime,CAV_t.vel(CAV_t.arrivalexit(i,1)/dt:CAV_t.arrivalexit(i,2)/dt,i))
        hold on
    end
    xlabel("time")
    ylabel("Velocity")
    % Position
    for i = 1:1:total
        traveltime = CAV_t.arrivalexit(i,1):0.1:CAV_t.arrivalexit(i,2);
        subplot(3,1,3)
        plot(traveltime,CAV_t.pos(CAV_t.arrivalexit(i,1)/dt:CAV_t.arrivalexit(i,2)/dt,i))
        hold on
    end
    xlabel("time")
    ylabel("Position")

%% Event triggered
    figure;
    for i = 1:1:total
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        subplot(3,1,1)
        plot(traveltime,CAV_e.acc(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,2)/dt,i))
        hold on
    end
    title("Event triggered metrics")
    xlabel("time")
    ylabel("Acceleration")
    % Velocity
    for i = 1:1:total
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        subplot(3,1,2)
        plot(traveltime,CAV_e.vel(CAV_e.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,2)/dt,i))
        hold on
    end
    xlabel("time")
    ylabel("Velocity")
    % Position
    for i = 1:1:total
        traveltime = CAV_e.arrivalexit(i,1):0.1:CAV_e.arrivalexit(i,2);
        subplot(3,1,3)
        plot(traveltime,CAV_e.pos(CAV_t.arrivalexit(i,1)/dt:CAV_e.arrivalexit(i,2)/dt,i))
        hold on
    end
    xlabel("time")
    ylabel("Position")
end