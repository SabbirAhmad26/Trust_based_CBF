function [car, metric,CAV_e,init_queue_carla_compatible] = init_e(total,range)
%%%%%%%%%%%%%%%%%%CBF para
car.cars = 0; 
car.cars1 = 0; 
car.cars2 = 0; 
car.Car_leaves = 0; 
car.Car_leavesMain = 0; 
car.Car_leavesMerg = 0;
car.Trusted_Car_leaves = 0;
car.que1 = [];
car.que2 = [];
car.table = [];
metric.Ave_time = 0; 
metric.Ave_u2 = 0; 
metric.Ave_eng = 0; 
metric.Ave_u2Main = 0;
metric.Ave_u2Merg = 0; 
metric.Ave_engMain = 0; 
metric.Ave_engMerg = 0; 
metric.Ave_timeMain = 0; 
metric.Ave_timeMerg = 0; 

init_queue_carla_compatible = {};
CAV_e.acc=zeros(range,total);
CAV_e.arrivalexit=zeros(total,2);
CAV_e.vel=zeros(range,total);
CAV_e.pos=zeros(range,total);
CAV_e.rear_end=NaN(range,total);
CAV_e.rear_end_CBF=NaN(range,total);
CAV_e.lateral=NaN(range,total,5);
CAV_e.lateral_CBF=NaN(range,total,5);
CAV_e.time=zeros(total,1);
CAV_e.fuel=zeros(total,1);
CAV_e.energy=zeros(total,1);
CAV_e.x_tk=cell(total,8);
CAV_e.v_tk=cell(total,8);
CAV_e.trust_tk=cell(total,6);
CAV_e.posx = zeros(range,total);
CAV_e.posy = zeros(range,total);
CAV_e.angle = zeros(range,total);

for i = 1:1:8
    for j = 1:1:total
        CAV_e.x_tk{j,i}= zeros(1,total);
        CAV_e.v_tk{j,i}= zeros(1,total);
    end
end
for i = 1:1:6
    for j = 1:1:total
        CAV_e.trust_tk{j,i}= zeros(1,total);
    end
end


CAV_e.counter=zeros(total,1);
CAV_e.trust = zeros(range,total);
CAV_e.buffer=zeros(total,3);
CAV_e.SpoofedCarsList = [];
CAV_e.SpoofedCAVsSuccedingcarlist = [];
CAV_e.numcollisions = 0;
init_queue_carla_compatible = {"id", "vehtype","arrival time","lane strategy",...
    "edge1","edge2","initial vel","initial position","agent type","lane id","decision"};