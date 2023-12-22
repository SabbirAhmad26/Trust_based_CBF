function [car, metric,CAV_t] = init_t(total,range)
%%%%%%%%%%%%%%%%%%CBF para
car.cars = 0; 
car.cars1 = 0; 
car.cars2 = 0; 
car.Car_leaves = 0; 
car.Car_leavesMain = 0; 
car.Car_leavesMerg = 0;
car.order = [];
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


CAV_t.acc=zeros(range,1);
CAV_t.arrivalexit=zeros(total,2);
CAV_t.vel=zeros(range,1);
CAV_t.pos=zeros(range,1);
CAV_t.rear_end=zeros(range,total);
CAV_t.rear_end_CBF=zeros(range,total);
CAV_t.lateral=zeros(range,total,5);
CAV_t.lateral_CBF=zeros(range,total,5);
CAV_t.fuel=zeros(total,1);
CAV_t.energy=zeros(total,1);
CAV_t.time=zeros(total,1);
CAV_t.counter=zeros(total,1);
CAV_t.buffer=zeros(total,3);
CAV_t.trust = zeros(range,total);
