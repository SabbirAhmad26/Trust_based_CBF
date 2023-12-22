clc
close all
clear all

trajs = fileread('Position Values for ABCD.txt');
width = 4;
[A,upperb,lowerb] = Carla_intersection(trajs,width);
ind = find(A.x >= 229);

plot(A.x,A.y,'LineStyle','--', 'Color','b')
hold on
% if numel(upperb)
%     plot(upperb(:,1),upperb(:,2),'LineStyle','--')
% end
% % hold on
% if numel(lowerb)
%     plot(lowerb(:,1),lowerb(:,2),'LineStyle','-')
% end
hold on

axis([229,371,126,271]);
load("outer_road4.mat");
plot(upperb(:,1),upperb(:,2),'LineStyle','-','Color','blue')
hold on
% 
% 
load("outer_road3.mat");

plot(lowerb(:,1),lowerb(:,2),'LineStyle','-','Color','blue')
hold on
load("outer_road2.mat");

plot(lowerb(:,1),lowerb(:,2),'LineStyle','-','Color','blue')
hold on
load("outer_road1.mat");

plot(upperb(:,1),upperb(:,2),'LineStyle','-','Color','blue')
hold on


hold on; plot([300.45,300.45],[126,187],'Color','blue');
hold on; plot([300.63,302.1],[210,271],'Color','blue');
hold on; plot([229,290],[199.24,199.56],'Color','blue');
hold on;plot([310,371],[200.11,200.26],'Color','blue')


