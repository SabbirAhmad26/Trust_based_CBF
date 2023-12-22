function [lg, frame1] = Copy_of_draw_intersection_twolane(trajs)

figure1 = figure;
frame1 = axes('Parent',figure1,...
    'Position',[0.108928571428571 0.107142857142857 0.8 0.8]);
% axis(frame1);
lg = [];
width = 4;
[A,upperb,lowerb] = Carla_intersection(trajs,width);
ind = find(A.x >= 229);
lg= [];
plot(A.x,A.y,'LineStyle','--', 'Color','b')
hold on


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


hold on; plot([300.45,300.45],[110,187],'Color','blue');
hold on; plot([300.63,303.0],[210,292],'Color','blue');
hold on; plot([209,290],[199.24,199.56],'Color','blue');
hold on;plot([310,371],[200.11,200.26],'Color','blue');
axis([229,371,126,271]);