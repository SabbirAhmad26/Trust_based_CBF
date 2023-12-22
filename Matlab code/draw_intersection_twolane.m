function [lg, frame1] = draw_intersection_twolane(geometry_road)

% L = geometry_road.L;
L = geometry_road.L;
w = geometry_road.w;
r = geometry_road.r;

figure(1)
frame1 = axes('position',[0 0.1 0.8 0.8]);
axis(frame1);
% plot([-10,L],[L+r,L+r],'b',[L+w*4+2*r,L*2+w*4+2*r+10],[L+r,L+r],'b',...
%     [-10,L],[L+4*w+r,L+4*w+r],'b',[L+w*4+r*2,L*2+w*4+r*2+10],[L+4*w+r,L+4*w+r],'b',...
%     [L+r,L+r],[-10,L],'b',[L+r,L+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b',...
%     [L+4*w+r,L+4*w+r],[-10,L],'b',[L+4*w+r,L+4*w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b',... 
%     [-10,L],[L+2*w+r,L+2*w+r],'b',[L+w*4+2*r,L*2+w*4+2*r+10],[L+2*w+r,L+2*w+r],'b',...
%     [L+2*w+r,L+2*w+r],[-10,L],'b',[L+2*w+r,L+2*w+r],[L+w*4+r*2,L*2+w*4+r*2+10],'b',...  
%     [-10,L],[L+w+r,L+w+r],'b--',[L+w*4+2*r,L*2+w*4+2*r+10],[L+w+r,L+w+r],'b--',...
%     [L+w+r,L+w+r],[-10,L],'b--',[L+w+r,L+w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b--',...
%     [-10,L],[L+3*w+r,L+3*w+r],'b--',[L+w*4+2*r,L*2+w*4+2*r+10],[L+3*w+r,L+3*w+r],'b--',...
%     [L+3*w+r,L+3*w+r],[-10,L],'b--',[L+3*w+r,L+3*w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b--');





trajs = fileread('Position Values for ABCD.txt');
trajs = string(trajs);
trajs = splitlines(trajs);
TF = (trajs == '');
trajs(TF) = [];

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
A.x =  [];
A.y = [];
A.angle = [];


A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];

str = trajs(righta+1:lefta-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
end
A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];




str = trajs(lefta+1:straighta-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
end


A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];

str = trajs(straighta+1:originb-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end

A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];
%%%%% 

str = trajs(rightb+1:leftb-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end
A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];

str = trajs(leftb+1:straightb-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end

A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];
str = trajs(straightb+1:originc-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end
A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];


str = trajs(leftc+1:rightc-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end
A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];


str = trajs(rightc+1:straightc-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end

A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];

str = trajs(straightc+1:origind-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end

A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];


str = trajs(rightd+1:leftd-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end

A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];

str = trajs(leftd+1:straightd-1);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end
A.x = [A.x;NaN];
A.y = [A.y;NaN];
A.angle = [A.angle; NaN];


str = trajs(straightd+1:end);
for i =1:1:length(str)
    temp = strsplit(str(i),' ');
    x_info = temp(1);
    y_info = temp(2);
    angle_info = temp(3);
    x_info = str2num(replace(x_info, "x=",''));
    y_info = str2num(replace(y_info, "y=",''));
    angle_info = str2num(replace(angle_info, "angle=",''));
    if (x_info - 300)^2 + (y_info-200) ^2 <= 80^2
        A.x = [A.x;x_info];
        A.y = [A.y;y_info];
        A.angle = [A.angle;angle_info];
    else
        continue
    end
end

plot(A.x,A.y,'LineStyle','--','Color','r')
hold on








hold on
plot([238-10,238+L],[194-r,194-r],'b',[238+L+w*4+2*r,238+2*L+w*4+2*r+10],[194-r,194-r],'b',...
     [238-10,238+L],[4*w+194-r,4*w+194-r],'b',[238+L+w*4+r*2,238+L*2+w*4+r*2+10],[4*w+194-r,4*w+194-r],'b',...
     [295-r,295-r],[-10+124 ,136+L],'b',[305+r,305+r],[94+w*4+2*r,136+L],'b',...
     [4*w+194-r+85,4*w+194-r+85],[238-25,238+L],'b',[305+r,305+r],[238-27,238+L],'b',... 
     [238-10,238+L],[202-r,202-r],'b',[238+L+w*4+2*r,238+2*L+w*4+2*r+10],[202-r,202-r],'b',...
     [303-r,303-r],[-10+124 ,136+L],'b',[4*w+202-r+85,4*w+202-r+85],[238-27,238+L],'b',...  
     [238-10,238+L],[198-r,198-r],'b--',[238+L+w*4+2*r,238+2*L+w*4+2*r+10],[198-r,198-r],'b--',...
     [299-r,299-r],[-10+124 ,136+L],'b--',[4*w+198-r+85,4*w+198-r+85],[238-27,238+L],'b--',...
     [238-10,238+L],[206-r,206-r],'b--',[238+L+w*4+2*r,238+2*L+w*4+2*r+10],[206-r,206-r],'b--',...
     [307-r,307-r],[-10+124 ,136+L],'b--',[4*w+206-r+85,4*w+206-r+85],[238-27,238+L],'b--');
hold on;



hold on;

x0 = L + 238 ;
y0 = L + 137;
alpha = -0 : 0.1 : 90;
x = x0 + 2.5 * cosd(alpha);
y = y0 + 2.5 * sind(alpha);
plot(x, y, 'b');

x0 = L+4*w+2*r + 238;
y0 = L+ 137;
alpha = 90 : 0.1 : 180;
x = x0 + r * cosd(alpha);
y = y0 + r * sind(alpha);
plot(x, y, 'b');

x0 = L+4*w+2*r + 238;
y0 = L+4*w+2*r+ 137;
alpha = 180 : 0.1 : 270;
x = x0 + r * cosd(alpha);
y = y0 + r * sind(alpha);
plot(x, y, 'b');

x0 = L + 238;
y0 = L+4*w+2*r + 137;
alpha = 270 : 0.1 : 360;
x = x0 + r * cosd(alpha);
y = y0 + r * sind(alpha);
plot(x, y, 'b');

axis equal
% eml.extrinsic('imread');
% img = imread('bkg.png','png');
% imagesc([-20, 500],[-252, -552],img);
% imagesc([-20, 500],[50, -255],img);

% f = 0.85;
% plot([-20,500],[10,10],'b',[-20,500],[0,0],'b--',...
%     [-20,362.68],[-10,-10],'b',[402.68,500],[-10,-10],'b',...
%     [24.53,362.28],[-205*f,-10],'r',[36.27,400],[-210*f,0],'r--',...
%     [42.88,402.28],[-217.5*f,-10],'r');

% set(gcf,'unit','centimeters','position',[10 4 25 17]);
% plot([-20,500],[-255,-255],'y','linewidth',4)
%%%%%%%%%%%%%%%%%%%%%OC%%%%%%%%%%%%%%%%%%%%%%
% f = 0.93;
% plot([-20,500],[-290,-290],'b',[-20,500],[-300,-300],...
%     'b--',[-20,362.68],[-310,-310],'b',[402.68,500],[-310,-310],'b',...
%     [24.53,362.28],[-505*f,-310],'r',[36.27,400],[-510*f,-300],'r--',...
%     [42.88,402.28],[-517.5*f,-310],'r');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t=-50;
y=zeros(100,1);
% p = plot(t,y,'o',...
%     'EraseMode','background','MarkerSize',5, 'MarkerEdgeColor','red', 'MarkerFaceColor','red');
% p_s = plot(t,y,'o',...
%     'EraseMode','background','MarkerSize',5, 'MarkerEdgeColor','green', 'MarkerFaceColor','green');

% axis([0 L*2+4*w+2*r 0 L*2+4*w+2*r]);  
axis([238-10 238+2*L+w*4+2*r+10 120 290]);  
set(gca,'Xgrid','on');
lg= [];
% text(0, L*2+4*w+30, 'CBF', 'fontsize', 15, 'Color', 'red', 'fontweight', 'bold');
% text(400, L*2+4*w+30, 'Ave time(s):');
% text(400, L*2+4*w+60, 'Ave fuel(mL):');
% text(360, -100, 'Main ave. ener.(i):', 'Color', 'red', 'fontweight', 'bold');
% text(340, -120, 'Merging ave. ener.(i):', 'Color', 'red', 'fontweight', 'bold');
% text(330, -140, 'Main ave. $$\int \frac{1}{2}u_i^2(t)dt:$$', 'Color', 'red', 'fontweight', 'bold');
% text(310, -170, 'Merging ave. $$\int \frac{1}{2}u_i^2(t)dt:$$', 'Color', 'red', 'fontweight', 'bold');
% text(360, -190, 'Main ave. time(s):', 'Color', 'red', 'fontweight', 'bold');
% text(340, -210, 'Merging ave. time(s):', 'Color', 'red', 'fontweight', 'bold');
% text(100, L*2+4*w+40, 'Ave $$\int \frac{1}{2}u_i^2(t)dt:$$');
% text(510, 30, 'CNT:');

% lg.time_legend = text(580, L*2+4*w+30, '0');
% lg.energy_legend = text(580, L*2+4*w+60, '0');
% lg.Mae_lengend = text(500, -100, '0');
% lg.Mee_lengend = text(500, -120, '0');
% lg.Mau_lengend = text(500, -140, '0');
% lg.Meu_lengend = text(500, -170, '0');
% lg.Mat_lengend = text(500, -190, '0');
% lg.Met_lengend = text(500, -210, '0');
% lg.u2_legend = text(300, L*2+4*w+40, '0');
% lg.through_legend = text(580, 30, '0');

% frame2 = axes('position',[0.50 0.05 0.4 0.4]);
% axis(frame2);
% 
% plot([-10,L],[L+r,L+r],'b',[L+w*4+2*r,L*2+w*4+2*r+10],[L+r,L+r],'b',...
%     [-10,L],[L+4*w+r,L+4*w+r],'b',[L+w*4+r*2,L*2+w*4+r*2+10],[L+4*w+r,L+4*w+r],'b',...
%     [L+r,L+r],[-10,L],'b',[L+r,L+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b',...
%     [L+4*w+r,L+4*w+r],[-10,L],'b',[L+4*w+r,L+4*w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b',... 
%     [-10,L],[L+2*w+r,L+2*w+r],'b',[L+w*4+2*r,L*2+w*4+2*r+10],[L+2*w+r,L+2*w+r],'b',...
%     [L+2*w+r,L+2*w+r],[-10,L],'b',[L+2*w+r,L+2*w+r],[L+w*4+r*2,L*2+w*4+r*2+10],'b',...  
%     [-10,L],[L+w+r,L+w+r],'b--',[L+w*4+2*r,L*2+w*4+2*r+10],[L+w+r,L+w+r],'b--',...
%     [L+w+r,L+w+r],[-10,L],'b--',[L+w+r,L+w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b--',...
%     [-10,L],[L+3*w+r,L+3*w+r],'b--',[L+w*4+2*r,L*2+w*4+2*r+10],[L+3*w+r,L+3*w+r],'b--',...
%     [L+3*w+r,L+3*w+r],[-10,L],'b--',[L+3*w+r,L+3*w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b--');
% hold on;
% 
% x0 = L;
% y0 = L;
% alpha = 0 : 0.1 : 90;
% x = x0 + r * cosd(alpha);
% y = y0 + r * sind(alpha);
% plot(x, y, 'b');
% 
% x0 = L+4*w+2*r;
% y0 = L;
% alpha = 90 : 0.1 : 180;
% x = x0 + r * cosd(alpha);
% y = y0 + r * sind(alpha);
% plot(x, y, 'b');
% 
% x0 = L+4*w+2*r;
% y0 = L+4*w+2*r;
% alpha = 180 : 0.1 : 270;
% x = x0 + r * cosd(alpha);
% y = y0 + r * sind(alpha);
% plot(x, y, 'b');
% 
% x0 = L;
% y0 = L+4*w+2*r;
% alpha = 270 : 0.1 : 360;
% x = x0 + r * cosd(alpha);
% y = y0 + r * sind(alpha);
% plot(x, y, 'b');
% 
% axis equal
% 
% % p2 = plot(t,y,'o',...
% %     'EraseMode','background','MarkerSize',10, 'MarkerEdgeColor','red', 'MarkerFaceColor','red');
% % p_s2 = plot(t,y,'o',...
% %     'EraseMode','background','MarkerSize',10, 'MarkerEdgeColor','green', 'MarkerFaceColor','green');
% 
% axis([L-10 L+4*w+2*r+10 L-10 L+4*w+2*r+10]);  
% set(gca,'Xgrid','on');
