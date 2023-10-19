function [ lg, frame1] = draw_intersection_twolane()

L1 = 300;
L = L1;
w = 3.5;
r = 4;

figure(1)
frame1 = axes('position',[0.02 0.1 0.8 0.8]);
axis(frame1);
plot([-10,L1],[L1+r,L1+r],'b',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+r,L1+r],'b',...
    [-10,L1],[L1+4*w+r,L1+4*w+r],'b',[L1+w*4+r*2,L1*2+w*4+r*2+10],[L1+4*w+r,L1+4*w+r],'b',...
    [L1+r,L1+r],[-10,L1],'b',[L1+r,L1+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b',...
    [L1+4*w+r,L1+4*w+r],[-10,L1],'b',[L1+4*w+r,L1+4*w+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b',... 
    [-10,L1],[L1+2*w+r,L1+2*w+r],'b',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+2*w+r,L1+2*w+r],'b',...
    [L1+2*w+r,L1+2*w+r],[-10,L1],'b',[L1+2*w+r,L1+2*w+r],[L1+w*4+r*2,L1*2+w*4+r*2+10],'b',...  
    [-10,L1],[L1+w+r,L1+w+r],'b--',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+w+r,L1+w+r],'b--',...
    [L1+w+r,L1+w+r],[-10,L1],'b--',[L1+w+r,L1+w+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b--',...
    [-10,L1],[L1+3*w+r,L1+3*w+r],'b--',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+3*w+r,L1+3*w+r],'b--',...
    [L1+3*w+r,L1+3*w+r],[-10,L1],'b--',[L1+3*w+r,L1+3*w+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b--');
hold on;

x0 = L;
y0 = L;
alpha = 0 : 0.1 : 90;
x = x0 + r * cosd(alpha);
y = y0 + r * sind(alpha);
plot(x, y, 'b');

x0 = L+4*w+2*r;
y0 = L;
alpha = 90 : 0.1 : 180;
x = x0 + r * cosd(alpha);
y = y0 + r * sind(alpha);
plot(x, y, 'b');

x0 = L+4*w+2*r;
y0 = L+4*w+2*r;
alpha = 180 : 0.1 : 270;
x = x0 + r * cosd(alpha);
y = y0 + r * sind(alpha);
plot(x, y, 'b');

x0 = L;
y0 = L+4*w+2*r;
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

axis([0 L1*2+4*w+2*r 0 L1*2+4*w+2*r]);  
set(gca,'Xgrid','on');

text(0, L1*2+4*w+30, 'CBF', 'fontsize', 15, 'Color', 'red', 'fontweight', 'bold');
text(400, L1*2+4*w+30, 'Ave time(s):');
text(400, L1*2+4*w+60, 'Ave fuel(mL):');
% text(360, -100, 'Main ave. ener.(i):', 'Color', 'red', 'fontweight', 'bold');
% text(340, -120, 'Merging ave. ener.(i):', 'Color', 'red', 'fontweight', 'bold');
% text(330, -140, 'Main ave. $$\int \frac{1}{2}u_i^2(t)dt:$$', 'Color', 'red', 'fontweight', 'bold');
% text(310, -170, 'Merging ave. $$\int \frac{1}{2}u_i^2(t)dt:$$', 'Color', 'red', 'fontweight', 'bold');
% text(360, -190, 'Main ave. time(s):', 'Color', 'red', 'fontweight', 'bold');
% text(340, -210, 'Merging ave. time(s):', 'Color', 'red', 'fontweight', 'bold');
text(100, L1*2+4*w+40, 'Ave $$\int \frac{1}{2}u_i^2(t)dt:$$');
text(510, 30, 'CNT:');

lg.time_legend = text(580, L1*2+4*w+30, '0');
lg.energy_legend = text(580, L1*2+4*w+60, '0');
% lg.Mae_lengend = text(500, -100, '0');
% lg.Mee_lengend = text(500, -120, '0');
% lg.Mau_lengend = text(500, -140, '0');
% lg.Meu_lengend = text(500, -170, '0');
% lg.Mat_lengend = text(500, -190, '0');
% lg.Met_lengend = text(500, -210, '0');
lg.u2_legend = text(300, L1*2+4*w+40, '0');
lg.through_legend = text(580, 30, '0');

% frame2 = axes('position',[0.50 0.05 0.4 0.4]);
% axis(frame2);
% 
% plot([-10,L1],[L1+r,L1+r],'b',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+r,L1+r],'b',...
%     [-10,L1],[L1+4*w+r,L1+4*w+r],'b',[L1+w*4+r*2,L1*2+w*4+r*2+10],[L1+4*w+r,L1+4*w+r],'b',...
%     [L1+r,L1+r],[-10,L1],'b',[L1+r,L1+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b',...
%     [L1+4*w+r,L1+4*w+r],[-10,L1],'b',[L1+4*w+r,L1+4*w+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b',... 
%     [-10,L1],[L1+2*w+r,L1+2*w+r],'b',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+2*w+r,L1+2*w+r],'b',...
%     [L1+2*w+r,L1+2*w+r],[-10,L1],'b',[L1+2*w+r,L1+2*w+r],[L1+w*4+r*2,L1*2+w*4+r*2+10],'b',...  
%     [-10,L1],[L1+w+r,L1+w+r],'b--',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+w+r,L1+w+r],'b--',...
%     [L1+w+r,L1+w+r],[-10,L1],'b--',[L1+w+r,L1+w+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b--',...
%     [-10,L1],[L1+3*w+r,L1+3*w+r],'b--',[L1+w*4+2*r,L1*2+w*4+2*r+10],[L1+3*w+r,L1+3*w+r],'b--',...
%     [L1+3*w+r,L1+3*w+r],[-10,L1],'b--',[L1+3*w+r,L1+3*w+r],[L1+w*4+2*r,L1*2+w*4+2*r+10],'b--');
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
% axis([L1-10 L1+4*w+2*r+10 L1-10 L1+4*w+2*r+10]);  
% set(gca,'Xgrid','on');
