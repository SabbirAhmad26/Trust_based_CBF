function [A,UpperBound,LowerBound] = Carla_intersetion(trajs,width)


righta = find(trajs == 'Right A:');
% righta = [];
lefta = find(trajs == 'Left A:');
% lefta = [];
straighta = find(trajs == 'straight A:');
% straighta = [];

originb = find(trajs == 'Origin Lane B:');

rightb = find(trajs == 'Right B:');
% rightb = [];

leftb = find(trajs == 'Left B:');
% leftb = [];
straightb = find(trajs == "Straight B: ");
% straightb = [];

originc = find(trajs == 'Origin Lane C:');
% originc = [];

rightc = find(trajs == 'Right C:');
% rightc = [];
leftc = find(trajs == 'Left C:');
% leftc = [];
straightc =find (trajs == 'Straight C:');
% straightc = [];

origind = find(trajs == "Origin Lane D: ");
% origind= [];

rightd = find(trajs == "Right D: ");
% rightd =[];

leftd = find(trajs == 'Left D:');
% leftd= [];

straightd = find(trajs == "Straight D");
% straightd = [];
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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];

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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];

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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];

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
    if i <= 26
        A.y = [A.y ;205.3 + 0.01 * rand()];
    else
        A.y = [A.y;y_info];
    end
    A.x = [A.x;x_info];
    A.angle = [A.angle;angle_info];
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
    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
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

    A.x = [A.x;x_info];
    A.y = [A.y;y_info];
    A.angle = [A.angle;angle_info];
end

% ind = ~isnan(A.y);
% A.y=A.y(ind);
% 
% ind = ~isnan(A.x);
% A.x=A.x(ind);
% 
% ind = ~isnan(A.angle);
% A.angle=A.angle(ind);
UpperBound = [];
LowerBound = [];
% syms x1
% for k = 1:5:length(A.x)-1
%     if k == 200
%         stop = 1;
%     end
%     MidPoint = [(A.x(k+1)+ A.x(k))/2 (A.y(k+1)+ A.y(k))/2];
%     m = (A.y(k+1) - A.y(k))/(A.x(k+1) - A.x(k));
%     xx = 0.1*A.x(k):0.01:A.x(k+1);
%     yy = @(xx) -1/m*(xx - MidPoint(1,1)) + MidPoint(1,2);
%     eqn = (-1/m*(x1 - MidPoint(1,1)) + MidPoint(1,2) - MidPoint(1,2))^2 + (x1 - MidPoint(1,1))^2 == width^2;
%     solx1 = solve(eqn,x1);
% %     if length(UpperBound) < 117 %rightb-leftb
% %      if length(UpperBound) == 35 %rightc-leftc
% %     if length(UpperBound) <= 93
% %         m = m;
% %     else
% %         m = -m;
% %     end
%     if (m >= 0)
%             UpperBound = [UpperBound;double(solx1(1)),double(yy(solx1(1)))];
%             LowerBound = [LowerBound;double(solx1(2)),double(yy(solx1(2)))];
%     else
%             UpperBound = [UpperBound;double(solx1(2)),double(yy(solx1(2)))];
%             LowerBound = [LowerBound;double(solx1(1)),double(yy(solx1(1)))];
%     end
% 
% 
% end






