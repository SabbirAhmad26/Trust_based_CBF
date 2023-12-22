

function [lengths] = find_MPs(origin,exit,trajs,jindex)


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


%12 merging points
L.locations = [288.92, 204.819; 
            295.374,187.581;  
            312.595, 195.049; 
            305.7, 213.316;    
            295.257, 205.089; 
            295.331, 194.567; 
            305.829, 194.951;  
            305.755, 205.462; 
            300.536,205.263;  
            295.614, 199.744; 
            300.52, 194.9     
            305.634, 200.307];              

L.lengths = [];
lengths = [];
distance = 0;
flag1 = true;
flag2 = true;
flag3 = true;
flag4 = true;

% trajs = fileread('Position Values for ABCD.txt');
% trajs = string(trajs);
% trajs = splitlines(trajs);
% TF = (trajs == '');
% trajs(TF) = [];

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


if origin == 8
    origin = 2;
elseif origin == 7
    origin = 1;
elseif origin == 2
    origin = 4;
elseif origin == 1
    origin = 3;
elseif origin == 4
    origin = 6;
elseif origin == 3
    origin = 5;
elseif origin == 6
    origin = 8;
elseif origin == 5
    origin = 7;
end







if (origin == 2 && exit == 3)
    str = trajs(righta+1:lefta-1);
elseif (origin == 2 && exit == 1)
    str = trajs(straighta+1:originb-1);
elseif(origin ==1 && exit ==2)
    str = trajs(lefta+1:straighta-1);
elseif(origin == 4 && exit==3)
    str = trajs(rightb+1:leftb-1);
elseif (origin==4 && exit==1)
    str = trajs(straightb+1:originc-1);
elseif (origin == 3 && exit==2)
    str = trajs(leftb+1:straightb-1);
elseif(origin == 6 && exit==3)
    str = trajs(rightc+1:straightc-1);
elseif (origin == 6 && exit==1)
    str = trajs(straightc+1:origind-1);
elseif (origin == 5 && exit==2)
    str = trajs(leftc+1:rightc-1);
elseif (origin == 8 && exit==3)
    str = trajs(rightd+1:leftd-1);
elseif (origin == 8 && exit==1)
    str = trajs(straightd+1:end);
elseif (origin == 7 && exit==2)
    str = trajs(leftd+1:straightd-1);
end




temp = strsplit(str(jindex),' ');
temp_x = str2num(replace(temp(1), "x=",''));
temp_y = str2num(replace(temp(2), "y=",''));

if (origin == 2 && exit == 3)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (x_info <= L.locations(1,1) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    %lengths = [lengths;distance];
    L.lengths = [L.lengths;lengths];

elseif (origin == 2 && exit == 1)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (y_info <= L.locations(5,2) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (y_info <= L.locations(10,2) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        elseif (y_info <= L.locations(6,2) && flag3)
            lengths = [lengths; distance];
            flag3 = false;
        elseif (y_info <= L.locations(2,2) && flag4)
            lengths = [lengths; distance];
            flag4 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif(origin ==1 && exit ==2)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (y_info <= L.locations(9,2) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (x_info >= L.locations(12,1) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif(origin == 4 && exit==3)
   for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (y_info <= L.locations(2,2) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif (origin==4 && exit==1)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (x_info >= L.locations(6,1) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (x_info >= L.locations(11,1) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        elseif (x_info >= L.locations(7,1) && flag3)
            lengths = [lengths; distance];
            flag3 = false;
        elseif (x_info >= L.locations(3,1) && flag4)
            lengths = [lengths; distance];
            flag4 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif (origin == 3 && exit==2)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (y_info >= L.locations(9,2) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (y_info >= L.locations(10,2) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif(origin == 6 && exit==3)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (x_info >= L.locations(3,1) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    %lengths = [lengths;distance];
    L.lengths = [L.lengths;lengths];

elseif (origin == 6 && exit==1)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (y_info >= L.locations(7,2) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (y_info >= L.locations(12,2) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        elseif (y_info >= L.locations(8,2) && flag3)
            lengths = [lengths; distance];
            flag3 = false;
        elseif (y_info >= L.locations(4,2) && flag4)
            lengths = [lengths; distance];
            flag4 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif (origin == 5 && exit==2)
   for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (x_info <= L.locations(11,1) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (x_info <= L.locations(10,1) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif (origin == 8 && exit==3)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (y_info >= L.locations(4,2) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif (origin == 8 && exit==1)
    for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (x_info <= L.locations(8,1) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (x_info <= L.locations(9,1) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        elseif (x_info <= L.locations(5,1) && flag3)
            lengths = [lengths; distance];
            flag3 = false;
        elseif (x_info <= L.locations(1,1) && flag4)
            lengths = [lengths; distance];
            flag4 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    L.lengths = [L.lengths;lengths];

elseif (origin == 7 && exit==2)
   for i =jindex:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));
        distance = distance + sqrt((temp_x-x_info)^2 + (temp_y - y_info)^2);
        if (y_info <= L.locations(12,2) && flag1)
            lengths = [lengths; distance];
            flag1 = false;
        elseif (y_info <= L.locations(11,2) && flag2)
            lengths = [lengths; distance];
            flag2 = false;
        end
        temp_x = x_info;
        temp_y = y_info;
    end
    %lengths = [lengths;distance];
    L.lengths = [L.lengths;lengths];

end

%plot(A.x,A.y,'LineStyle','--')

% L = 290-238;
% r = 2.5;
% w = 4;
% hold on
% plot([238-10,238+L],[194-r,194-r],'r',[238+L+w*4+2*r,238+2*L+w*4+2*r+10],[194-r,194-r],'r',...
%     [238-10,238+L],[4*w+194-r,4*w+194-r],'r',[238+L+w*4+r*2,238+L*2+w*4+r*2+10],[4*w+194-r,4*w+194-r],'r',...
%     [295-r,295-r],[-10+124 ,124+L],'b',[L+r,L+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b',...
%     [L+4*w+r,L+4*w+r],[-10,L],'b',[L+4*w+r,L+4*w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b',... 
%     [-10,L],[L+2*w+r,L+2*w+r],'b',[L+w*4+2*r,L*2+w*4+2*r+10],[L+2*w+r,L+2*w+r],'b',...
%     [L+2*w+r,L+2*w+r],[-10,L],'b',[L+2*w+r,L+2*w+r],[L+w*4+r*2,L*2+w*4+r*2+10],'b',...  
%     [-10,L],[L+w+r,L+w+r],'b--',[L+w*4+2*r,L*2+w*4+2*r+10],[L+w+r,L+w+r],'b--',...
%     [L+w+r,L+w+r],[-10,L],'b--',[L+w+r,L+w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b--',...
%     [-10,L],[L+3*w+r,L+3*w+r],'b--',[L+w*4+2*r,L*2+w*4+2*r+10],[L+3*w+r,L+3*w+r],'b--',...
%     [L+3*w+r,L+3*w+r],[-10,L],'b--',[L+3*w+r,L+3*w+r],[L+w*4+2*r,L*2+w*4+2*r+10],'b--');
% hold on;

% plot([238.72 - 10 ,238.72 + L],[194 -  r ,194 - r],'r')
