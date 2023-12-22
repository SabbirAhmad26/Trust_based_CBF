
% origin=2,exit = "right",j=3; distance = 0;
% distancecurrent = 5;
function [xinfo,yinfo,angleinfo,j] = getXY(trajs,origin,exit,distancecurrent,distance,j,realpose,prerealpose)

    
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
    
    if (origin == 8 && exit == 3)
        str = trajs(righta+1:lefta-1);
    elseif (origin == 8 && exit == 1)
        str = trajs(straighta+1:originb-1);
    elseif(origin ==7 && exit ==2)
        str = trajs(lefta+1:straighta-1);
    elseif(origin == 2 && exit==3)
        str = trajs(rightb+1:leftb-1);
    elseif (origin==2 && exit==1)
        str = trajs(straightb+1:originc-1);
    elseif (origin == 1 && exit==2)
        str = trajs(leftb+1:straightb-1);
    elseif(origin == 4 && exit==3)
        str = trajs(rightc+1:straightc-1);
    elseif (origin == 4 && exit==1)
        str = trajs(straightc+1:origind-1);
    elseif (origin == 3 && exit==2)
        str = trajs(leftc+1:rightc-1);
    elseif (origin == 6 && exit==3)
        str = trajs(rightd+1:leftd-1);
    elseif (origin == 6 && exit==1)
        str = trajs(straightd+1:end);
    elseif (origin == 5 && exit==2)
        str = trajs(leftd+1:straightd-1);
    end
    
    temp = strsplit(str(j),' ');
    % x_info = temp(1);
    % y_info = temp(2);
    angle_info = temp(3);
    % tempx = str2num(replace(x_info, "x=",''));
    % tempy = str2num(replace(y_info, "y=",''));
    tempangle = str2num(replace(angle_info, "angle=",''));

    tempx = realpose(1);
    tempy = realpose(2);

    for i =j+1:1:length(str)
        temp = strsplit(str(i),' ');
        x_info = temp(1);
        y_info = temp(2);
        angle_info = temp(3);
        x_info = str2num(replace(x_info, "x=",''));
        y_info = str2num(replace(y_info, "y=",''));        
        angle_info = str2num(replace(angle_info, "angle=",''));
        if (distance + sqrt((x_info-tempx)^2 + (y_info-tempy)^2) > distancecurrent)
            j = i-1;
            break
        end
    end
    
    t = (distancecurrent-distance)/sqrt((x_info - tempx)^2 + (y_info-tempy)^2);
    xinfo = (1-t)*tempx + t*x_info;
    yinfo = (1-t)*tempy + t*y_info;
    % angleinfo = (1-t)*tempa    
    angle_rad = atan2((realpose(1) - prerealpose(1)),(realpose(2) - prerealpose(2)));
    angle_deg = angle_rad * 360/2/pi;
    angleinfo= rem((angle_deg + 360),360);