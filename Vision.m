function indices = Vision(que,egocar,k)
%VISIONTEST Summary of this function goes here
%   Detailed explanation goes here
len = length(que);
R = 50;
indices = [];
EgoHeading = atan2(egocar.realpose(2)-egocar.prerealpose(2),egocar.realpose(1)-egocar.prerealpose(1));
for i=1:1: len
    if i == k
        continue
    else
        RelativeHeading = EgoHeading - atan2(-egocar.prerealpose(2)-0.01+que{i}.realpose(2),-egocar.prerealpose(1)-0.01+que{i}.realpose(1));
        if (egocar.realpose(1)-que{i}.realpose(1))^2+(egocar.realpose(2)-que{i}.realpose(2))^2 <= R^2
            if -pi/8 <= RelativeHeading &&   RelativeHeading <= pi/8
                indices = [indices;i];
            end
        end        
    end

end

end

