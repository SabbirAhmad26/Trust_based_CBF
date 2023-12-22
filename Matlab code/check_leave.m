
function [index,order,CAV_e] = check_leave(time, car, que,CAV_e)
global dt

len = length(que);
index = [];
order = car.order;

for i = 1:1:len
    if (que{i}.state(1) > que{i}.metric(4) + 75) 
        index = i;
        CAV_e.arrivalexit(que{index}.id(2),2) = time * dt;
        order =car.order;
        order(find(order == i)) = [];
        for ii = 1:1:length(order)
            if (order(ii) > i)
                order(ii) = order(ii) -1;
            end
        end
        break;
    end
end