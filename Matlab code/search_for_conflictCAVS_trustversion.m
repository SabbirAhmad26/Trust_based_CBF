function [ip, index, position] = search_for_conflictCAVS_trustversion(que,table, order, k, egocar,MultipleConstraints,trust_thereshold)
if MultipleConstraints == 1                                          
    trust_th = trust_thereshold.high;
else
    trust_th = 0;
end

index = [];
position = [];
for i = 1 : length(table)
    if (table{i}(1) == egocar.id(2))
        k = i;
        break;
    end
end


ip = [];
for j = k - 1 : -1 : 1
    if (table{j}(30) == table{k}(30))
        ip = [ip;table{j}(31)];
        if (que{ip(end)}.trust(1) >= trust_th)
            break;
        end
    end
end





for i = 1:1: length(egocar.id) -5 +1
    index{i} = [];
    position{i} = [];
end

for i = 5 : length(egocar.id)
    flag = 0;
    for j = k - 1 : -1 : 1
        if (table{j}(egocar.id(i)+1) > 0)
            index{i-5 +1} = [index{i-5 +1}, table{j}(31)];
            position{i-5 +1} = [position{i-5 +1}, table{j}(egocar.id(i)+1)];
            flag = 1;
            if(que{index{i-5 +1}(end)}.trust(1) >= trust_th)
                break;
            end
        end
    end
    if (flag == 0)
        index{i-5 +1} = [index{i-5 +1}, -1];
        position{i-5 +1} = [position{i-5 +1}, -1];
    end
end

 end