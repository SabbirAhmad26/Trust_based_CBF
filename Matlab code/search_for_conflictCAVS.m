function [ip, index, position] = search_for_conflictCAVS(table, egocar)

index = [];
position = [];
for i = 1 : length(table)
    if (table{i}(1) == egocar.id(2))
        k = i;
        break;
    end
end

ip = -1;
for j = k - 1 : -1 : 1
    if (table{j}(30) == table{k}(30))
        ip = table{j}(31);
        break;
    end
end

for i = 5 : length(egocar.id)
    flag = 0;
    for j = k - 1 : -1 : 1
        if (table{j}(egocar.id(i)+1) > 0)
            index = [index, table{j}(31)];
            position = [position, table{j}(egocar.id(i)+1)];
            flag = 1;
            break;
        end
    end
    if (flag == 0)
        index = [index, -1];
        position = [position, -1];
    end
end