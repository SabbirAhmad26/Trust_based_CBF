function car = update_table(car)
order = car.order;
size = length(order);
car.table = cell(1, size);
for j = 1 : length(order)
    temp = zeros(1, 15);
    %% 1st: index, 2-29: CP 1-28, 30: current lane
    % 31: order No, that is the index of the vehicle in the queue
    temp(1) = car.que1{order(j)}.id(2);
    for i = 5 : length(car.que1{order(j)}.id)
        temp(car.que1{order(j)}.id(i)+1) = i - 4;
    end
    temp(14) = car.que1{order(j)}.id(3);
    temp(15) = order(j);
    car.table{j} = temp;
end