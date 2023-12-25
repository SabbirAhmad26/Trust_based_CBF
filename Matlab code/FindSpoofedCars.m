function [SpoofedCarsList,OrderSpoofedCarList,SpoofedCAVsSuccedingcarlist,que] = FindSpoofedCars(que,table,order)

NumSpoofedCars = 0; % Initiliaze the variable for number of spoofedcars 
SpoofedCarsList = []; % Initiliaze the array containing the index of spoofed cars based on arrival order
SpoofedCAVsSuccedingcarlist = []; % Initiliaze the array containing the index of succeding cars based on arrival order
for k=1:1:length(order) % Search through whole queue to find the index of spoofed cars based on arrival order
    if que{order(k)}.MUSTleave == 1 % The condition for activation of mitigation
        NumSpoofedCars = NumSpoofedCars + 1; 
        SpoofedCarsList = [SpoofedCarsList,order(k)];
    end
end
% SpoofedCarsList = [SpoofedCarsList,length(order)+1];

OrderSpoofedCarList = []; % To store the actual order of the spoofed cars 
for j = 1:1:length(SpoofedCarsList)
    OrderSpoofedCarList = [OrderSpoofedCarList,find(order == SpoofedCarsList(j))];
end
OrderSpoofedCarList= [OrderSpoofedCarList,length(order)+1];% actual order of the spoofed cars + the index of 

for j = 1:1:length(SpoofedCarsList)
    index = -1;                     
    for jj = order(OrderSpoofedCarList(j)+1:OrderSpoofedCarList(j+1)-1) %Looking in only cars between two consecutive spoofed cars
        egocar = que{jj};
        [ip, ~, ~] = search_for_conflictCAVS(table, egocar);
        if sum(ip == SpoofedCarsList(j))
            index = jj;
            SpoofedCAVsSuccedingcarlist = [SpoofedCAVsSuccedingcarlist;index];
            que{jj}.overtake = 1;
            break
        end
    end
    if index == -1
        SpoofedCAVsSuccedingcarlist = [SpoofedCAVsSuccedingcarlist;index];
    end
   end
end