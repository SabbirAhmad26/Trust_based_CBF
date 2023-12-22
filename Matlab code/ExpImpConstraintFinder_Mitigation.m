function [ExplicitConstraintIndices] = ExpImpConstraintFinder_Mitigation(que,indexinthelist, k,SpoofedCarsList,order,table)
ExplicitConstraintIndices = k; % k is the index of the succeeding car in arrival order

OrderSpoofedCarList = [];
for j = 1:1:length(SpoofedCarsList)
    OrderSpoofedCarList = [OrderSpoofedCarList,find(order == SpoofedCarsList(j))];
end
OrderSpoofedCarList= [OrderSpoofedCarList,length(order)+1];


for j = 1:1:  length(ExplicitConstraintIndices)
   for i = order(OrderSpoofedCarList(indexinthelist)+1:OrderSpoofedCarList(indexinthelist+1)-1) %Looking in only cars between two consecutive spoofed cars
        if i == ExplicitConstraintIndices(j) 
            continue
        end
        egocar = que{i}; 
        [ip, index, ~] = search_for_conflictCAVS_new(table, egocar); % Finding all the ips and indicies for egocars
        if numel(intersect(ip,ExplicitConstraintIndices)) ||  numel(intersect(index,ExplicitConstraintIndices)) % Check to see if  they are common with the other cars
            ExplicitConstraintIndices = [ExplicitConstraintIndices,i];
        end
    end
end
ExplicitConstraintIndices = ExplicitConstraintIndices(2:end);

end

