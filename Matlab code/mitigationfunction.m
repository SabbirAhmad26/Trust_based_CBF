function order = mitigationfunction(que1,table,order,CAV_e)
%MITIGATIONFUNCTION Summary of this function goes here
%   Detailed explanation goes here
  
[SpoofedCarsList,SpoofedCAVsSuccedingcarlist,que1] = FindSpoofedCars(que1,table,order);
for j =1:1:length(CAV_e.SpoofedCarsList)
    if CAV_e.SpoofedCAVsSuccedingcarlist(j)~=-1
        suc_ind = CAV_e.SpoofedCAVsSuccedingcarlist(j);
        s_ind = CAV_e.SpoofedCarsList(j);
        if que1{s_ind}.state(1) > que1{suc_ind}.state(1)
            SpoofedCarsList(j) = [];
            SpoofedCAVsSuccedingcarlist(j) = -1;
        end
    end
end

for j =1:1:length(SpoofedCarsList)
    if SpoofedCAVsSuccedingcarlist(j)~=-1 
        ExplicitConstraintIndices = ExpImpConstraintFinder_Mitigation(que1,SpoofedCAVsSuccedingcarlist(j),SpoofedCarsList,table);
        ind1 = find(order == SpoofedCarsList(j));
        ind2 = find(order == SpoofedCAVsSuccedingcarlist(j));
        set1 = order(ind1+1:ind2-1);
        Sj = ExplicitConstraintIndices';
        set2 = order(ind2 + 1:end);
        set2 = setxor(set2,intersect(set2,Sj));
        set2 = reshape(set2,1,length(set2));
        CAV_e.SpoofedCarsList(j) = SpoofedCarsList(j);
        CAV_e.SpoofedCAVsSuccedingcarlist(j) = SpoofedCAVsSuccedingcarlist(j);
    else
        CAV_e.SpoofedCarsList(j) = SpoofedCarsList(j);
        CAV_e.SpoofedCAVsSuccedingcarlist(j) = SpoofedCAVsSuccedingcarlist(j);
        ExplicitConstraintIndices = [];
        ind1 = find(order == SpoofedCarsList(j));
        % ind2 = length(order)+1;
        set1 = order(ind1+1:length(order));
        ind2 = [];
        set2 = [];
    end
    order = [order(1:ind1-1),set1,set2,order(ind2),order(ind1),ExplicitConstraintIndices'];
end
end

