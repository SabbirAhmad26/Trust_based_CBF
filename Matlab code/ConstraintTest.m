function score = ConstraintTest(simindex,que,table,order,k,ego,trust_thereshold)
%CONSTRAINTTEST Summary of this function goes here
%   Detailed explanation goes here
MultipleConstraints = 0;
[ip, index, position] = search_for_conflictCAVS_trustversion(que,table, order, k, ego,MultipleConstraints,trust_thereshold);
thereshold = 0;
if simindex == 80
    stop = 1;
end
if  (ego.id(2) == 9)
    stop = 1;
end

ultimate_score_rear = 0;
score_lateral = zeros(1,length(ego.lateralconstraint));
%if numel(ego.rearendconstraint)
if numel(ip) && numel(ego.rearendconstraint)
    if ( min(ego.rearendconstraint) >= thereshold || ego.infeasiblity == 1 ) || que{ip}.scores(2) == 0 || que{ip}.trust(1) <= trust_thereshold.low 
        ultimate_score_rear = 1;
    end
else
    ultimate_score_rear = 1;
end
if numel(ego.lateralconstraint) && numel(index)
       for i = 1:1:length(ego.lateralconstraint)
        if (numel(ego.lateralconstraint{i})) && index{i}(1)~=-1
            if (ego.lateralconstraint{i}(end) >= thereshold || ego.infeasiblity == 1 ) || que{index{i}(1)}.scores(2) == 0 || que{index{i}(1)}.trust(1) <= trust_thereshold.low  
                score_lateral(i) = 1;
            end
       else
             score_lateral(i) = 1;
       end
        

    end
else
    score_lateral = 1;
end

ultimate_score_lateral = prod(score_lateral);

score = min(ultimate_score_lateral,ultimate_score_rear);

if score == 0
    stop = 1;
end
end

