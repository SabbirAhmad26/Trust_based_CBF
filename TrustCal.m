function [ego,score,trust] = TrustCal(simindex,que,table,ego,k,CAV_e,trust_thereshold)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% if  (ego.id(2) == 35) && simindex>= 705
%     stop = 1;
% end
lambda =0.95;
if sum(isnan(ego.scores)) == 4 || (ego.scores(1) == 1)
    score0 = Continuity(ego);
    if score0 > 0
        score1 = DynamicTest(ego);
        score2 = ConstraintTest(simindex,que,table,ego,trust_thereshold);

        score3 = VisionTest(que,ego,k);
        score = [score0;score1;score2;score3];
        c = 9;
%         w = [0.6 0.6 0.6 0.6;1000 50 1000 0.25];
        w = [0.6 0.6 0.6 0.6;1000 10 250 0.25];
        ego.reward = lambda*ego.reward + w(1,:)*score;
        ego.regret = lambda*ego.regret + w(2,:)*(1-score);
        if  (ego.id(2) == 11) && numel(intersect(score,[0,0,0,0]))
            stop = 1;
        end
        if w(2,1:3)*(1-score(1:3)) > 0
            stop = 1;
        end
        %trust = min(ego.trust(1) + b/(b+r+c),1);
        trust = ego.reward/(ego.reward+ego.regret+c);
    else
        score = [0,0,0,0];
        trust = 0;            
    end
else
    score = [0,0,0,0];
    trust = 0;    
    return
    
    
end



done = 1;

end

