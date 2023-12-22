function [ego,score,trust] = TrustCal(simindex,que,table,order,ego,order_k,k,CAV_e,trust_thereshold)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

lambda =0.95;
if sum(isnan(ego.scores)) == 4 || (ego.scores(1) == 1)
    score0 = Continuity(ego);
    if score0 > 0
        score1 = DynamicTest(ego);
        score2 = ConstraintTest(simindex,que,table,order,k,ego,trust_thereshold);
        

        score3 = VisionTest(que,ego,order_k);
        score = [score0;score1;score2;score3];
        c = 5;
%         w = [0.6 0.6 0.6 0.6;1000 50 1000 0.25];
        w = [0.6 0.6 0.6 0.6;1000 10 250 0.25];
        ego.reward = lambda*ego.reward + w(1,:)*score;
        ego.regret = lambda*ego.regret + w(2,:)*(1-score);

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


end

