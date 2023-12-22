function [outputArg1,outputArg2] = EvidenceCheck(que,ego)
%EVIDENCECHECK Summary of this function goes here
%   Detailed explanation goes here

score1 = DynamicTest(ego);
score2 = ConstraintTest(ego);
score3 = VisionTest(car,ego);

end

