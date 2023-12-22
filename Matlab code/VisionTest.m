function score = VisionTest(que,ego,k)
%VISIONTEST Summary of this function goes here
%   Detailed explanation goes here
len = length(que);
score = 0;
for i = 1:1:len
    if i == k
        continue
    end
    if find(que{i}.see == k)
        score = score + 1;
    end
end
score = sign(score);
end

