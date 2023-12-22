function rt = RandomInitAttacker(i,one)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

noise1 = 0.5*rand();
noise2 = 5*(rand() - 0.5);
x0 = one.state; 
rt = [x0(1)+noise1 , x0(2), noise2];


end

