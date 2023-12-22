function score = DynamicTest(ego)
global dt
%DYNAMICTEST Summary of this function goes here
%   Detailed explanation goes here
Umin = -6;
Umax = 3;
u_thereshold = 0.5;
thereshold = 0.3;
MeasuredAcc = (ego.state(2) - ego.prestate(2))/dt;
Measuredpos = (ego.state(1) - ego.prestate(1));
Dynamicpos = 0.5 * MeasuredAcc * dt^2 + ego.prestate(2)*dt;
if (MeasuredAcc + u_thereshold)  >= Umin && (MeasuredAcc - u_thereshold) <= Umax 
    if Measuredpos <= Dynamicpos + thereshold &&  Dynamicpos - thereshold <= Measuredpos 
        score = 1;
    else
        score = 0;
    end
else
    score = 0;
end

if score == 0
    stop = 1;
end

end

