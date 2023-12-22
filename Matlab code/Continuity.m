function score = Continuity(ego)
%CONTINUITY Summary of this function goes here
%   Detailed explanation goes here
v_max =  40;
u_max = 5.88;
ego.id(2)
%maximum_displacement = 0.5*u_max*0.1^2 + v_max*0.1;
maximum_displacement = 20;
if max(abs(ego.realpose - ego.prerealpose)) >= maximum_displacement
    score = 0;
else
        score = 1;
end
end


