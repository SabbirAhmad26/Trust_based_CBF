function [c,ceq] = constraint(x)
c = x(2)-x(4)+1.8/400*x(2)*x(1);
ceq = [];