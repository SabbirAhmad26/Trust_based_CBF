function dx = second_order_model(t,x)
global u noise1 noise2

dx = zeros(2,1);
dx(1) = x(2) + noise1;
dx(2) = u(1) + noise2;