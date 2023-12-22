function z = endpoint(y,x)
m = (y(2) - x(2))/(y(1)-x(1)) ;
c = y(2) - m*y(1);
t = 13;
z = (1-t)*x+t*y;
