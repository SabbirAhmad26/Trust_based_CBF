function result = OCT1(t0,v0,L)

warning('off')
%L = 400;
global beta
%beta = 2*(0.4*10)^2/12;

syms a b c d tm
eqns = [0.5*a*t0^2 + b*t0 + c == v0, ...
1/6*a*t0^3 + 0.5*b*t0^2 + c*t0 + d == 0, ...
a*tm + b == 0, ...
1/6*a*tm^3 + 0.5*b*tm^2 + c*tm + d == L, ...
beta - 0.5*b^2 + a*c == 0];
%beta + 0.5*a^2*tm^2 + a*b*tm + a*c == 0];
S = solve(eqns,[a b c d tm],'ReturnConditions',true);
ai = vpa(S.a); bi = vpa(S.b); ci = vpa(S.c); di = vpa(S.d); tmi = vpa(S.tm);
result = [];
for i = 1:length(S.tm)
    if(imag(vpa(S.tm(i)))==0 && real(vpa(S.tm(i))) > t0+ 0 && real(vpa(S.tm(i))) <= t0 + 100)
        result = [ai(i),bi(i),ci(i),di(i),tmi(i),1/2*ai(i)*tmi(i)^2+bi(i)*tmi(i)+ci(i)];
    end       
end




