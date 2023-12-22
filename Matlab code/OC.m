function rt = OC(i, one, que, ip)

violation = one.violation;
x0 = one.state(1);
v0 = one.state(2);
t = 0.1*i;
vMax = 15;
uMax = 3;
deltaSafetyDistance = 10;
L = 300;

parameter = double(one.ocpar);

if (x0 > L || (violation ~= 4 && t > parameter(6)))
    v = vMax;
    u = 0;
    x = x0 + vMax * 0.1;
    rt = [x, v, u];
    return;
end

if (violation == 4 && t > parameter(12))
    v = vMax;
    u = 0;
    x = x0 + vMax * 0.1;
    rt = [x, v, u];
    return;
end    

if (violation == 0 || (violation == 1 && t <= parameter(5))  || (violation == 2 && t <= parameter(5)))
    ak = parameter(1);
    bk = parameter(2);
    ck = parameter(3);
    dk = parameter(4);
elseif (violation == 1)
    u = 0;
    v = vMax;
    x = x0 + vMax*0.1;
    rt = [x, v, u];
    return;
elseif (violation == 2)
    u = uMax;
    v = v0 + u*0.1;
    x = x0 + v0*0.1 + 0.5*u*0.1^2;
    rt = [x, v, u];
    return;
elseif (violation == 3)
    if (t < parameter(5))
        ak = parameter(1);
        bk = parameter(2);
        ck = parameter(3);
        dk = parameter(4);
    else
        u = que{ip}.state(3);
        v = que{ip}.state(2);
        x = que{ip}.state(1) - deltaSafetyDistance;
        rt = [x, v, u];
        return;
    end
elseif (violation == 4)
    if (t <= parameter(5))
        ak = parameter(1);
        bk = parameter(2);
        ck = parameter(3);
        dk = parameter(4);
    elseif (t > parameter(5) && t < parameter(11))
        u = que{ip}.state(3);
        v = que{ip}.state(2);
        x = que{ip}.state(1) - deltaSafetyDistance;
        rt = [x, v, u];
        return;
    elseif (t <= parameter(12))
        ak = parameter(7);
        bk = parameter(8);
        ck = parameter(9);
        dk = parameter(10);
    end
end

v = 0.5*ak*t^2 + bk*t + ck;
u = ak*t + bk;
x = 1/6*ak*t^3 + 0.5*bk*t^2 + ck*t + dk;

rt = [x, v, u];

end