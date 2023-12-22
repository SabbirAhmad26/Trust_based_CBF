function L1 = get_L_1(t0, new, coe)
varphi = 1.8;
if(new(5) - coe(5)>= varphi)
    L1 = 400;
else
    syms t1
    s_new = 1/6*new(1)*t1^3 + 1/2*new(2)*t1^2 + new(3)*t1 + new(4);
    v_new = 1/2*new(1)*t1^2 + new(2)*t1 + new(3);
    s_coe = 1/6*coe(1)*t1^3 + 1/2*coe(2)*t1^2 + coe(3)*t1 + coe(4);
    S = solve(s_coe - s_new == varphi*v_new, t1,'ReturnConditions',true);
    t = vpa(S.t1);
    ta = [];
    for i = 1:length(t)
        if(imag(t(i))==0 && real(t(i)) >= t0 && real(t(i)) <= new(5))
            ta = t(i);           
        end       
    end
    if(numel(ta) == 0)
        L1 = 400;
        return;
    else
        L1 = 1/6*new(1)*ta^3 + 1/2*new(2)*ta^2 + new(3)*ta + new(4);
    end
end