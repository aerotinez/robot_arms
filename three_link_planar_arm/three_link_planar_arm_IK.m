function q = three_link_planar_arm_IK(x_d, a)
    px = x_d(1,:);
    py = x_d(2,:);
    psi = (pi/180)*x_d(3,:);

    a1 = a(1,1);
    a2 = a(2,1);
    a3 = a(3,1);

    pwx = px - a3*cos(psi);
    pwy = py - a3*sin(psi);

    c2 = (pwx^2 + pwy^2 - a1^2 - a2^2)/(2*a1*a2);
    if psi > 0
        s2 = sqrt(1 - c2^2);
    else
        s2 = -sqrt(1 - c2^2);
    end

    s1 = ((a1 + a2*c2)*pwy - a2*s2*pwx)/(pwx^2 + pwy^2);
    c1 = ((a1 + a2*c2)*pwx + a2*s2*pwy)/(pwx^2 + pwy^2);

    q1 = atan2(s1, c1);
    q2 = atan2(s2, c2);
    q3 = psi - q1 - q2;

    q = [q1; q2; q3];
end