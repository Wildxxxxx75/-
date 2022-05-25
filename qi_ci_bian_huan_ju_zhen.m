function qi_ci_bian_huan_ju_zhen(th,dz,da,a)
T=[cos(th) -cos(a)*sin(th) sin(a)*sin(th) da*cos(th);
    sin(th) cos(a)*cos(th) -sin(a)*cos(th) da*sin(th);
    0 sin(a) cos(a) dz;
    0 0 0 1]
end