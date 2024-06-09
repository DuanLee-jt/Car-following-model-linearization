function [a] = a_LIDM_lambda2(v1,v2,lambda21,lambda22)
%a_LIDM_lambda2 p2=0或p3=0或p4=0的情况下的加速度函数
%  v1-本车速度，v2-前车速度，lambda21-参数，lambda22-参数
a1 = (0.4154+0.0419*lambda21)*v1 - 0.5167*v2 + 1.1641;
a2 = (-0.0506+0.0002*lambda22)*v1 + 0.0937;
c = v1+v1*(v2-v1)/(2*sqrt(1.5));
if c<0
    a = a2;
else
    a = a1;
end
end
