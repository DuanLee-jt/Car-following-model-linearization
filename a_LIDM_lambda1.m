function [a] = a_LIDM_lambda1(v1,v2,lambda11,lambda12)
%A_LIDM_LAMBDA1 p1=0的情况下的加速度函数
%  v1-本车速度，v2-前车速度，lambda1-参数
a1 = (0.4154+0.0419*lambda11)*v1 - 0.5167*v2 + 1.1641;
a2 = (-0.0506+0.0002*lambda12)*v1 + 0.0937;
c = v1+v1*(v2-v1)/(2*sqrt(1.5));
% 判断a2起没起作用
% c = 1;
if c<0
    a = a2;
else
    a = a1;
end
end

