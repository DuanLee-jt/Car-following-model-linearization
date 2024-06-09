function [a] = a_LIDM_K2(v1,v2,xDelta,k21,k22)
%A_OF_LIDM 计算LIDM的加速度
%输入：v1本车车速度，v2前车速度，Delta_x与前车的间距，k2是待确定的系数
% c是实现IDM中的max
a1 = (-0.1013-k21)*v1 + k21*v2 + 0.0419*xDelta + 1.1641;
a2 = (-0.0506-k22)*v1 + k22*v2 + 0.0002*xDelta + 0.0937;
c = v1+v1*(v2-v1)/(2*sqrt(1.5));
if c<0
    a = a2;
else
    a =a1;
end
end

