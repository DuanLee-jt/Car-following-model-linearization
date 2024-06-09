function [a] = a_LIDM_K3(v1,v2,xDelta,k31,k32)
%A_OF_LIDM 计算LIDM的加速度
%输入：v1本车车速度，v2前车速度，Delta_x与前车的间距，k3是待确定的系数
% c是实现IDM中的max
a1 = 0.4154*v1 - 0.5167*v2 + k31*xDelta + 1.1641;
a2 = -0.0506*v1 + k32*xDelta + 0.0937;
c = v1+v1*(v2-v1)/(2*sqrt(1.5));
if c<0
    a = a2;
else
    a =a1;
end
end
