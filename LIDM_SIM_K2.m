% LIDM模型跟驰仿真
% 仿真场景：400m环形道路，仿真时长3600s，步长0.1s，10辆车，初始等间隔分布
% 主要模拟将K2视为系数的情况
% 加速度计算调用函数a_LIDM_K2

clc
clf
clear

% 常量定义
circleLength = 400; % 圈长（m）
vehNum = 10; % 车辆数量
startLoc = 10; % 10号车的起始位置
startVel =  20; % 初始速度
t = 1; % 时间间隔（s）
time = 1800; % 仿真时间（s）
time_1 = 3600; % 暖机时间（s）

% 待确定的系数初始范围
k21 = -1:0.1:-0.2;
k22 = -0.06:0.01:0;
% 创建对应矩阵
x = zeros(time/t,vehNum,length(k21),length(k22));
v = zeros(time/t,vehNum,length(k21),length(k22));
acc = zeros(time/t,vehNum,length(k21),length(k22));
xDelta = zeros(time/t,vehNum,length(k21),length(k22));

% 初始化
xDelta(1,:,:,:) = circleLength/vehNum;
v(1,:,:,:) = startVel;

for ik21 = 1:length(k21)
    i21 = k21(ik21);
    for ik22 = 1:length(k22)
        i22 = k22(ik22);
        x(1,:,ik21,ik22) = flip(startLoc:circleLength/vehNum:circleLength); % 等间距分布
        for iVeh = 2:vehNum
            acc(1,iVeh,ik21,ik22) = a_LIDM_K2(v(1,iVeh,ik21,ik22),v(1,iVeh-1,ik21,ik22),xDelta(1,iVeh,ik21,ik22),i21,i22);
        end
        acc(1,1,ik21,ik22) = a_LIDM_K2(v(1,1,ik21,ik22),v(1,vehNum,ik21,ik22),xDelta(1,1,ik21,ik22),i21,i22);
    end
end

for ik21 = 1:length(k21)
    i21 = k21(ik21);

    for ik22 = 1:length(k22)
        i22 = k22(ik22);

        for iFrame = 2:time/t
            % 更新位置
            x(iFrame,:,ik21,ik22) = x(iFrame-1,:,ik21,ik22)+v(iFrame-1,:,ik21,ik22)*t+0.5*acc(iFrame-1,:,ik21,ik22)*t^2;
            x(iFrame,:,ik21,ik22) = mod(x(iFrame,:,ik21,ik22),circleLength);
            
            % 更新间距
            for iVeh = 2:vehNum
                xDelta(iFrame,iVeh,ik21,ik22) = mod(x(iFrame,iVeh-1,ik21,ik22)-x(iFrame,iVeh,ik21,ik22),circleLength);
            end
            xDelta(iFrame,1,ik21,ik22) = mod(x(iFrame,vehNum,ik21,ik22)-x(iFrame,1,ik21,ik22),circleLength);
            
            % 更新速度
            v(iFrame,:,ik21,ik22) = max(0,v(iFrame-1,:,ik21,ik22)+acc(iFrame-1,:,ik21,ik22)*t);
            
            % 更新加速度，判断是否到达扰动时间
            if iFrame >= time_1/t && iFrame <= (time_1+2)/t
                acc(iFrame,1,ik21,ik22) = -0.5;
                for iVeh = 2:vehNum
                    acc(iFrame,iVeh,ik21,ik22) = a_LIDM_K2(v(iFrame,iVeh,ik21,ik22),v(iFrame,iVeh-1,ik21,ik22),xDelta(iFrame,iVeh,ik21,ik22),i21,i22);
                end
            else
                for iVeh = 2:vehNum
                    acc(iFrame,iVeh,ik21,ik22) = a_LIDM_K2(v(iFrame,iVeh,ik21,ik22),v(iFrame,iVeh-1,ik21,ik22),xDelta(iFrame,iVeh,ik21,ik22),i21,i22);
                end
                acc(iFrame,1,ik21,ik22) = a_LIDM_K2(v(iFrame,1,ik21,ik22),v(iFrame,vehNum,ik21,ik22),xDelta(iFrame,1,ik21,ik22),i21,i22);
            end
           
        end
    end
end

%% 

% 设置保存路径 
savePath1 = 'E:\毕设\图\LIDM\K2\暖机\速度角度';
% savePath1 = 'E:\毕设\图\LIDM\K2\速度角度';

for ik21 = 1:length(k21)
    i21 = k21(ik21);
    for ik22 = 1:length(k22)
        i22 = k22(ik22);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = v(:,iVeh,ik21,ik22)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.7, 0.9, sprintf('$\\tilde{k}_{21}:%.1f, \\tilde{k}_{22}:%.2f$', i21, i22), ...
            'Units', 'normalized', ...
            'FontSize', 25, ...
            'FontName', 'Times New Roman', ...
            'Interpreter', 'latex');
        view(-65,50)
        %zlim([0,100])
        xlabel('车辆编号','fontname','宋体','FontWeight','bold')
        ylabel('时间/s','fontname','宋体','FontWeight','bold')
        zlabel('速度/(m/s)','fontname','宋体','FontWeight','bold')
        set(gca,'YDir','reverse');
        set(gcf,'unit','centimeters','position',[2,2,30,18]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        box off

        filename = sprintf('k21_%.1f_k22_%.2f.png', i21, i22);
        fullPath = fullfile(savePath1, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end



%% 

% 设置保存路径 
savePath2 = 'E:\毕设\图\LIDM\K2\暖机\间距角度';
% savePath2 = 'E:\毕设\图\LIDM\K2\间距角度';

for ik21 = 1:length(k21)
    i21 = k21(ik21);
    for ik22 = 1:length(k22)
        i22 = k22(ik22);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = xDelta(:,iVeh,ik21,ik22)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.7, 0.9, sprintf('$\\tilde{k}_{21}:%.1f , \\tilde{k}_{22}:%.2f$', i21, i22), ...
            'Units', 'normalized', ...
            'FontSize', 25, ...
            'FontName', 'Times New Roman', ...
            'Interpreter', 'latex');
        view(-65,50)
        %zlim([39,41])
        xlabel('车辆编号','fontname','宋体','FontWeight','bold')
        ylabel('时间/s','fontname','宋体','FontWeight','bold')
        zlabel('间距/m','fontname','宋体','FontWeight','bold')
        set(gca,'YDir','reverse');
        set(gcf,'unit','centimeters','position',[2,2,30,18]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        box off

        filename = sprintf('k21_%.1f_k22_%.2f.png', i21, i22);
        fullPath = fullfile(savePath2, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end