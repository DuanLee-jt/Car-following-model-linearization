% LIDM模型跟驰仿真
% 仿真场景：400m环形道路，仿真时长3600s，步长0.1s，10辆车，初始等间隔分布
% 主要模拟将K3视为系数的情况
% 加速度计算调用函数a_LIDM_K3

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
position_change = 10; 

% 待确定的系数初始范围
k31 = 0.0419:0.0419;
k32 = 0.0002:0.0002;
% 创建对应矩阵
x = zeros(time/t,vehNum,length(k31),length(k32));
v = zeros(time/t,vehNum,length(k31),length(k32));
acc = zeros(time/t,vehNum,length(k31),length(k32));
xDelta = zeros(time/t,vehNum,length(k31),length(k32));

% 初始化
xDelta(1,:,:,:) = circleLength/vehNum;
v(1,:,:,:) = startVel;

for ik31 = 1:length(k31)
    i31 = k31(ik31);
    for ik32 = 1:length(k32)
        i32 = k32(ik32);
        x(1,:,ik31,ik32) = flip(startLoc:circleLength/vehNum:circleLength); % 等间距分布
        for iVeh = 2:vehNum
            acc(1,iVeh,ik31,ik32) = a_LIDM_K3(v(1,iVeh,ik31,ik32),v(1,iVeh-1,ik31,ik32),xDelta(1,iVeh,ik31,ik32),i31,i32);
        end
        acc(1,1,ik31,ik32) = a_LIDM_K3(v(1,1,ik31,ik32),v(1,vehNum,ik31,ik32),xDelta(1,1,ik31,ik32),i31,i32);
    end
end

for ik31 = 1:length(k31)
    i31 = k31(ik31);

    for ik32 = 1:length(k32)
        i32 = k32(ik32);

        for iFrame = 2:time/t
            % 更新位置
            x(iFrame,:,ik31,ik32) = x(iFrame-1,:,ik31,ik32)+v(iFrame-1,:,ik31,ik32)*t+0.5*acc(iFrame-1,:,ik31,ik32)*t^2;
            x(iFrame,:,ik31,ik32) = mod(x(iFrame,:,ik31,ik32),circleLength);
            
            % 更新间距
            for iVeh = 2:vehNum
                xDelta(iFrame,iVeh,ik31,ik32) = mod(x(iFrame,iVeh-1,ik31,ik32)-x(iFrame,iVeh,ik31,ik32),circleLength);
            end
            xDelta(iFrame,1,ik31,ik32) = mod(x(iFrame,vehNum,ik31,ik32)-x(iFrame,1,ik31,ik32),circleLength);

            if iFrame == time_1/t
                % 施加扰动
                disp(x(iFrame,1,ik31,ik32))
                x(iFrame,1,ik31,ik32) = x(iFrame,1,ik31,ik32)-position_change;
                % 更新间距
                xDelta(iFrame,1,ik31,ik32) = xDelta(iFrame,1,ik31,ik32) + position_change;
                xDelta(iFrame,2,ik31,ik32) = xDelta(iFrame,2,ik31,ik32) - position_change;
            end
            
            % 更新速度
            v(iFrame,:,ik31,ik32) = max(0,v(iFrame-1,:,ik31,ik32)+acc(iFrame-1,:,ik31,ik32)*t);
            

            % 更新加速度，判断是否到达扰动时间
            for iVeh = 2:vehNum
                acc(iFrame,iVeh,ik31,ik32) = a_LIDM_K3(v(iFrame,iVeh,ik31,ik32),v(iFrame,iVeh-1,ik31,ik32),xDelta(iFrame,iVeh,ik31,ik32),i31,i32);
            end
            acc(iFrame,1,ik31,ik32) = a_LIDM_K3(v(iFrame,1,ik31,ik32),v(iFrame,vehNum,ik31,ik32),xDelta(iFrame,1,ik31,ik32),i31,i32);
        end
    end
end

%% 

% 设置保存路径 
savePath1 = 'E:\毕设\图\LIDM\k\暖机\速度角度';
% savePath1 = 'E:\毕设\图\LIDM\k\速度角度';

for ik31 = 1:length(k31)
    i31 = k31(ik31);
    for ik32 = 1:length(k32)
        i32 = k32(ik32);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = v(:,iVeh,ik31,ik32)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.65, 0.9, sprintf('$\\tilde{k}_{31}:%.2f, \\tilde{k}_{32}:%.4f$', i31, i32), ...
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

        filename = sprintf('k31_%.4f_k32_%.6f.png', i31, i32);
        fullPath = fullfile(savePath1, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end



%% 

% 设置保存路径 
savePath2 = 'E:\毕设\图\LIDM\k\暖机\间距角度';
% savePath2 = 'E:\毕设\图\LIDM\k\间距角度';

for ik31 = 1:length(k31)
    i31 = k31(ik31);
    for ik32 = 1:length(k32)
        i32 = k32(ik32);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = xDelta(:,iVeh,ik31,ik32)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.65, 0.9, sprintf('$\\tilde{k}_{31}:%.2f , \\tilde{k}_{32}:%.4f$', i31, i32), ...
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

        filename = sprintf('k31_%.4f_k32_%.6f.png', i31, i32);
        fullPath = fullfile(savePath2, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end