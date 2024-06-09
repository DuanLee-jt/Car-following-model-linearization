% LIDM模型跟驰仿真
% 仿真场景：400m环形道路，仿真时长3600s，步长0.1s，10辆车，初始等间隔分布
% 主要模拟P2、P3、P4=0的情况
% 加速度计算调用函数a_LIDM_lambda2

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
time_1 = 4000; % 暖机时间（s）

% 待确定的系数初始范围
lambda21 = 1:2:10;
lambda22 = [1,5,10,250];
% 创建对应矩阵
x = zeros(time/t,vehNum,length(lambda21),length(lambda22));
v = zeros(time/t,vehNum,length(lambda21),length(lambda22));
acc = zeros(time/t,vehNum,length(lambda21),length(lambda22));
xDelta = zeros(time/t,vehNum,length(lambda21),length(lambda22));

% 初始化
xDelta(1,:,:,:) = circleLength/vehNum;
v(1,:,:,:) = startVel;

for ilambda21 = 1:length(lambda21)
    i21 = lambda21(ilambda21);
    for ilambda22 = 1:length(lambda22)
        i22 = lambda22(ilambda22);
        x(1,:,ilambda21,ilambda22) = flip(startLoc:circleLength/vehNum:circleLength); % 等间距分布
        for iVeh = 2:vehNum
            acc(1,iVeh,ilambda21,ilambda22) = a_LIDM_lambda2(v(1,iVeh,ilambda21,ilambda22),v(1,iVeh-1,ilambda21,ilambda22),i21,i22);
        end
        acc(1,1,ilambda21,ilambda22) = a_LIDM_lambda2(v(1,1,ilambda21,ilambda22),v(1,vehNum,ilambda21,ilambda22),i21,i22);
    end
end

for ilambda21 = 1:length(lambda21)
    i21 = lambda21(ilambda21);

    for ilambda22 = 1:length(lambda22)
        i22 = lambda22(ilambda22);

        for iFrame = 2:time/t
            % 更新位置
            x(iFrame,:,ilambda21,ilambda22) = x(iFrame-1,:,ilambda21,ilambda22)+v(iFrame-1,:,ilambda21,ilambda22)*t+0.5*acc(iFrame-1,:,ilambda21,ilambda22)*t^2;
            x(iFrame,:,ilambda21,ilambda22) = mod(x(iFrame,:,ilambda21,ilambda22),circleLength);
            
            % 更新间距
            for iVeh = 2:vehNum
                xDelta(iFrame,iVeh,ilambda21,ilambda22) = mod(x(iFrame,iVeh-1,ilambda21,ilambda22)-x(iFrame,iVeh,ilambda21,ilambda22),circleLength);
            end
            xDelta(iFrame,1,ilambda21,ilambda22) = mod(x(iFrame,vehNum,ilambda21,ilambda22)-x(iFrame,1,ilambda21,ilambda22),circleLength);
            
            % 更新速度
            v(iFrame,:,ilambda21,ilambda22) = max(0,v(iFrame-1,:,ilambda21,ilambda22)+acc(iFrame-1,:,ilambda21,ilambda22)*t);
            

            % 更新加速度，判断是否到达扰动时间
            if iFrame >= time_1/t && iFrame <= (time_1+2)/t
                disp(iFrame)
                acc(iFrame,1,ilambda21,ilambda22) = -0.5;
                for iVeh = 2:vehNum
                    acc(iFrame,iVeh,ilambda21,ilambda22) = a_LIDM_lambda1(v(iFrame,iVeh,ilambda21,ilambda22),v(iFrame,iVeh-1,ilambda21,ilambda22),i21,i22);
                end
            else
                for iVeh = 2:vehNum
                    acc(iFrame,iVeh,ilambda21,ilambda22) = a_LIDM_lambda1(v(iFrame,iVeh,ilambda21,ilambda22),v(iFrame,iVeh-1,ilambda21,ilambda22),i21,i22);
                end
                acc(iFrame,1,ilambda21,ilambda22) = a_LIDM_lambda1(v(iFrame,1,ilambda21,ilambda22),v(iFrame,vehNum,ilambda21,ilambda22),i21,i22);
            end
        end
    end
end


% 设置保存路径 
savePath1 = 'E:\毕设\图\LIDM\lambda2\暖机\速度角度';
% savePath1 = 'E:\毕设\图\LIDM\lambda2\速度角度';

for ilambda21 = 1:length(lambda21)
    i21 = lambda21(ilambda21);
    for ilambda22 = 1:length(lambda22)
        i22 = lambda22(ilambda22);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = v(:,iVeh,ilambda21,ilambda22)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.8, 0.9, sprintf('$\\tilde{\\lambda}_{21}:%d, \\tilde{\\lambda}_{22}:%d$', i21, i22), ...
            'Units', 'normalized', ...
            'FontSize', 25, ...
            'FontName', 'Times New Roman', ...
            'Interpreter', 'latex');
        view(-65,50)
        zlim([0,50])
        xlabel('车辆编号','fontname','宋体','FontWeight','bold')
        ylabel('时间/s','fontname','宋体','FontWeight','bold')
        zlabel('速度/(m/s)','fontname','宋体','FontWeight','bold')
        set(gca,'YDir','reverse');
        set(gcf,'unit','centimeters','position',[2,2,30,18]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        box off

        filename = sprintf('λ21_%d_λ22_%d.png',i21,i22);
        fullPath = fullfile(savePath1, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end

% 设置保存路径 
savePath2 = 'E:\毕设\图\LIDM\lambda2\暖机\加速度角度';
% savePath2 = 'E:\毕设\图\LIDM\lambda2\加速度角度';

for ilambda21 = 1:length(lambda21)
    i21 = lambda21(ilambda21);
    for ilambda22 = 1:length(lambda22)
        i22 = lambda22(ilambda22);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = acc(:,iVeh,ilambda21,ilambda22)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.8, 0.9, sprintf('$\\tilde{\\lambda}_{21}:%d, \\tilde{\\lambda}_{22}:%d$', i21, i22), ...
            'Units', 'normalized', ...
            'FontSize', 25, ...
            'FontName', 'Times New Roman', ...
            'Interpreter', 'latex');
        view(-65,50)
        zlim([-10,10])
        xlabel('车辆编号','fontname','宋体','FontWeight','bold')
        ylabel('时间/s','fontname','宋体','FontWeight','bold')
        zlabel('加速度/(m/s^2)','fontname','宋体','FontWeight','bold')
        set(gca,'YDir','reverse');
        set(gcf,'unit','centimeters','position',[2,2,30,18]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        box off

        filename = sprintf('λ21_%d_λ22_%d.png',i21,i22);
        fullPath = fullfile(savePath2, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end

% 设置保存路径 
savePath3 = 'E:\毕设\图\LIDM\lambda2\暖机\间距角度';
% savePath1 = 'E:\毕设\图\LIDM\lambda\间距角度';

for ilambda21 = 1:length(lambda21)
    i21 = lambda21(ilambda21);
    for ilambda22 = 1:length(lambda22)
        i22 = lambda22(ilambda22);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = xDelta(:,iVeh,ilambda21,ilambda22)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.8, 0.9, sprintf('$\\tilde{\\lambda}_{21}:%d, \\tilde{\\lambda}_{22}:%d$', i21, i22), ...
            'Units', 'normalized', ...
            'FontSize', 25, ...
            'FontName', 'Times New Roman', ...
            'Interpreter', 'latex');
        view(-65,50)
        zlim([0,80])
        xlabel('车辆编号','fontname','宋体','FontWeight','bold')
        ylabel('时间/s','fontname','宋体','FontWeight','bold')
        zlabel('速度/(m/s)','fontname','宋体','FontWeight','bold')
        set(gca,'YDir','reverse');
        set(gcf,'unit','centimeters','position',[2,2,30,18]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        box off

        filename = sprintf('λ21_%d_λ22_%d.png',i21,i22);
        fullPath = fullfile(savePath3, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end

