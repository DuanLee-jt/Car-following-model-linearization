% LIDM模型跟驰仿真
% 仿真场景：400m环形道路，仿真时长3600s，步长0.1s，10辆车，初始等间隔分布
% 主要模拟P1=0的情况
% 加速度计算调用函数a_LIDM_lambda1

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
lambda11 = 2:1:2;
lambda12 = [1,5,10.100];
% 创建对应矩阵
x = zeros(time/t,vehNum,length(lambda11),length(lambda12));
v = zeros(time/t,vehNum,length(lambda11),length(lambda12));
acc = zeros(time/t,vehNum,length(lambda11),length(lambda12));
xDelta = zeros(time/t,vehNum,length(lambda11),length(lambda12));

% 初始化
xDelta(1,:,:,:) = circleLength/vehNum;
v(1,:,:,:) = startVel;

for ilambda11 = 1:length(lambda11)
    i11 = lambda11(ilambda11);
    for ilambda12 = 1:length(lambda12)
        i12 = lambda12(ilambda12);
        x(1,:,ilambda11,ilambda12) = flip(startLoc:circleLength/vehNum:circleLength); % 等间距分布
        for iVeh = 2:vehNum
            acc(1,iVeh,ilambda11,ilambda12) = a_LIDM_lambda1(v(1,iVeh,ilambda11,ilambda12),v(1,iVeh-1,ilambda11,ilambda12),i11,i12);
        end
        acc(1,1,ilambda11,ilambda12) = a_LIDM_lambda1(v(1,1,ilambda11,ilambda12),v(1,vehNum,ilambda11,ilambda12),i11,i12);
    end
end

for ilambda11 = 1:length(lambda11)
    i11 = lambda11(ilambda11);

    for ilambda12 = 1:length(lambda12)
        i12 = lambda12(ilambda12);

        for iFrame = 2:time/t
            % 更新位置
            x(iFrame,:,ilambda11,ilambda12) = x(iFrame-1,:,ilambda11,ilambda12)+v(iFrame-1,:,ilambda11,ilambda12)*t+0.5*acc(iFrame-1,:,ilambda11,ilambda12)*t^2;
            x(iFrame,:,ilambda11,ilambda12) = mod(x(iFrame,:,ilambda11,ilambda12),circleLength);
            
            % 更新间距
            for iVeh = 2:vehNum
                xDelta(iFrame,iVeh,ilambda11,ilambda12) = mod(x(iFrame,iVeh-1,ilambda11,ilambda12)-x(iFrame,iVeh,ilambda11,ilambda12),circleLength);
            end
            xDelta(iFrame,1,ilambda11,ilambda12) = mod(x(iFrame,vehNum,ilambda11,ilambda12)-x(iFrame,1,ilambda11,ilambda12),circleLength);
            
            % 更新速度
            v(iFrame,:,ilambda11,ilambda12) = max(0,v(iFrame-1,:,ilambda11,ilambda12)+acc(iFrame-1,:,ilambda11,ilambda12)*t);
            

            % 更新加速度，判断是否到达扰动时间
            if iFrame >= time_1/t && iFrame <= (time_1+2)/t
                disp(iFrame)
                acc(iFrame,1,ilambda11,ilambda12) = -0.5;
                for iVeh = 2:vehNum
                    acc(iFrame,iVeh,ilambda11,ilambda12) = a_LIDM_lambda1(v(iFrame,iVeh,ilambda11,ilambda12),v(iFrame,iVeh-1,ilambda11,ilambda12),i11,i12);
                end
            else
                for iVeh = 2:vehNum
                    acc(iFrame,iVeh,ilambda11,ilambda12) = a_LIDM_lambda1(v(iFrame,iVeh,ilambda11,ilambda12),v(iFrame,iVeh-1,ilambda11,ilambda12),i11,i12);
                end
                acc(iFrame,1,ilambda11,ilambda12) = a_LIDM_lambda1(v(iFrame,1,ilambda11,ilambda12),v(iFrame,vehNum,ilambda11,ilambda12),i11,i12);
            end
        end
    end
end

%% 

% 设置保存路径 
savePath1 = 'E:\毕设\图\LIDM\lambda\暖机\1';
% savePath1 = 'E:\毕设\图\LIDM\lambda\速度角度';

for ilambda11 = 1:length(lambda11)
    i11 = lambda11(ilambda11);
    for ilambda12 = 1:length(lambda12)
        i12 = lambda12(ilambda12);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = v(:,iVeh,ilambda11,ilambda12)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.8, 0.9, sprintf('$\\tilde{\\lambda}_{11}:%.1f, \\tilde{\\lambda}_{12}:%.1f$', i11, i12), ...
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

        filename = sprintf('λ1_%d_λ2_%d.png',i11,i12);
        fullPath = fullfile(savePath1, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end



%% 

% 设置保存路径 
savePath3 = 'E:\毕设\图\LIDM\lambda\暖机\间距角度';
% savePath3 = 'E:\毕设\图\LIDM\lambda\间距角度';

for ilambda11 = 1:length(lambda11)
    i11 = lambda11(ilambda11);
    for ilambda12 = 1:length(lambda12)
        i12 = lambda12(ilambda12);
        figure('Visible', 'off')
        for iVeh=1:vehNum
            px = ones(1,time/t)*iVeh;
            py = 0:t:time-t;
            pz = xDelta(:,iVeh,ilambda11,ilambda12)';
            plot3(px,py,pz)
            hold on
        end
        set(gca,'FontName','Times New Roman','FontSize',25);
        text(0.8, 0.9, sprintf('$\\tilde{\\lambda}_{11}:%.1f , \\tilde{\\lambda}_{12}:%.1f$', i11, i12), ...
            'Units', 'normalized', ...
            'FontSize', 25, ...
            'FontName', 'Times New Roman', ...
            'Interpreter', 'latex');
        view(-65,50)
        zlim([39,41])
        xlabel('车辆编号','fontname','宋体','FontWeight','bold')
        ylabel('时间/s','fontname','宋体','FontWeight','bold')
        zlabel('间距/m','fontname','宋体','FontWeight','bold')
        set(gca,'YDir','reverse');
        set(gcf,'unit','centimeters','position',[2,2,30,18]);
        set(gca,'LooseInset',get(gca,'TightInset'));
        box off

        filename = sprintf('λ1_%d_λ2_%d.png',i11,i12);
        fullPath = fullfile(savePath3, filename);
        print(gcf, '-dpng', '-r600', fullPath);

        close(gcf)
        hold off; % 重置 hold on 状态，防止影响后续图形的绘制
    end
end



