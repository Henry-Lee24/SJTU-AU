C=[0 0.3 1 2 2.7 3  3.3 4 5 5.7 6
   0 1.25 1.75 1.75 1 0 -1 -1.75 -1.75 -1.25 0 ]; %控制点坐标
V1=[0;1]; %起点 C1 出发速度方向 V1,
V2=[0;1]; %到终点 CN 速度方向 V2,
L=0.9/2; %L 为小车长度
NC=length(C); %原始控制点数
RC=[C(:,1)-V1*L,C(:,1),C(:,1)+V1*L,C(:,2:NC-1), ...
C(:,NC)-V2*L,C(:,NC),C(:,NC)+V2*L];
N=length(RC); %控制点扩充后的控制点数目
s=0:0.01:1; %归一化路程
f1s=(1-s).^3/6; %四个样条函数 f1(s)
f2s=(3*s.^3-6*s.^2+4)/6; %f2(s)
f3s=(-3*s.^3+3*s.^2+3*s+1)/6; %f3(s)
f4s=s.^3/6; %f4(s)
%绘制 B 样条曲线
figure(1); mycolor='mbc' ;
plot(RC(1,:),RC(2,:),'r*');
hold on;
for i=1:N-3
    P = RC(:,i)*f1s+RC(:,i+1)*f2s+RC(:,i+2)*f3s+RC(:,i+3)*f4s;
    plot(P(1,:),P(2,:),mycolor(mod(i,3)+1),LineWidth=1.25);
    title('过起点和终点的 B 样条曲线');
    xlim([-0.25 6.25]);
    ylim([-1 3]);
    hold on;
    grid on;
end
hold off;
%绘制曲率变化曲线
figure(2)
for i=1:N-3
    P = RC(:,i)*f1s+RC(:,i+1)*f2s+RC(:,i+2)*f3s+RC(:,i+3)*f4s;
    v_x = diff(P(1,:));
    a_x = diff(v_x);
    v_y = diff(P(2,:));
    a_y = diff(v_y);
    a_x(length(v_x)) = a_x(end); % 使数组维度一致
    a_y(length(v_y)) = a_y(end);
    K = abs(v_x.*a_y-a_x.*v_y) ./ (v_x.^2+v_y.^2).^(3/2);
    K(length(s)) = K(end);
    plot(P(1,:),K,mycolor(mod(i,3)+1),LineWidth=1.25);
    title('轨迹曲线的曲率变化');
    xlim([-0.25 6.25]);
    ylim([-0.1 1.2]);
    hold on;
    grid on;
end
hold off;
%绘制机器人运动轨迹
figure(3)
for i=1:N-3
    P = RC(:,i)*f1s+RC(:,i+1)*f2s+RC(:,i+2)*f3s+RC(:,i+3)*f4s;
    v_x = diff(P(1,:));
    v_y = diff(P(2,:));
    title('机器人运动轨迹');
    phi=v_y./v_x;
    for k = 1:20:length(s)-1
        car = polyshape([P(1,k)-0.9/2 P(1,k)-0.9/2 P(1,k)+0.9/2 P(1,k)+0.9/2], [P(2,k)-0.4/2 P(2,k)+0.4/2 P(2,k)+0.4/2 P(2,k)-0.4/2]);
        plot(rotate(car,atan(phi(k))*180/3.1415,[P(1,k),P(2,k)]), 'FaceColor',mycolor(mod(i,3)+1),'FaceAlpha',0.3);
    end
    hold on;
    grid on;
end
hold off;