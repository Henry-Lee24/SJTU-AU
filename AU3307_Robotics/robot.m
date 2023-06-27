%设置DH参数
the1 = 0; d1 = 0; a1 = 0; alp1 = 0;
the2 = 0; d2 = 0; a2 = 0; alp2 = -90*pi/180;
the3 = 0; d3 = 20; a3 = 100; alp3 = 0;
the4 = 0; d4 = 100; a4 = 10; alp4 = -90*pi/180;
the5 = 0; d5 = 0; a5 = 0; alp5 = 90*pi/180;
the6 = 0; d6 = 0; a6= 0; alp6 = -90*pi/180;
%建立六个连杆
L1 = Link([the1, d1, a1, alp1,0], 'modified');
L2 = Link([the2, d2, a2, alp2,0], 'modified');
L3 = Link([the3, d3, a3, alp3,0], 'modified');
L4 = Link([the4, d4, a4, alp4,0], 'modified');
L5 = Link([the5, d5, a5, alp5,0], 'modified');
L6 = Link([the6, d6, a6, alp6,0], 'modified');
%生成puma
mypuma = SerialLink([L1,L2,L3,L4,L5,L6],'name', 'puma hw');
mypuma.display();
mypuma.plot([0,0,0,0,0,0])
mypuma.teach
%随机次数
N = 30000;
theta1 = (-pi/2+(pi/2+pi/2)*rand(N,1));%关节1角度[-pi/2, pi/2]
theta2 = (-pi/2+(pi/2+pi/2)*rand(N,1));%关节2角度[-pi/2, pi/2]
theta3 = (-pi/2+(pi/2+pi/2)*rand(N,1));%关节3角度[-pi/2, pi/2]
theta4 = (-pi/2+(pi/2+pi/2)*rand(N,1));%关节4角度[-pi/2, pi/2]
theta5 = (-pi/2+(pi/2+pi/2)*rand(N,1));%关节5角度[-pi/2, pi/2]
theta6 = (-pi/2+(pi/2+pi/2)*rand(N,1));%关节6角度[-pi/2, pi/2]
q = [theta1, theta2, theta3, theta4, theta5,theta6];
%末端位姿
tail = mypuma.fkine(q);

X=zeros(N,1);
Y=zeros(N,1);
Z=zeros(N,1);
for n=1:1:N
    X(n)=tail(n).t(1);
    Y(n)=tail(n).t(2);
    Z(n)=tail(n).t(3);
end
plot3(X,Y,Z,'b.','MarkerSize',0.5);%画出落点
hold on;




