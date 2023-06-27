%设置DH参数
the1 = 0; d1 = 0; a1 = 0; alp1 = 0;
the2 = 0; d2 = 0; a2 = 0; alp2 = -90*pi/180;
the3 = 0; d3 = 0; a3 = 150; alp3 = 0;
the4 = 0; d4 = 150; a4 = 0; alp4 = -90*pi/180;
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
mypuma.base = transl(-50, 0, 0);
mypuma.display();
mypuma.plot([0,0,0,0,0,0])
hold on;
mypuma.teach
%first line
q0 = mypuma.ikine(transl(100, 100,10),'tol',1);
q1 = mypuma.ikine(transl(100,50,200),'tol',1);
t = [0:0.025:2];
qf = mtraj(@tpoly, q0, q1, t);
T1 = mypuma.fkine(qf);
for i = 1:1:81
    plot3(T1(i).t(1),T1(i).t(2),T1(i).t(3),'b.','MarkerSize',0.5);
    hold on
end
mypuma.plot(qf);

%second line
q2 = mypuma.ikine(transl(100,-50,200),'tol',1);
qs = mtraj(@tpoly, q1, q2, t);
T2 = mypuma.fkine(qs);
for i = 1:1:81
    plot3(T2(i).t(1),T2(i).t(2),T2(i).t(3),'b.','MarkerSize',0.5);
    hold on
end
mypuma.plot(qs);

%third line
q3 = mypuma.ikine(transl(100,-100,10),'tol',1);
qt = mtraj(@tpoly, q2, q3, t);
T3 = mypuma.fkine(qt);
for i = 1:1:81
    plot3(T3(i).t(1),T3(i).t(2),T3(i).t(3),'b.','MarkerSize',0.5);
    hold on
end
mypuma.plot(qt);