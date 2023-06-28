A = [0,0,1,0;0,0,0,1;0,74.873,-0.566,0;0,80.796,-0.236,0];
B = [0;0;16.975;7.087];
C = diag([1,1,1,1]);
D = [0;0;0;0];
e = eig(A);

states = {'theta' 'alpha' 'theta_dot' 'alpha_dot'};
inputs = {'V_m'}; outputs = {'theta'; 'alpha';'theta_dot'; 'alpha_dot'};
t_test = 0: 0.01: 3; %测试时间
%稳定性分析
sys_ss = ss(A, B, C, D, 'statename', states, 'inputname', inputs, 'outputname', outputs);
step(sys_ss, t_test);
%可控性分析
Qc = ctrb(A,B);
disp(rank(Qc));

%LQR控制器
%Q = diag([50,50,1,1]);
Q = diag([80,60,5,5]);
R=1;
K=lqr(A,B,Q,R);
%新模型
Ap = A - B*K;
Bp = B;Cp = C;Dp = D;
e1 = eig(Ap);

x0 = [0;-10;0;0]; %初始阶跃
u = zeros(size(t_test));

%对比测试
theta_q= [10,20,40,50,60,80,100,150];
alpha_q= [150,100 ,80, 60, 50, 30, 20,10];
for i=1:length(theta_q)
    %q1 = 50;
    q1 = theta_q(i);
    q2 = alpha_q(i);
    %q2 = 50;
    Q = [q1 0 0 0
    0 q2 0 0
    0 0 1 0
    0 0 0 1];
    [K,P,r] = lqr(A,B,Q,R);
    disp(K);
    Ap = A - B*K;
    Bp = B;Cp = C;Dp = D;
    sys_lqr2 = ss(Ap, Bp, Cp, Dp, 'statename', states, 'inputname', inputs, 'outputname', outputs);
    lsim(sys_lqr2, u, t_test, x0);
    
    hold on
    grid on
end
legend('1','2','3','4','5','6','7','8');
