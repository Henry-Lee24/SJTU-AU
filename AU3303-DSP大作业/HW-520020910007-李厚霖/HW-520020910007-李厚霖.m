tiledlayout(5,6)
for k = 1:27
    a=num2str(k,'%04d');
    path = strcat('DS',a,'.csv');
    signal = csvread(path,16);
    signalX = signal(:,1);
    signalY = signal(:,2);
    choosen_signal=[];
    j = 1;
    k = 1;
    Fs = 51200;
    for i = 1500:length(signal)
       if(abs(signalY(i)) > 0.8)
           for j = 1:2048
             choosen_signal(j,:) = signal((i-200+j),:);
           end
           break
      end
    end
    choosen_signalX = choosen_signal(:,1);
    choosen_signalY = choosen_signal(:,2);
    ave = mean(choosen_signalY);
    %disp(ave);
    choosen_signalY = choosen_signalY - ave;
    choosen_fft = fft(choosen_signalY,2048);
    for s = 1:2000
        choosen_fft_new(s,:)= choosen_fft(s+24,:);
    end
    choosen_fft_new = fftshift(choosen_fft_new);
    m = [0:1:2047];
    nexttile
    %figure
    %plot(choosen_signalX,choosen_signalY);
    %plot(m,abs(choosen_fft));
    %title(a);
    %figure
    m = [0:1:1999];
    plot(m,abs(choosen_fft_new));
    title(a)
    %stft(choosen_signalY,Fs,'FFTLength',2048)
end
%滤波环节，以DS0027为例
%巴特沃斯滤波器
wp = 20000*2*pi/Fs;                %设置通带数字角频率
wst = 12500*2*pi/Fs;                %设置阻带数字角频率
Omegap = 2*Fs*tan(wp/2);
Omegast = 2*Fs*tan(wst/2);
Rp = 1; %通带最大衰减
As = 20; %阻带最小衰减

[N,OmegaC] = buttord(Omegap,Omegast,Rp,As,'s');
fprintf('巴特沃斯滤波器 N= %4d\n', N);
[z,p,k] = buttap(N);        %创建巴特沃斯低通滤波器原型
[Bap,Aap] = zp2tf(z,p,k);   %由零极点转换为传递函数的形式
[Bbs,Abs] = lp2hp(Bap,Aap,OmegaC);%低通原型变高通
[Bbz,Abz] = bilinear(Bbs,Abs,Fs);%用双线性变换法设计数字滤波器

figure
final_signal=filter(Bbz, Abz, choosen_signalY);
m = [0:1:2047];
plot((m-1024)/2048*51200,fftshift(abs(fft(choosen_signalY,2048))));%原始信号图像
xlabel('频率/kHz');
ylabel('幅值');
title('原始信号图像');
figure
plot((m-1024)/2048*51200,fftshift(abs(fft(final_signal,2048))));%滤波后的信号图像
xlabel('频率/kHz');
ylabel('幅值');
title('巴特沃斯滤波结果')


%椭圆滤波器
wp2 = 30000/Fs;                %设置通带数字角频率
wst2 = 15000/Fs;                %设置阻带数字角频率

Rp2 = 1; %通带最大衰减
As2 = 30; %阻带最小衰减

[N2, OmegaC2] = ellipord(wp2,wst2,Rp2,As2);%创建椭圆滤波器原型
fprintf('椭圆滤波器 N= %4d\n', N2);
[b,a] = ellip(N2,Rp2,As2, OmegaC2,'high');%得到高通滤波器

filter_hp_s = filter(b,a,choosen_signalY);%对原始信号进行滤波
figure
plot((m-1024)/2048*51200,fftshift(abs(fft(filter_hp_s,2048))));%滤波后的信号图像
xlabel('频率/kHz');
ylabel('幅值');
title('椭圆滤波结果')

