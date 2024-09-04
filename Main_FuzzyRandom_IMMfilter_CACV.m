% main function  
% compare{fFuzzy_IMMfilter,fClassic_IMMfilter}
% two models (CV and CA)
% 3 dimension example

%% 仿真参数设定
Nre=50;   % 循环次数
radar=1; % 1-fire control 2-surveillance
if radar==1
    T=0.2; %  采样间隔 2 0.2
    Tsim=40; % 仿真时间 160 40
elseif radar==2
    T=2; %  采样间隔 2 0.2
    Tsim=160; % 仿真时间 160 40
end
Ns=Tsim/T;% 仿真总采样点数
MqAve=zeros(1,2,Ns); % 平均模型概率
MqFuzzyAve=zeros(1,2,Ns); % 平均模型概率
eXgAve=zeros(9,Ns);  % 平均估计误差
eXgFuzzyAve=zeros(9,Ns); % 平均估计误差-fuzzy
eZAve=zeros(3,Ns);       % 平均测量误差


%% 过程噪声定义
q=3*[1;1;1]; % [1;1;0.5] 过程噪声标准偏差,      米        实际值
%q=[0;0;0];
qm=3;  % 1, 3     % 滤波器设置的过程噪声标准偏差 米  模型值
qmF=3;% 1, 3     % 滤波器设置的过程噪声标准偏差 米  模型值

%% 测量噪声定义
if radar==1
    rbeitak=0.1*pi/180;     % 0.9 传感器方位角测量噪声标准偏差 角度 实际值
    rebuxinonk=0.1*pi/180;  % 0.9 传感器高低角测量噪声标准偏差 角度 实际值
    rDk=10;                % 100 传感器距离测量噪声标准偏差   米   实际值

    rmbeitak=0.1*pi/180;         % 0.9 1.35 1.8 传感器方位角测量噪声标准偏差 模型值
    rmebuxinonk=0.1*pi/180;      % 0.9 1.35 1.8 传感器高低角测量噪声标准偏差 模型值
    rmDk=10;                    % 100 150 200 传感器距离测量噪声标准偏差   模型值
elseif radar==2
    rbeitak=0.9*pi/180;     % 0.9 传感器方位角测量噪声标准偏差 角度 实际值
    rebuxinonk=0.9*pi/180;  % 0.9 传感器高低角测量噪声标准偏差 角度 实际值
    rDk=100;                % 100 传感器距离测量噪声标准偏差   米   实际值

    rmbeitak=0.9*pi/180;         % 0.9 1.35 1.8 传感器方位角测量噪声标准偏差 模型值
    rmebuxinonk=0.9*pi/180;      % 0.9 1.35 1.8 传感器高低角测量噪声标准偏差 模型值
    rmDk=100;                    % 100 150 200 传感器距离测量噪声标准偏差   模型值
end

%% 航迹真值/测量值 变量定义
X=zeros(9,Ns); % 直角坐标 航迹状态真值 序列
x=zeros(3,Ns); % 直角坐标 航迹坐标真值 序列
Z=zeros(3,Ns); % 直角坐标 测量值 序列
R=zeros(3,3,Ns); % 直角坐标 测量误差协方差 序列

%% 航迹真值初始值/参数/变量 定义
if radar==1
    x0=[12000;8000;1000];
elseif radar==2
	x0=[120000;80000;20000];  
end
v0=[-100;-100;0];
xk=zeros(3,1);
vk=zeros(3,1);
a=zeros(3,Ns); % 加速度序列 定义
for n=1:Nre  % 运行次数
if radar==1
    jump=80; % +randi(50);
    for k=1:jump % 80
        a(:,k)=[0;0;0]; 
    end
    for k=jump+1:jump+51 %  80：130
    %     a(:,k)=[-20;-10;0];
        a(:,k)=[-30;-50;0];
    end
    for k=jump+52:Ns %  131
        a(:,k)=[0;0;0];
    end
elseif radar==2
    for k=1:30 % 
        a(:,k)=[0;0;0]; 
    end
    for k=31:40 
    %     a(:,k)=[-20;-10;0];
        a(:,k)=[-30;-50;0];
    end
    for k=41:Ns % 
        a(:,k)=[0;0;0];
    end    
end

%for n=1:Nre  % 运行次数
%% 测量生成与转换
for k=1:Ns % 测量生成与转换
    if k==1
        xk=x0;
        vk=v0;
        x(:,k)=xk; % 保留航迹坐标真值序列
    else
        xk_1=xk;       % 航迹坐标真值   迭代
        vk_1=vk;       % 航迹速度真值   迭代
        ak_1=a(:,k-1); % 航迹加速度真值 迭代
        [xk,vk]=ftrackgenerator(xk_1,vk_1,ak_1,T,q);  % 调用 ftrackgenerator 生成航路
        X(:,k)=[xk(1);vk(1);ak_1(1);xk(2);vk(2);ak_1(2);xk(3);vk(3);ak_1(3)]; % 保留航迹坐标真值序列
        x(:,k)=xk; % 保留航迹坐标真值序列
    end
       
    [beitak,ebuxinonk,Dk]=fzhiqiuCoordinateTransformer(xk);  % 直/球坐标变换
     % beitak     方位角坐标
     % ebuxinonk  高低角坐标
     % Dk         距离坐标
    
    [zbeitak,zebuxinonk,zDk]=fsensor(beitak,ebuxinonk,Dk,rbeitak,rebuxinonk,rDk);   % 生成测量－球坐标
     % zbeitak     方位角测量
     % zebuxinonk  高低角测量
     % zDk         距离测量
     % rbeitak    传感器方位角测量噪声标准偏差
     % rebuxinonk 传感器高低角测量噪声标准偏差
     % rDk        传感器距离测量噪声标准偏差
    
    Zk=fqiuzhiCoordinateTransformer(zbeitak,zebuxinonk,zDk);  % 球/直坐标变换，
     % Zk          直角坐标向量
    Z(:,k)=Zk;
    
    % 传感器直角坐标测量噪声方差阵 球/直坐标变换
    Rk=fqiuzhiCeliangwuchafangchaTransformer(zbeitak,zebuxinonk,zDk,rmbeitak,rmebuxinonk,rmDk);
     % rmbeitak    传感器方位角测量噪声标准偏差 模型值
     % rmebuxinonk 传感器高低角测量噪声标准偏差 模型值
     % rmDk        传感器距离测量噪声标准偏差   模型值
     % Rk          传感器直角坐标测量噪声方差阵 模型值
    R(:,:,k)=Rk;
end % for k=1:Ns % 测量生成与转换

%% IMM filter
Xg=zeros(9,Ns); % 直角坐标 加权平均状态估计值 序列
XgFuzzy=zeros(9,Ns); % 直角坐标 加权平均状态估计值 序列 FuzzyIMM filter
eXg=zeros(9,Ns); % 误差值
eXgFuzzy=zeros(9,Ns); 
eZ=zeros(3,Ns);
Xgq=zeros(9,2,Ns); % 直角坐标 CV/CA状态估计值 序列
XgqFuzzy=zeros(9,2,Ns); % 直角坐标 CV/CA状态估计值 序列
Pq=zeros(9,9,2,Ns); 
PqFuzzy=zeros(9,9,2,Ns); 
Mq=zeros(1,2,Ns); % 模型概率 序列
MqFuzzy=zeros(1,2,Ns); % 模型概率 序列
L=zeros(1,2,Ns); % 模型似然
Lfuzzy=zeros(1,2,Ns); % 模型似然
S=zeros(3,3,2,Ns); % 新息协方差
vv=zeros(3,2,Ns); % 新息
Ptc=[0.95   0.05   % 转出模型转移概率矩阵
     0.05   0.95];
PtcFuzzy=[1   1/5   % 转出模型转移可能性矩阵
          1/5   1];
for k=1:Ns  % IMM filter
    if k==2 % initialization，CV model based kalman filter
        Z1=Z(:,1);
        Z2=Z(:,2);
        R2=R(:,:,2);
        % fCV_initialization 滤波初始化及参数设置
       [F_CV,G_CV,H_CV,Q,Xg2_CV,P2_CV]=fCV_initialization(T,Z1,Z2,R2,qm);
        % Z1,Z2,         k=1,2 时的直角坐标测量向量
        %  F,G,H,Q,Xg2,P2 依次为 状态转移矩阵，增益阵，测量阵，过程噪声阵，
        %  k=2 时的状态估计向量，状态估计误差协方差阵  
    end
    if k==3 % initialization，CA model based kalman filter
        Z1=Z(:,1);
        Z2=Z(:,2);
        Z3=Z(:,3);
        R3=R(:,:,3);
        % fCA_initialization 滤波初始化及参数设置
       [F_CA,G_CA,H_CA,Q,Xg3_CA,P3_CA]=fCA_initialization(T,Z1,Z2,Z3,R3,qm);
        % Z1,Z2,Z3,         k=1,2,3 时的直角坐标测量向量
        %  F,G,H,Q,Xg3,P3 依次为 状态转移矩阵，增益阵，测量阵，过程噪声阵，
        %  k=3 时的状态估计向量，状态估计误差协方差阵
    end
    if k==2
        Mqk_1=[1/2,1/2]; % 初始模型的概率为1/2,1/2
        MqFuzzyk_1=[1,1]; % 初始模型的可能性为1,1
        % expand Xg2_CV from 6*1 into 9*1
        Expand_Xg2_x_CV=padarray(Xg2_CV([1 2],1),[1,0],'post');
        Expand_Xg2_y_CV=padarray(Xg2_CV([3 4],1),[1,0],'post');
        Expand_Xg2_z_CV=padarray(Xg2_CV([5 6],1),[1,0],'post');
        Expand_Xg2_CV=[Expand_Xg2_x_CV;Expand_Xg2_y_CV;Expand_Xg2_z_CV];
        Xgqk=[Expand_Xg2_CV,Expand_Xg2_CV];  
            
        % expand P2_CV from 6*6 into 9*9
        Expand_P_1=padarray(P2_CV([1 2],:),[1,0],'post');
        Expand_P_2=padarray(P2_CV([3 4],:),[1,0],'post');
        Expand_P_3=padarray(P2_CV([5 6],:),[1,0],'post');
        Expand_P=[Expand_P_1;Expand_P_2;Expand_P_3];
        Expand_P_4=padarray(Expand_P(:,[1 2]),[0,1],'post');
        Expand_P_5=padarray(Expand_P(:,[3 4]),[0,1],'post');
        Expand_P_6=padarray(Expand_P(:,[5 6]),[0,1],'post');
        Expand_PP=[Expand_P_4 Expand_P_5 Expand_P_6];
        Pqk=cat(3,Expand_PP,Expand_PP);   %  2个模型的状态估计协方差,cat表示沿着第3维分布
            
        Xgq(:,:,k)=Xgqk;  % 保留状态估计值序列
        XgqFuzzy(:,:,k)=Xgqk;  % 保留状态估计值序列
        Pq(:,:,:,k)=Pqk; 
        PqFuzzy(:,:,:,k)=Pqk; 
        Mq(:,:,k-1)=Mqk_1; 
        MqFuzzy(:,:,k-1)=MqFuzzyk_1; 
    elseif k==3
        Mqk_1=[1/2,1/2]; % 初始模型的概率为1/2,1/2
        MqFuzzyk_1=[1,1]; % 初始模型的可能性为1,1
        Xgqk=[Xg3_CA,Xg3_CA];
        Pqk=cat(3,P3_CA,P3_CA);   %  2个模型的状态估计协方差,cat表示沿着第3维分布
        Xgq(:,:,k)=Xgqk;  % 保留状态估计值序列
        XgqFuzzy(:,:,k)=Xgqk;  % 保留状态估计值序列
        Pq(:,:,:,k)=Pqk; 
        PqFuzzy(:,:,:,k)=Pqk; 
        Mq(:,:,k-1)=Mqk_1; 
        MqFuzzy(:,:,k-1)=MqFuzzyk_1; 
    elseif k>=4
        % fIMM_filter
        Mqk_2=Mq(:,:,k-2);
        Xgqk_1=Xgq(:,:,k-1);    % 状态估计值           迭代
        Pqk_1=Pq(:,:,:,k-1);      % 状态估计误差协方差值 迭代
        Rk=R(:,:,k);
        Zk=Z(:,k);
        [Xgqk,Pqk,Mqk_1,Lk,Sk,vvk]=fIMM_filter(Zk,Xgqk_1,Pqk_1,F_CV,G_CV,H_CV,F_CA,G_CA,H_CA,qm,Rk,Ptc,Mqk_2);
        Xgq(:,:,k)=Xgqk;  % 保留状态估计值序列
        Pq(:,:,:,k)=Pqk; 
        Mq(:,:,k-1)=Mqk_1; 
%         L(:,:,k)=Lk; % 保留似然
%         S(:,:,:,k)=Sk; % 保留新息协方差
%         vv(:,:,k)=vvk; % 保留新息
        % fFuzzyIMM_filter
        Mqk_2=MqFuzzy(:,:,k-2);
        Xgqk_1=XgqFuzzy(:,:,k-1);    % 状态估计值           迭代
        Pqk_1=PqFuzzy(:,:,:,k-1);      % 状态估计误差协方差值 迭代
%         Rk=R(:,:,k);
%         Zk=Z(:,k);
        [Xgqk,Pqk,Mqk_1,Lk,Sk,vvk]=fFuzzyIMM_filter(Zk,Xgqk_1,Pqk_1,F_CV,G_CV,H_CV,F_CA,G_CA,H_CA,qmF,Rk,PtcFuzzy,Mqk_2);
        XgqFuzzy(:,:,k)=Xgqk;  % 保留状态估计值序列
        PqFuzzy(:,:,:,k)=Pqk; 
        MqFuzzy(:,:,k-1)=Mqk_1; 
%        Lfuzzy(:,:,k)=Lk; % 保留似然
    end   
end % for k=1:Ns % IMM filter

for k=2:Ns   
    % fIMM_filter
%     d=Mq(1,1,k-1)-Mq(1,2,k-1);
%     if d>=0
%         Xg(:,k)=Xgq(:,1,k); 
%     else
%         Xg(:,k)=Xgq(:,2,k); 
%     end
    Xg(:,k)=Xgq(:,1,k)*Mq(1,1,k-1)+Xgq(:,2,k)*Mq(1,2,k-1); 
    % fFuzzyIMM_filter
    d=MqFuzzy(1,1,k-1)-MqFuzzy(1,2,k-1);
    if d>=0
        XgFuzzy(:,k)=XgqFuzzy(:,1,k); 
    else
        XgFuzzy(:,k)=XgqFuzzy(:,2,k); 
    end
end

%%  计算误差值
for k=2:Ns 
    eXg(:,k)=Xg(:,k)-X(:,k);
    eXgFuzzy(:,k)=XgFuzzy(:,k)-X(:,k);
    eZ(:,k)=Z(:,k)-x(:,k);
end  
 
%%  计算平均值
MqAve=MqAve+Mq/Nre;
MqFuzzyAve=MqFuzzyAve+MqFuzzy/Nre;
MqFuzzyAve=MqFuzzyAve./(MqFuzzyAve(1,1,:)+MqFuzzyAve(1,2,:)); % 可能性->概率
eXgAve=eXgAve+eXg.^2/Nre;
eXgFuzzyAve=eXgFuzzyAve+eXgFuzzy.^2/Nre;
eZAve=eZAve+eZ.^2/Nre;
end
eXgAve=eXgAve.^0.5;
eXgFuzzyAve=eXgFuzzyAve.^0.5;
eZAve=eZAve.^0.5;
%% %%%%%%%%%%%%%% 绘图 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% 航迹绘图 %%%%%%
trackplot=0;
if trackplot==1
k=1:Ns;
figure(20);
plot(k,x(1,k),...
    k,Z(1,k),'y');
grid;

figure(21);
plot(k,x(2,k),...
    k,Z(2,k),'y');
grid;

figure(22);
plot(k,x(3,k),...
    k,Z(3,k),'y');
grid;

figure(23);
plot(x(1,k),x(2,k),...
    Z(1,k),Z(2,k),'y');
grid;
end
%%%% 绘制误差曲线 %%%%%%
k=3:Ns;

figure (1)
% plot(k,Xg(1,k)-x(1,k),...
plot(k,eXgAve(1,k),...
    k,eXgFuzzyAve(1,k),'g',...
    k,eZAve(1,k),'r')
title ('Position RMSE in x-Axis')
grid;
legend('Estimation Error - IMM','Estimation Error - HIMM','Measurment Error') 
xlabel('scans');
ylabel('meter');

figure (2)
plot(k,eXgAve(4,k),...
    k,eXgFuzzyAve(4,k),'g',...
    k,eZAve(2,k),'r')
title ('Position RMSE in y-Axis')
grid;
legend('Estimation Error - IMM','Estimation Error - HIMM','Measurment Error') 
xlabel('scans');
ylabel('meter');

figure (3)
plot(k,eXgAve(7,k),...
    k,eXgFuzzyAve(7,k),'g',...
    k,eZAve(3,k),'r')
title ('Position RMSE in z-Axis')
grid;
legend('Estimation Error - IMM','Estimation Error - HIMM','Measurment Error') 
xlabel('scans');
ylabel('meter');

figure (4)
plot(k,eXgAve(2,k),...
    k,eXgFuzzyAve(2,k),'g')
title ('eVgx-b/eVgxFuzzy-g') 
grid;

figure (5)
plot(k,eXgAve(5,k),...
    k,eXgFuzzyAve(5,k),'g')
title ('eVgy-b/eVgyFuzzy-g') 
grid;

figure (6)
plot(k,eXgAve(8,k),...
    k,eXgFuzzyAve(8,k),'g')
title ('eVgz-b/eVgzFuzzy-g') 
grid;

figure (7)
plot(k,eXgAve(3,k),...
    k,eXgFuzzyAve(3,k),'g')
title ('eAgx-b/eAgxFuzzy-g') 
grid;

figure (8)
plot(k,eXgAve(6,k),...
    k,eXgFuzzyAve(6,k),'g')
title ('eAgy-b/eAgyFuzzy-g') 
grid;

figure (9)
plot(k,eXgAve(9,k),...
    k,eXgFuzzyAve(9,k),'g')
title ('eAgz-b/eAgzFuzzy-g') 
grid;

figure (10)
temp=zeros(2,Ns);
temp(1,:)=MqAve(1,1,:);
temp(2,:)=MqAve(1,2,:);
plot(k,temp(1,k),'b',...
    k, temp(2,k),'r')
title ('Model Probability') 
legend('DWNA','DWPA') 
xlabel('scans');
ylabel('probability');
grid;

figure (11)
temp=zeros(2,Ns);
temp(1,:)=MqFuzzyAve(1,1,:);
temp(2,:)=MqFuzzyAve(1,2,:);
plot(k,temp(1,k),'b',...
    k, temp(2,k),'r')
title ('Model Probability') 
legend('DWNA','DWPA') 
xlabel('scans');
ylabel('probability');
grid;

% %%%% 绘制误差曲线 %%%%%%
% k=3:Ns;
% 
% figure (1)
% plot(k,Xg(1,k)-x(1,k),...
%     k,XgFuzzy(1,k)-x(1,k),'g',...
%     k,Z(1,k)-x(1,k),'r')
% title ('egx-b/egxFuzzy-g/elx-r')
% grid;
% 
% figure (2)
% plot(k,Xg(4,k)-x(2,k),...
%     k,XgFuzzy(4,k)-x(2,k),'g',...
%     k,Z(2,k)-x(2,k),'r')
% title ('egy-b/egyFuzzy-g/ely-r')
% grid;
% 
% figure (3)
% plot(k,Xg(7,k)-x(3,k),...
%     k,XgFuzzy(7,k)-x(3,k),'g',...
%     k,Z(3,k)-x(3,k),'r')
% title ('egz-b/egzFuzzy-g/elz-r')
% grid;
% 
% figure (4)
% plot(k,Xg(2,k),...
%     k,XgFuzzy(2,k),'g')
% title ('Vgx-b/VgxFuzzy-g') 
% grid;
% 
% figure (5)
% plot(k,Xg(5,k),...
%     k,XgFuzzy(5,k),'g')
% title ('Vgy-b/VgyFuzzy-g') 
% grid;
% 
% figure (6)
% plot(k,Xg(8,k),...
%     k,XgFuzzy(8,k),'g')
% title ('Vgz-b/VgzFuzzy-g') 
% grid;
% 
% figure (7)
% plot(k,Xg(3,k),...
%     k,XgFuzzy(3,k),'g')
% title ('Agx-b/AgxFuzzy-g') 
% grid;
% 
% figure (8)
% plot(k,Xg(6,k),...
%     k,XgFuzzy(6,k),'g')
% title ('Agy-b/AgyFuzzy-g') 
% grid;
% 
% figure (9)
% plot(k,Xg(9,k),...
%     k,XgFuzzy(9,k),'g')
% title ('Agz-b/AgzFuzzy-g') 
% grid;



