% main function  
% compare{fFuzzy_IMMfilter,fClassic_IMMfilter}
% two models (CV and CA)
% 3 dimension example

%% 滤波器 仿真参数设定
T=2; %  滤波器采样间隔
Tsim=60; % 滤波器仿真时间
Ns=Tsim/T;% 滤波器仿真总采样点数

K=Ns; % 滤波器仿真总采样点数
Nr=50; % 仿真运行次数(循环次数)

Mq=zeros(1,2,K); % 模型概率
MqAve=zeros(1,2,K); % 平均模型概率
L=zeros(1,2,K); % 模型似然
S=zeros(1,2,K); % 新息协方差
vv=zeros(1,2,K); % 新息

for nr=1:Nr % 需改变该循环的位置
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main_CA_CMKF   ：CA_CMKF 主调函数

q=1; % 过程噪声标准偏差的平方,      米        实际值
qm=3;%20     % 滤波器设置的过程噪声标准偏差的平方 米  模型值

rxk=1*50;
Rk=rxk^2;     % 传感器测量噪声标准偏差 角度 实际值
Rmk=1*rxk^2;         % 传感器测量噪声标准偏差 模型值

x=zeros(1,Ns); % 直角坐标 航迹真值 序列  1D
vx=zeros(1,Ns); % 直角坐标 航迹真值 序列  1D
Z=zeros(1,Ns); % 直角坐标 测量值 序列
R=zeros(1,1,Ns); % 直角坐标 测量值 序列
Xg=zeros(2,Ns); % 直角坐标 状态估计值 序列
Ag=zeros(1,Ns); % 直角坐标 加速度估计值 序列

x0=[10000];
v0=[-150];
xk=zeros(1,1);
vk=zeros(1,1);
a=zeros(1,Ns); % 加速度序列 定义
for k=1:14
    a(:,k)=[0];
end
for k=15:20
    if Type==1
        a(:,k)=[-5];
    elseif Type==2
        a(:,k)=[-25];
    elseif Type==3
        a(:,k)=[-45];        
    end
end
for k=21:Ns
    a(:,k)=[0];
end

for k=1:Ns
    if k==1
        xk=x0;
        vk=v0;
        x(:,k)=xk; % 保留航迹坐标真值序列
    else
        xk_1=xk;       % 航迹坐标真值   迭代
        vk_1=vk;       % 航迹速度真值   迭代
        ak_1=a(:,k-1); % 航迹加速度真值 迭代
        [xk,vk]=ftrackgenerator(xk_1,vk_1,ak_1,T,q);  % 调用 ftrackgenerator 生成航路
        x(:,k)=xk; % 保留航迹坐标真值序列
        vx(:,k)=vk; % 保留航迹坐标真值序列
    end
        
    [Zk]=fsensor(xk,rxk);   % 生成测量－球坐标
    Z(:,k)=Zk;
    R(:,:,k)=Rk;
    
    if k==2
        Z1=Z(:,1);
        Z2=Z(:,2);
        R2=R(:,:,2);
        % fCV_CMKF_initialization 滤波初始化及参数设置
       [F,G,B,H,Xgm2,Pm2]=fCVU_CMKF_initialization(T,Z1,Z2,R2);
        % Z1,Z2,         k=1,2 时的直角坐标测量向量
        %  F,G,B,H,Q,Xg2,P2 依次为 状态转移矩阵，增益阵，机动输入矩阵，测量阵，过程噪声阵，
        %  k=2 时的状态估计向量，状态估计误差协方差阵
       [Xg2,P2,Mq2,Ptc]=fIMM_initialization(Xgm2,Pm2);
        
    end
    
    if k>=2
        if k==2
            Mqk_1=Mq2;  %%  滤波初始赋值
            Xgqk=Xg2;   
            Pqk=P2;
            Xgq(:,:,k)=Xg2;  % 保留状态估计值序列
            Pq(:,:,:,k)=Pqk; 
            Mq(:,:,k-1)=Mqk_1; 
        else
            Mqk_2=Mqk_1;
            Xgqk_1=Xgqk; % 状态估计值           迭代
            Pqk_1=Pqk;      % 状态估计误差协方差值 迭代
            [Xgqk,Pqk,Mqk_1,Lk,Sk,vvk]=fIMM_5model(Zk,Xgqk_1,Pqk_1,F,G,B,H,qm,u,Rmk,Ptc,Mqk_2);
            Xgq(:,:,k)=Xgqk;  % 保留状态估计值序列
            Pq(:,:,:,k)=Pqk; 
            Mq(:,:,k-1)=Mqk_1; 
            L(:,:,k)=Lk; % 保留似然
            S(:,:,k)=Sk; % 保留新息协方差
            vv(:,:,k)=vvk; % 保留新息
        end
    end
        
end % for k=1:Ns

MqAve=MqAve+Mq/Nr;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  基于IMM模型概率的 fBayesianClassifier, fFuzzyClassifier
PrchkComBs=0.3333*ones(1,Nc2);   % fBayesianClassifier 目标 k 时刻后验类型概率
Prchk_1ComBs=0.3333*ones(1,Nc2); % fBayesianClassifier 目标 k-1 时刻后验类型概率
PrchComBs=zeros(K,Nc2);          % fBayesianClassifier 目标后验类型概率序列 

PrchkFuzzy=0.3333*ones(1,Nc2);   % fFuzzyClassifier 目标 k 时刻后验类型概率
PochkFuzzy=0.3333*ones(1,Nc2);   % fFuzzyClassifier 目标 k 时刻后验类型可能性
Prchk_1Fuzzy=0.3333*ones(1,Nc2); % fFuzzyClassifier 目标 k-1 时刻后验类型概率
Pochk_1Fuzzy=ones(1,Nc2);   % fFuzzyClassifier 目标 k 时刻后验类型可能性
PrchFuzzy=zeros(K,Nc2);   % fFuzzyClassifier 目标后验类型概率序列 
PochFuzzy=zeros(K,Nc2);   % fFuzzyClassifier 目标后验类型可能性序列 

transitionmatrix=1;
if transitionmatrix==1  % 5个机动模型    
    Prf_c=[0.33 0.33 0.33    % fBayesianClassifier 目标特征-类型转移阵Pr(c=i|f),i=1:Nc2。
           0.00 0.50 0.50     %  f=1,2/3，4/5分别表示弱机动，中等机动，强机动，c=1,2,3 分别表示民航,轰炸机,战斗机 
           0.00 0.50 0.50
           0.00 0.00 1.00
           0.00 0.00 1.00];    
    
   Prf_c_fuzzy=[1 1 1   % fFuzzyClassifier 目标特征-类型转移阵Pr(c=i|f),i=1:Nc2。
                0 1 1   %  f=1,2/3，4/5分别表示弱机动，中等机动，强机动，c=1,2,3 分别表示民航,轰炸机,战斗机 
                0 1 1
                0 0 1
                0 0 1];               
else  % 3个机动模型
    Prf_c=[0.33 0.33 0.33    % fBayesianClassifier 目标特征-类型转移阵Pr(c=i|f),i=1:Nc2。
           0    0.50 0.50
           0    0    1];     % 当c=1时表示目标类型1,f=1，2，3分别表示弱机动，中等机动，强机动
   
    Prf_c_fuzzy=[0.33 0.33 0.33    % fFuzzyClassifier 目标特征-类型转移阵Pr(c=i|f),i=1:Nc2。
                 0    0.50 0.50
                 0    0    1];     % 当c=1时表示目标类型1,f=1，2，3分别表示弱机动，中等机动，强机动
end
   
for k=1:K     
    Mqk=Mq(1,:,k); 
    Lk=L(1,:,k);
    Sk=S(1,:,k);
    vvk=vv(1,:,k);
    if transitionmatrix==1
        Mqk1=cat(2,Mqk(1,1),Mqk(1,2),Mqk(1,3),Mqk(1,4),Mqk(1,5));
        Lk1=cat(2,Lk(1,1),Lk(1,2),Lk(1,3),Lk(1,4),Lk(1,5));
    else
        Mqk1=cat(2,Mqk(1,1),Mqk(1,2)+Mqk(1,3),Mqk(1,4)+Mqk(1,5));
        Lk1=cat(2,Lk(1,1),Lk(1,2)+Lk(1,3),Lk(1,4)+Lk(1,5));
    end
      
    if k<10  % 10 等到加速度估计稳定后 方才启动基于运动学参数估计的 分类器
        PrchComBs(k,:)=PrchkComBs(1,:);  % 保存分类概率序列
    else
        Prchk_1ComBs=PrchkComBs;
        PrchkComBs=fBayesianClassifier(Nc2,Prchk_1ComBs,Mqk1,Lk1,Prf_c);  % 调用fBayesianClassifier
        PrchComBs(k,:)=PrchkComBs(1,:);   % 保存分类概率序列
    end

    if k<10  % 10 等到加速度估计稳定后 方才启动基于运动学参数估计的 分类器
        PrchFuzzy(k,:)=PrchkFuzzy(1,:);  % 保存分类概率序列
    else
        Pochk_1Fuzzy=PochkFuzzy;
        [PochkFuzzy,PrchkFuzzy]=fFuzzyClassifier(Nc2,Pochk_1Fuzzy,Mqk1,Lk1,Prf_c_fuzzy);  % 调用fFuzzyClassifier
        PochFuzzy(k,:)=PochkFuzzy(1,:);   % 保存分类概率序列
        PrchFuzzy(k,:)=PrchkFuzzy(1,:);   % 保存分类概率序列
    end    
end
    PrchComBsAve=PrchComBsAve+PrchComBs/Nr; % 计算平均概率
    PochFuzzyAve=PochFuzzyAve+PochFuzzy/Nr; % 计算平均可能性
    PrchFuzzyAve=PrchFuzzyAve+PrchFuzzy/Nr; % 计算平均可能性    
end  %for nr=1:Nr 仿真运行次数(循环次数)

%%% 后验目标分类概率绘图 %%%%%%%%%%
k=1:K;

figure (1)
plot(k,PrchComBsAve(k,1),'K-',...
    k,PrchComBsAve(k,2),'K:',...
    k,PrchComBsAve(k,3),'kx')
title ('Average Probabilities For Bayesian Method')
legend('TYPE 1','TYPE 2','TYPE 3') 
xlabel('seconds');
ylabel('probability');
    
figure (3)
plot(k,PochFuzzyAve(k,1),'K-',...
     k,PochFuzzyAve(k,2),'K:',...
     k,PochFuzzyAve(k,3),'kx')
title ('Average Possibilities For Fuzzy Method')
legend('TYPE 1','TYPE 2','TYPE 3') 
xlabel('seconds');
ylabel('possibility');
     
 figure (4)
 plot(k,PrchFuzzyAve(k,1),'K-',...
     k,PrchFuzzyAve(k,2),'K:',...
     k,PrchFuzzyAve(k,3),'kx')
 title ('Average Probabilities For Fuzzy Method')
 legend('TYPE 1','TYPE 2','TYPE 3') 
 xlabel('seconds');
 ylabel('probability');       

%%%% 绘制误差曲线 %%%%%%
k=3:Ns;

ploterror=0;

if ploterror==1

Xg(:,:)=Xgq(:,1,:);
figure (11)
plot(k,Xg(1,k)-x(1,k),...
    k,Z(1,k)-x(1,k),'r')
title ('egx1-b/elx1-r')
grid;

figure (21)
plot(k,Xg(2,k))
title ('Vgx-b')
grid;

figure (31)
plot(k,x(1,k))
title ('x-b')
grid;

figure (41)
plot(k,Z(1,k))
title ('Z-b')
grid;
end

% 模型概率
MqAve1=MqAve(:,1,:);
MqAve2=MqAve(:,2,:);
MqAve3=MqAve(:,3,:);
MqAve4=MqAve(:,4,:);
MqAve5=MqAve(:,5,:);
figure (9)
plot(k,MqAve1(1,k),'k-',...
     k,MqAve2(1,k),'k:',...
     k,MqAve3(1,k),'kx',...
     k,MqAve4(1,k),'k-.',...
     k,MqAve5(1,k),'k*')
title ('Average Model Probabilities-IMM')
%legend('CV','-2g','+2g','-4g','+4g') 
legend('CV','+3g','-3g','+5g','-5g') 
xlabel('seconds');
ylabel('probabilities');