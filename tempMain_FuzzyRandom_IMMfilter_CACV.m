% main function  
% compare{fFuzzy_IMMfilter,fClassic_IMMfilter}
% two models (CV and CA)
% 3 dimension example

%% �˲��� ��������趨
T=2; %  �˲����������
Tsim=60; % �˲�������ʱ��
Ns=Tsim/T;% �˲��������ܲ�������

K=Ns; % �˲��������ܲ�������
Nr=50; % �������д���(ѭ������)

Mq=zeros(1,2,K); % ģ�͸���
MqAve=zeros(1,2,K); % ƽ��ģ�͸���
L=zeros(1,2,K); % ģ����Ȼ
S=zeros(1,2,K); % ��ϢЭ����
vv=zeros(1,2,K); % ��Ϣ

for nr=1:Nr % ��ı��ѭ����λ��
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main_CA_CMKF   ��CA_CMKF ��������

q=1; % ����������׼ƫ���ƽ��,      ��        ʵ��ֵ
qm=3;%20     % �˲������õĹ���������׼ƫ���ƽ�� ��  ģ��ֵ

rxk=1*50;
Rk=rxk^2;     % ����������������׼ƫ�� �Ƕ� ʵ��ֵ
Rmk=1*rxk^2;         % ����������������׼ƫ�� ģ��ֵ

x=zeros(1,Ns); % ֱ������ ������ֵ ����  1D
vx=zeros(1,Ns); % ֱ������ ������ֵ ����  1D
Z=zeros(1,Ns); % ֱ������ ����ֵ ����
R=zeros(1,1,Ns); % ֱ������ ����ֵ ����
Xg=zeros(2,Ns); % ֱ������ ״̬����ֵ ����
Ag=zeros(1,Ns); % ֱ������ ���ٶȹ���ֵ ����

x0=[10000];
v0=[-150];
xk=zeros(1,1);
vk=zeros(1,1);
a=zeros(1,Ns); % ���ٶ����� ����
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
        x(:,k)=xk; % ��������������ֵ����
    else
        xk_1=xk;       % ����������ֵ   ����
        vk_1=vk;       % �����ٶ���ֵ   ����
        ak_1=a(:,k-1); % �������ٶ���ֵ ����
        [xk,vk]=ftrackgenerator(xk_1,vk_1,ak_1,T,q);  % ���� ftrackgenerator ���ɺ�·
        x(:,k)=xk; % ��������������ֵ����
        vx(:,k)=vk; % ��������������ֵ����
    end
        
    [Zk]=fsensor(xk,rxk);   % ���ɲ�����������
    Z(:,k)=Zk;
    R(:,:,k)=Rk;
    
    if k==2
        Z1=Z(:,1);
        Z2=Z(:,2);
        R2=R(:,:,2);
        % fCV_CMKF_initialization �˲���ʼ������������
       [F,G,B,H,Xgm2,Pm2]=fCVU_CMKF_initialization(T,Z1,Z2,R2);
        % Z1,Z2,         k=1,2 ʱ��ֱ�������������
        %  F,G,B,H,Q,Xg2,P2 ����Ϊ ״̬ת�ƾ��������󣬻���������󣬲����󣬹���������
        %  k=2 ʱ��״̬����������״̬�������Э������
       [Xg2,P2,Mq2,Ptc]=fIMM_initialization(Xgm2,Pm2);
        
    end
    
    if k>=2
        if k==2
            Mqk_1=Mq2;  %%  �˲���ʼ��ֵ
            Xgqk=Xg2;   
            Pqk=P2;
            Xgq(:,:,k)=Xg2;  % ����״̬����ֵ����
            Pq(:,:,:,k)=Pqk; 
            Mq(:,:,k-1)=Mqk_1; 
        else
            Mqk_2=Mqk_1;
            Xgqk_1=Xgqk; % ״̬����ֵ           ����
            Pqk_1=Pqk;      % ״̬�������Э����ֵ ����
            [Xgqk,Pqk,Mqk_1,Lk,Sk,vvk]=fIMM_5model(Zk,Xgqk_1,Pqk_1,F,G,B,H,qm,u,Rmk,Ptc,Mqk_2);
            Xgq(:,:,k)=Xgqk;  % ����״̬����ֵ����
            Pq(:,:,:,k)=Pqk; 
            Mq(:,:,k-1)=Mqk_1; 
            L(:,:,k)=Lk; % ������Ȼ
            S(:,:,k)=Sk; % ������ϢЭ����
            vv(:,:,k)=vvk; % ������Ϣ
        end
    end
        
end % for k=1:Ns

MqAve=MqAve+Mq/Nr;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  ����IMMģ�͸��ʵ� fBayesianClassifier, fFuzzyClassifier
PrchkComBs=0.3333*ones(1,Nc2);   % fBayesianClassifier Ŀ�� k ʱ�̺������͸���
Prchk_1ComBs=0.3333*ones(1,Nc2); % fBayesianClassifier Ŀ�� k-1 ʱ�̺������͸���
PrchComBs=zeros(K,Nc2);          % fBayesianClassifier Ŀ��������͸������� 

PrchkFuzzy=0.3333*ones(1,Nc2);   % fFuzzyClassifier Ŀ�� k ʱ�̺������͸���
PochkFuzzy=0.3333*ones(1,Nc2);   % fFuzzyClassifier Ŀ�� k ʱ�̺������Ϳ�����
Prchk_1Fuzzy=0.3333*ones(1,Nc2); % fFuzzyClassifier Ŀ�� k-1 ʱ�̺������͸���
Pochk_1Fuzzy=ones(1,Nc2);   % fFuzzyClassifier Ŀ�� k ʱ�̺������Ϳ�����
PrchFuzzy=zeros(K,Nc2);   % fFuzzyClassifier Ŀ��������͸������� 
PochFuzzy=zeros(K,Nc2);   % fFuzzyClassifier Ŀ��������Ϳ��������� 

transitionmatrix=1;
if transitionmatrix==1  % 5������ģ��    
    Prf_c=[0.33 0.33 0.33    % fBayesianClassifier Ŀ������-����ת����Pr(c=i|f),i=1:Nc2��
           0.00 0.50 0.50     %  f=1,2/3��4/5�ֱ��ʾ���������еȻ�����ǿ������c=1,2,3 �ֱ��ʾ��,��ը��,ս���� 
           0.00 0.50 0.50
           0.00 0.00 1.00
           0.00 0.00 1.00];    
    
   Prf_c_fuzzy=[1 1 1   % fFuzzyClassifier Ŀ������-����ת����Pr(c=i|f),i=1:Nc2��
                0 1 1   %  f=1,2/3��4/5�ֱ��ʾ���������еȻ�����ǿ������c=1,2,3 �ֱ��ʾ��,��ը��,ս���� 
                0 1 1
                0 0 1
                0 0 1];               
else  % 3������ģ��
    Prf_c=[0.33 0.33 0.33    % fBayesianClassifier Ŀ������-����ת����Pr(c=i|f),i=1:Nc2��
           0    0.50 0.50
           0    0    1];     % ��c=1ʱ��ʾĿ������1,f=1��2��3�ֱ��ʾ���������еȻ�����ǿ����
   
    Prf_c_fuzzy=[0.33 0.33 0.33    % fFuzzyClassifier Ŀ������-����ת����Pr(c=i|f),i=1:Nc2��
                 0    0.50 0.50
                 0    0    1];     % ��c=1ʱ��ʾĿ������1,f=1��2��3�ֱ��ʾ���������еȻ�����ǿ����
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
      
    if k<10  % 10 �ȵ����ٶȹ����ȶ��� �������������˶�ѧ�������Ƶ� ������
        PrchComBs(k,:)=PrchkComBs(1,:);  % ��������������
    else
        Prchk_1ComBs=PrchkComBs;
        PrchkComBs=fBayesianClassifier(Nc2,Prchk_1ComBs,Mqk1,Lk1,Prf_c);  % ����fBayesianClassifier
        PrchComBs(k,:)=PrchkComBs(1,:);   % ��������������
    end

    if k<10  % 10 �ȵ����ٶȹ����ȶ��� �������������˶�ѧ�������Ƶ� ������
        PrchFuzzy(k,:)=PrchkFuzzy(1,:);  % ��������������
    else
        Pochk_1Fuzzy=PochkFuzzy;
        [PochkFuzzy,PrchkFuzzy]=fFuzzyClassifier(Nc2,Pochk_1Fuzzy,Mqk1,Lk1,Prf_c_fuzzy);  % ����fFuzzyClassifier
        PochFuzzy(k,:)=PochkFuzzy(1,:);   % ��������������
        PrchFuzzy(k,:)=PrchkFuzzy(1,:);   % ��������������
    end    
end
    PrchComBsAve=PrchComBsAve+PrchComBs/Nr; % ����ƽ������
    PochFuzzyAve=PochFuzzyAve+PochFuzzy/Nr; % ����ƽ��������
    PrchFuzzyAve=PrchFuzzyAve+PrchFuzzy/Nr; % ����ƽ��������    
end  %for nr=1:Nr �������д���(ѭ������)

%%% ����Ŀ�������ʻ�ͼ %%%%%%%%%%
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

%%%% ����������� %%%%%%
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

% ģ�͸���
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