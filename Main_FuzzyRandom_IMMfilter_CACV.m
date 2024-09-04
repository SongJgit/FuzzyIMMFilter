% main function  
% compare{fFuzzy_IMMfilter,fClassic_IMMfilter}
% two models (CV and CA)
% 3 dimension example

%% ��������趨
Nre=50;   % ѭ������
radar=1; % 1-fire control 2-surveillance
if radar==1
    T=0.2; %  ������� 2 0.2
    Tsim=40; % ����ʱ�� 160 40
elseif radar==2
    T=2; %  ������� 2 0.2
    Tsim=160; % ����ʱ�� 160 40
end
Ns=Tsim/T;% �����ܲ�������
MqAve=zeros(1,2,Ns); % ƽ��ģ�͸���
MqFuzzyAve=zeros(1,2,Ns); % ƽ��ģ�͸���
eXgAve=zeros(9,Ns);  % ƽ���������
eXgFuzzyAve=zeros(9,Ns); % ƽ���������-fuzzy
eZAve=zeros(3,Ns);       % ƽ���������


%% ������������
q=3*[1;1;1]; % [1;1;0.5] ����������׼ƫ��,      ��        ʵ��ֵ
%q=[0;0;0];
qm=3;  % 1, 3     % �˲������õĹ���������׼ƫ�� ��  ģ��ֵ
qmF=3;% 1, 3     % �˲������õĹ���������׼ƫ�� ��  ģ��ֵ

%% ������������
if radar==1
    rbeitak=0.1*pi/180;     % 0.9 ��������λ�ǲ���������׼ƫ�� �Ƕ� ʵ��ֵ
    rebuxinonk=0.1*pi/180;  % 0.9 �������ߵͽǲ���������׼ƫ�� �Ƕ� ʵ��ֵ
    rDk=10;                % 100 �������������������׼ƫ��   ��   ʵ��ֵ

    rmbeitak=0.1*pi/180;         % 0.9 1.35 1.8 ��������λ�ǲ���������׼ƫ�� ģ��ֵ
    rmebuxinonk=0.1*pi/180;      % 0.9 1.35 1.8 �������ߵͽǲ���������׼ƫ�� ģ��ֵ
    rmDk=10;                    % 100 150 200 �������������������׼ƫ��   ģ��ֵ
elseif radar==2
    rbeitak=0.9*pi/180;     % 0.9 ��������λ�ǲ���������׼ƫ�� �Ƕ� ʵ��ֵ
    rebuxinonk=0.9*pi/180;  % 0.9 �������ߵͽǲ���������׼ƫ�� �Ƕ� ʵ��ֵ
    rDk=100;                % 100 �������������������׼ƫ��   ��   ʵ��ֵ

    rmbeitak=0.9*pi/180;         % 0.9 1.35 1.8 ��������λ�ǲ���������׼ƫ�� ģ��ֵ
    rmebuxinonk=0.9*pi/180;      % 0.9 1.35 1.8 �������ߵͽǲ���������׼ƫ�� ģ��ֵ
    rmDk=100;                    % 100 150 200 �������������������׼ƫ��   ģ��ֵ
end

%% ������ֵ/����ֵ ��������
X=zeros(9,Ns); % ֱ������ ����״̬��ֵ ����
x=zeros(3,Ns); % ֱ������ ����������ֵ ����
Z=zeros(3,Ns); % ֱ������ ����ֵ ����
R=zeros(3,3,Ns); % ֱ������ �������Э���� ����

%% ������ֵ��ʼֵ/����/���� ����
if radar==1
    x0=[12000;8000;1000];
elseif radar==2
	x0=[120000;80000;20000];  
end
v0=[-100;-100;0];
xk=zeros(3,1);
vk=zeros(3,1);
a=zeros(3,Ns); % ���ٶ����� ����
for n=1:Nre  % ���д���
if radar==1
    jump=80; % +randi(50);
    for k=1:jump % 80
        a(:,k)=[0;0;0]; 
    end
    for k=jump+1:jump+51 %  80��130
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

%for n=1:Nre  % ���д���
%% ����������ת��
for k=1:Ns % ����������ת��
    if k==1
        xk=x0;
        vk=v0;
        x(:,k)=xk; % ��������������ֵ����
    else
        xk_1=xk;       % ����������ֵ   ����
        vk_1=vk;       % �����ٶ���ֵ   ����
        ak_1=a(:,k-1); % �������ٶ���ֵ ����
        [xk,vk]=ftrackgenerator(xk_1,vk_1,ak_1,T,q);  % ���� ftrackgenerator ���ɺ�·
        X(:,k)=[xk(1);vk(1);ak_1(1);xk(2);vk(2);ak_1(2);xk(3);vk(3);ak_1(3)]; % ��������������ֵ����
        x(:,k)=xk; % ��������������ֵ����
    end
       
    [beitak,ebuxinonk,Dk]=fzhiqiuCoordinateTransformer(xk);  % ֱ/������任
     % beitak     ��λ������
     % ebuxinonk  �ߵͽ�����
     % Dk         ��������
    
    [zbeitak,zebuxinonk,zDk]=fsensor(beitak,ebuxinonk,Dk,rbeitak,rebuxinonk,rDk);   % ���ɲ�����������
     % zbeitak     ��λ�ǲ���
     % zebuxinonk  �ߵͽǲ���
     % zDk         �������
     % rbeitak    ��������λ�ǲ���������׼ƫ��
     % rebuxinonk �������ߵͽǲ���������׼ƫ��
     % rDk        �������������������׼ƫ��
    
    Zk=fqiuzhiCoordinateTransformer(zbeitak,zebuxinonk,zDk);  % ��/ֱ����任��
     % Zk          ֱ����������
    Z(:,k)=Zk;
    
    % ������ֱ������������������� ��/ֱ����任
    Rk=fqiuzhiCeliangwuchafangchaTransformer(zbeitak,zebuxinonk,zDk,rmbeitak,rmebuxinonk,rmDk);
     % rmbeitak    ��������λ�ǲ���������׼ƫ�� ģ��ֵ
     % rmebuxinonk �������ߵͽǲ���������׼ƫ�� ģ��ֵ
     % rmDk        �������������������׼ƫ��   ģ��ֵ
     % Rk          ������ֱ������������������� ģ��ֵ
    R(:,:,k)=Rk;
end % for k=1:Ns % ����������ת��

%% IMM filter
Xg=zeros(9,Ns); % ֱ������ ��Ȩƽ��״̬����ֵ ����
XgFuzzy=zeros(9,Ns); % ֱ������ ��Ȩƽ��״̬����ֵ ���� FuzzyIMM filter
eXg=zeros(9,Ns); % ���ֵ
eXgFuzzy=zeros(9,Ns); 
eZ=zeros(3,Ns);
Xgq=zeros(9,2,Ns); % ֱ������ CV/CA״̬����ֵ ����
XgqFuzzy=zeros(9,2,Ns); % ֱ������ CV/CA״̬����ֵ ����
Pq=zeros(9,9,2,Ns); 
PqFuzzy=zeros(9,9,2,Ns); 
Mq=zeros(1,2,Ns); % ģ�͸��� ����
MqFuzzy=zeros(1,2,Ns); % ģ�͸��� ����
L=zeros(1,2,Ns); % ģ����Ȼ
Lfuzzy=zeros(1,2,Ns); % ģ����Ȼ
S=zeros(3,3,2,Ns); % ��ϢЭ����
vv=zeros(3,2,Ns); % ��Ϣ
Ptc=[0.95   0.05   % ת��ģ��ת�Ƹ��ʾ���
     0.05   0.95];
PtcFuzzy=[1   1/5   % ת��ģ��ת�ƿ����Ծ���
          1/5   1];
for k=1:Ns  % IMM filter
    if k==2 % initialization��CV model based kalman filter
        Z1=Z(:,1);
        Z2=Z(:,2);
        R2=R(:,:,2);
        % fCV_initialization �˲���ʼ������������
       [F_CV,G_CV,H_CV,Q,Xg2_CV,P2_CV]=fCV_initialization(T,Z1,Z2,R2,qm);
        % Z1,Z2,         k=1,2 ʱ��ֱ�������������
        %  F,G,H,Q,Xg2,P2 ����Ϊ ״̬ת�ƾ��������󣬲����󣬹���������
        %  k=2 ʱ��״̬����������״̬�������Э������  
    end
    if k==3 % initialization��CA model based kalman filter
        Z1=Z(:,1);
        Z2=Z(:,2);
        Z3=Z(:,3);
        R3=R(:,:,3);
        % fCA_initialization �˲���ʼ������������
       [F_CA,G_CA,H_CA,Q,Xg3_CA,P3_CA]=fCA_initialization(T,Z1,Z2,Z3,R3,qm);
        % Z1,Z2,Z3,         k=1,2,3 ʱ��ֱ�������������
        %  F,G,H,Q,Xg3,P3 ����Ϊ ״̬ת�ƾ��������󣬲����󣬹���������
        %  k=3 ʱ��״̬����������״̬�������Э������
    end
    if k==2
        Mqk_1=[1/2,1/2]; % ��ʼģ�͵ĸ���Ϊ1/2,1/2
        MqFuzzyk_1=[1,1]; % ��ʼģ�͵Ŀ�����Ϊ1,1
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
        Pqk=cat(3,Expand_PP,Expand_PP);   %  2��ģ�͵�״̬����Э����,cat��ʾ���ŵ�3ά�ֲ�
            
        Xgq(:,:,k)=Xgqk;  % ����״̬����ֵ����
        XgqFuzzy(:,:,k)=Xgqk;  % ����״̬����ֵ����
        Pq(:,:,:,k)=Pqk; 
        PqFuzzy(:,:,:,k)=Pqk; 
        Mq(:,:,k-1)=Mqk_1; 
        MqFuzzy(:,:,k-1)=MqFuzzyk_1; 
    elseif k==3
        Mqk_1=[1/2,1/2]; % ��ʼģ�͵ĸ���Ϊ1/2,1/2
        MqFuzzyk_1=[1,1]; % ��ʼģ�͵Ŀ�����Ϊ1,1
        Xgqk=[Xg3_CA,Xg3_CA];
        Pqk=cat(3,P3_CA,P3_CA);   %  2��ģ�͵�״̬����Э����,cat��ʾ���ŵ�3ά�ֲ�
        Xgq(:,:,k)=Xgqk;  % ����״̬����ֵ����
        XgqFuzzy(:,:,k)=Xgqk;  % ����״̬����ֵ����
        Pq(:,:,:,k)=Pqk; 
        PqFuzzy(:,:,:,k)=Pqk; 
        Mq(:,:,k-1)=Mqk_1; 
        MqFuzzy(:,:,k-1)=MqFuzzyk_1; 
    elseif k>=4
        % fIMM_filter
        Mqk_2=Mq(:,:,k-2);
        Xgqk_1=Xgq(:,:,k-1);    % ״̬����ֵ           ����
        Pqk_1=Pq(:,:,:,k-1);      % ״̬�������Э����ֵ ����
        Rk=R(:,:,k);
        Zk=Z(:,k);
        [Xgqk,Pqk,Mqk_1,Lk,Sk,vvk]=fIMM_filter(Zk,Xgqk_1,Pqk_1,F_CV,G_CV,H_CV,F_CA,G_CA,H_CA,qm,Rk,Ptc,Mqk_2);
        Xgq(:,:,k)=Xgqk;  % ����״̬����ֵ����
        Pq(:,:,:,k)=Pqk; 
        Mq(:,:,k-1)=Mqk_1; 
%         L(:,:,k)=Lk; % ������Ȼ
%         S(:,:,:,k)=Sk; % ������ϢЭ����
%         vv(:,:,k)=vvk; % ������Ϣ
        % fFuzzyIMM_filter
        Mqk_2=MqFuzzy(:,:,k-2);
        Xgqk_1=XgqFuzzy(:,:,k-1);    % ״̬����ֵ           ����
        Pqk_1=PqFuzzy(:,:,:,k-1);      % ״̬�������Э����ֵ ����
%         Rk=R(:,:,k);
%         Zk=Z(:,k);
        [Xgqk,Pqk,Mqk_1,Lk,Sk,vvk]=fFuzzyIMM_filter(Zk,Xgqk_1,Pqk_1,F_CV,G_CV,H_CV,F_CA,G_CA,H_CA,qmF,Rk,PtcFuzzy,Mqk_2);
        XgqFuzzy(:,:,k)=Xgqk;  % ����״̬����ֵ����
        PqFuzzy(:,:,:,k)=Pqk; 
        MqFuzzy(:,:,k-1)=Mqk_1; 
%        Lfuzzy(:,:,k)=Lk; % ������Ȼ
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

%%  �������ֵ
for k=2:Ns 
    eXg(:,k)=Xg(:,k)-X(:,k);
    eXgFuzzy(:,k)=XgFuzzy(:,k)-X(:,k);
    eZ(:,k)=Z(:,k)-x(:,k);
end  
 
%%  ����ƽ��ֵ
MqAve=MqAve+Mq/Nre;
MqFuzzyAve=MqFuzzyAve+MqFuzzy/Nre;
MqFuzzyAve=MqFuzzyAve./(MqFuzzyAve(1,1,:)+MqFuzzyAve(1,2,:)); % ������->����
eXgAve=eXgAve+eXg.^2/Nre;
eXgFuzzyAve=eXgFuzzyAve+eXgFuzzy.^2/Nre;
eZAve=eZAve+eZ.^2/Nre;
end
eXgAve=eXgAve.^0.5;
eXgFuzzyAve=eXgFuzzyAve.^0.5;
eZAve=eZAve.^0.5;
%% %%%%%%%%%%%%%% ��ͼ %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% ������ͼ %%%%%%
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
%%%% ����������� %%%%%%
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

% %%%% ����������� %%%%%%
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



