%%%%%%% fFuzzyIMM_filter --- FuzzyIMM filter based CV/CA models %%%%%%
function [Xgqk,Pqk,Mqk_1,Lk,Sk,vk]=fFuzzyIMM_filter(Zk,Xgqk_1,Pqk_1,F_CV,G_CV,H_CV,F_CA,G_CA,H_CA,q,Rk,Ptc,Mqk_2)

%% IMM �˲���ʼ������������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PI=3.1415926;

Ptrk_1=zeros(2,2);   % ת��ģ��ת�Ƹ��ʾ���
Mhk_1=zeros(1,2);    % ���飨������ģ�͸���1(CV),2(CA),���е�2ά�����ĸ�ģ��
Xghk_1=zeros(9,2);   % ����ǰ״̬����,���е�2ά�����ĸ�ģ��
DP=zeros(9,9,2,2);   % Э��������,���е�3ά�͵�4ά�����ĸ�ģ��
Phk_1=zeros(9,9,2);  % ������״̬Э����,���е�3ά�����ĸ�ģ��
Lk=zeros(1,2);       % ģ����Ȼ1(CV),2(CA),���е�2ά�����ĸ�ģ��
% Ptc=[1   1/4   % ת��ģ��ת�ƿ����Ծ���
%      1/4   1];
      
Xgqk=zeros(9,2);
Pqk=zeros(9,9,2);
vk=zeros(3,2);
Sk=zeros(3,3,2);
    % ģ�ͽ���
    for j=1:2
        Mhk_1(1,j)=max(Ptc(1,j)*Mqk_2(1,1),Ptc(2,j)*Mqk_2(1,2)); % �������飨������ģ�͸���1,2
    end
    for j=1:2
        for i=1:2
            Ptrk_1(i,j)=Ptc(j,i)*Mqk_2(1,j)/Mhk_1(1,i); % ����ת��ģ��ת�Ƹ���
        end
    end   
    for j=1:2
        d=Ptrk_1(j,2)/Ptrk_1(j,1);
%       d=Mhk_1(1,2)/Mhk_1(1,1);
        if d>=1
             Xghk_1(:,j)=Xgqk_1(:,2);% ���㽻����״̬����
        else
             Xghk_1(:,j)=Xgqk_1(:,1);% ���㽻����״̬����           
        end
    end
    for j=1:2
        for i=1:2
            DP(:,:,i,j)=(Xgqk_1(:,i)-Xghk_1(:,j))*(Xgqk_1(:,i)-Xghk_1(:,j))'; % ����ʽһ����㽻����Э����
        end
        Phk_1(:,:,j)=Pqk_1(:,:,j)+DP(:,:,1,j);  % ���㽻����Э����
    end

    %%%% Kalman filter, 1(CV),2(CA)
    for i=2:-1:1  % CA first, then CV
        if i==2 % CA model
            Xghk_1i=Xghk_1(:,i);
            Phk_1i=Phk_1(:,:,i);
            [Xgqki,Pqki,vki,Ski]=fCA_KF(Zk,Xghk_1i,Phk_1i,F_CA,G_CA,H_CA,q,Rk);% һ���˲�ѭ���󽻻���״̬���Ʊ�ɽ���ǰ״̬����
            Xgqk(:,i)=Xgqki;
            Pqk(:,:,i)=Pqki;
            vk(:,i)=vki;
            Sk(:,:,i)=Ski;
        end
        if i==1 % CV model
            tempX=Xghk_1(:,i);
            Xghk_1i=[tempX([1 2],1);tempX([4 5],1);tempX([7 8],1)]; % ��9*1������ȡ��6�У�����6*1����
            
            tempP=Phk_1(:,:,i);
            tempP1=tempP([1 2],:);             % �ǶԽ���ά����
            tempP2=tempP([4 5],:);
            tempP3=tempP([7 8],:);
            tempPP=[tempP1;tempP2;tempP3];     % 6*9����
            tempPP1=tempPP(:,[1 2]);           
            tempPP2=tempPP(:,[4 5]);           
            tempPP3=tempPP(:,[7 8]);
            Phk_1i=[tempPP1,tempPP2,tempPP3];  % �ǶԽ���ά���� 6*6����
%             tempP1=tempP([1 2],[1 2]);             % ����Խ���ά���������ɿ��ǷǶԽ���ά����
%             tempP2=tempP([4 5],[4 5]);
%             tempP3=tempP([7 8],[7 8]);
%             Phk_1i=blkdiag(tempP1,tempP2,tempP3);  % ����Խ���ά���������ɿ��ǷǶԽ���ά����
            
            [Xgqki,Pqki,vki,Ski]=fCV_KF(Zk,Xghk_1i,Phk_1i,F_CV,G_CV,H_CV,q,Rk);% һ���˲�ѭ���󽻻���״̬���Ʊ�ɽ���ǰ״̬����
            
            % expand Xgqki from 6*1 into 9*1
            Expand_Xgqki_x=padarray(Xgqki([1 2],1),[1,0],'post');
            Expand_Xgqki_y=padarray(Xgqki([3 4],1),[1,0],'post');
            Expand_Xgqki_z=padarray(Xgqki([5 6],1),[1,0],'post');
            Expand_Xgqki=[Expand_Xgqki_x;Expand_Xgqki_y;Expand_Xgqki_z];          
            Xgqk(:,i)=Expand_Xgqki;

            % expand Pqki from 6*6 into 9*9
            Expand_P_1=padarray(Pqki([1 2],:),[1,0],'post');
            Expand_P_2=padarray(Pqki([3 4],:),[1,0],'post');
            Expand_P_3=padarray(Pqki([5 6],:),[1,0],'post');
            Expand_P=[Expand_P_1;Expand_P_2;Expand_P_3];
            Expand_P_4=padarray(Expand_P(:,[1 2]),[0,1],'post');
            Expand_P_5=padarray(Expand_P(:,[3 4]),[0,1],'post');
            Expand_P_6=padarray(Expand_P(:,[5 6]),[0,1],'post');
            Expand_PP=[Expand_P_4 Expand_P_5 Expand_P_6];
            Pqk(:,:,i)=Expand_PP;
            Pqk(3,3,i)=Pqk(3,3,2); % �� CAģ���µļ��ٶȹ������ֵ��CVģ��
            Pqk(6,6,i)=Pqk(6,6,2);  
            Pqk(9,9,i)=Pqk(9,9,2); 
%             Pqk(3,3,i)=Xgqk(3,2)^2; % �� CAģ�ͼ��ٶȵ�ƽ����ΪCVģ�͵ļ��ٶȹ�������
%             Pqk(6,6,i)=Xgqk(6,2)^2; 
%             Pqk(9,9,i)=Xgqk(9,2)^2;
            
            vk(:,i)=vki;
            Sk(:,:,i)=Ski;
        end
    end
    %%% ������飨����ǰ��ģ�͸���
    for i=1:2
        Lk(1,i)=exp(-vk(:,i)'*inv(Sk(:,:,i))*vk(:,i)/2)/(((2*PI)^3*det(Sk(:,:,i)))^0.5); % ����ģ����Ȼ
    end
    for i=1:2
         Mqk_1(1,i)=Lk(1,i)*Mhk_1(1,i)/max(Lk(1,1)*Mhk_1(1,1),Lk(1,2)*Mhk_1(1,2)); % ������飨����ǰ��ģ�͸���
%        Mqk_1(1,i)=Lk(1,i)/max(Lk(1,1),Lk(1,2)); % ������飨����ǰ��ģ�͸���
    end
    %%% ȫ��ģ�ͼ�Ȩ���
    % Xgk(:,1)=Xgqk(:,1)*Mqk_1(1,1)+Xgqk(:,2)*Mqk_1(1,2); 
    % Pk(:,:)=Pqk(:,:,1)*Mqk_1(1,1)+Pqk(:,:,2)*Mqk_1(1,2);


