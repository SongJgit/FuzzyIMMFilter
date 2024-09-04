%%%%%%% fCV_KF --- CV model based kalman filter  %%%%%%
function [Xgk,Pk,vk,Sk]=fCV_KF(Zk,Xgk_1,Pk_1,F,G,H,q,Rk)

Xyk=zeros(6,1);
Zyk=zeros(3,1);
vk=zeros(3,1);
Pyk=zeros(6,6);
Sk=zeros(3,3);
Kk=zeros(6,3);

Xyk(:,1)=F*Xgk_1(:,1);
Zyk(:,1)=H*Xyk(:,1);
vk(:,1)=Zk(:,1)-Zyk(:,1);
Pyk(:,:)=F*Pk_1(:,:)*F'+G*G'*q^2; % G*Q*G';
Sk(:,:)=H*Pyk(:,:)*H'+Rk(:,:);
Kk(:,:)=Pyk(:,:)*H'*inv(Sk(:,:));
Pk(:,:)=Pyk(:,:)-Kk(:,:)*Sk(:,:)*Kk(:,:)';
Xgk(:,1)=Xyk(:,1)+Kk(:,:)*vk(:,1);

