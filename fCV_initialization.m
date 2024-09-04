%% fCV_initialization �˲���ʼ������������. CVģ��  

function [F,G,H,Q,Xg2,P2]=fCV_initialization(T,Z1,Z2,R2,q)

% Z1,Z2         k=1,2 ʱ��ֱ�������������
%  F,G,H,Q,Xg2,P2 ����Ϊ ״̬ת�ƾ��������󣬲����󣬹���������
%  k=2 ʱ��״̬����������״̬�������Э������

F=[1 T 0 0 0 0
   0 1 0 0 0 0
   0 0 1 T 0 0
   0 0 0 1 0 0
   0 0 0 0 1 T     
   0 0 0 0 0 1];

G=[1/2*T^2 T  0       0 0 0
   0       0  1/2*T^2 T 0 0
   0       0  0       0 1/2*T^2 T]';

%B=[1/2*T^2 T 1/2*T^2 T 1/2*T^2 T]';

H=[1 0 0 0 0 0
   0 0 1 0 0 0
   0 0 0 0 1 0];

Q=q^2*[1 0 0
       0 1 0
       0 0 1];

zx1=Z1(1,1);
zy1=Z1(2,1);
zz1=Z1(3,1);
zx2=Z2(1,1);
zy2=Z2(2,1);
zz2=Z2(3,1);
Xg2(:,1)=[zx2
         (zx2-zx1)/T
         zy2
         (zy2-zy1)/T
         zz2
         (zz2-zz1)/T];  
P2(:,:)=[R2(1,1)     R2(1,1)/T      R2(1,2)     R2(1,2)/T       R2(1,3)    R2(1,3)/T 
         R2(1,1)/T   2*R2(1,1)/T^2  R2(1,2)/T   2*R2(1,2)/T^2   R2(1,3)/T  2*R2(1,3)/T^2
         R2(1,2)     R2(1,2)/T      R2(2,2)     R2(2,2)/T       R2(2,3)    R2(2,3)/T
         R2(1,2)/T   2*R2(1,2)/T^2  R2(2,2)/T   2*R2(2,2)/T^2   R2(2,3)/T  2*R2(2,3)/T^2
         R2(1,3)     R2(1,3)/T      R2(2,3)     R2(2,3)/T       R2(3,3)    R2(3,3)/T
         R2(1,3)/T   2*R2(1,3)/T^2  R2(2,3)/T   2*R2(2,3)/T^2   R2(3,3)/T  2*R2(3,3)/T^2];     
     