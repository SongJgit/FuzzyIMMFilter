%  ftracker - kalman filter cv/ca

T=2; %  采样间隔
Tsim=60; % 仿真时间
Nt=Tsim/T;% 仿真总采样点数
n1=1;   % 循环次数
filter=1; % 0－CV,1- CA

q1=1;%20 目标1过程噪声标准偏差 m
q=3;%20 % 滤波器设置的过程噪声标准偏差 m

rd1=150; % 传感器1距离测量噪声实际标准偏差 m
rbeita1=0.9*pi/180;% 传感器1方位角测量噪声实际标准偏差 角度 
rebuxinon1=0.9*pi/180;% 传感器1高低角测量噪声实际标准偏差 角度 

rmd1=150; % 传感器1距离测量噪声模型标准偏差
rmbeita1=0.9*pi/180;% 传感器1方位角测量噪声模型标准偏差
rmebuxinon1=0.9*pi/180;% 传感器1高低角测量噪声模型标准偏差

rmdf1=rmd1^2; % 传感器1距离测量噪声模型方差
rmbeitaf1=rmbeita1^2;% 传感器1方位角测量噪声模型方差
rmebuxinonf1=rmebuxinon1^2; % 传感器1高低角测量噪声模型方差

for n=1:n1
%%%% 航迹生成－直角坐标 %%%%

%pi=3.1415926;
x1=zeros(1,Nt); %  目标1
y1=zeros(1,Nt);
z1=zeros(1,Nt);
Vx=zeros(1,Nt); %  目标1
Vy=zeros(1,Nt);
Vz=zeros(1,Nt);
Ax=zeros(1,Nt); %  目标1
Ay=zeros(1,Nt);
Az=zeros(1,Nt);

x1(1,1)=10000;
y1(1,1)=10000;
z1(1,1)=2000;
Vx(1,1)=-100;
Vy(1,1)=-100;
Vz(1,1)=0;
for k=1:10
    Ax(1,k)=0;
    Ay(1,k)=0;
    Az(1,k)=0;
end
for k=11:18
    Ax(1,k)=-20;
    Ay(1,k)=-10;
    Az(1,k)=0;
end

for k=19:30
    Ax(1,k)=0;
    Ay(1,k)=0;
    Az(1,k)=0;
end
    
for k=1:29 %  匀速
   x1(1,k+1)=x1(1,k)+Vx(1,k)*T+0.5*Ax(1,k)*T^2+normrnd(0,q1);
   y1(1,k+1)=y1(1,k)+Vy(1,k)*T+0.5*Ay(1,k)*T^2+normrnd(0,q1);
   z1(1,k+1)=z1(1,k)+Vz(1,k)*T+0.5*Az(1,k)*T^2+normrnd(0,0.5*q1);
   Vx(1,k+1)=Vx(1,k)+Ax(1,k)*T;
   Vy(1,k+1)=Vy(1,k)+Ay(1,k)*T;
   Vz(1,k+1)=Vz(1,k)+Az(1,k)*T;
end


%%%% 量测生成 %%%%%%%%%%
%%  直/球坐标变换
beita1=zeros(1,Nt);
ebuxinon1=zeros(1,Nt);
D1=zeros(1,Nt);
beita1c=zeros(1,Nt);
ebuxinon1c=zeros(1,Nt);
D1c=zeros(1,Nt);

for k=1:Nt % 航迹真值生成—球坐标
     beita1(1,k)=atan2(y1(1,k),x1(1,k)); % atan2四象限反正切
     ebuxinon1(1,k)=atan2(z1(1,k),(x1(1,k)^2+y1(1,k)^2)^0.5);
     D1(1,k)=(x1(1,k)^2+y1(1,k)^2+z1(1,k)^2)^0.5;
end
for k=1:Nt % 测量噪声—球坐标
   beita1c(1,k)=beita1(1,k)+normrnd(0,rbeita1); 
   ebuxinon1c(1,k)=ebuxinon1(1,k)+normrnd(0,rebuxinon1);
   D1c(1,k)=D1(1,k)+normrnd(0,rd1);
end 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 传感器1滤波%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 球/直坐标变换
zx1=zeros(1,Nt);
zy1=zeros(1,Nt);
zz1=zeros(1,Nt);
R1=zeros(3,3,Nt);
R2=zeros(3,3,Nt);

for k=1:Nt % 测量量球/直坐标变换
    zx1(1,k)=D1c(1,k)*cos(ebuxinon1c(1,k))*cos(beita1c(1,k));
    zy1(1,k)=D1c(1,k)*cos(ebuxinon1c(1,k))*sin(beita1c(1,k));
    zz1(1,k)=D1c(1,k)*sin(ebuxinon1c(1,k));
end
Z1=[zx1
    zy1
    zz1];

for k=1:Nt% 测量误差协方差球/直转换
    R1(1,1,k)=rmdf1*cos(ebuxinon1c(1,k))^2*cos(beita1c(1,k))^2+D1c(1,k)^2*cos(ebuxinon1c(1,k))^2*sin(beita1c(1,k))^2*rmbeitaf1... % 测量误差协方差球/直转换
        +D1c(1,k)^2*sin(ebuxinon1c(1,k))^2*cos(beita1c(1,k))^2*rmebuxinonf1;
    R1(2,2,k)=rmdf1*cos(ebuxinon1c(1,k))^2*sin(beita1c(1,k))^2+D1c(1,k)^2*cos(ebuxinon1c(1,k))^2*cos(beita1c(1,k))^2*rmbeitaf1...
        +D1c(1,k)^2*sin(ebuxinon1c(1,k))^2*cos(beita1c(1,k))^2*rmebuxinonf1;
    R1(3,3,k)=rmdf1*sin(ebuxinon1c(1,k))^2+D1c(1,k)^2*cos(ebuxinon1c(1,k))^2*rmebuxinonf1;
    R1(1,2,k)=0.5*sin(2*beita1c(1,k))*(rmdf1*cos(ebuxinon1c(1,k))^2-D1c(1,k)^2*cos(ebuxinon1c(1,k))^2*rmbeitaf1...
        +D1c(1,k)^2*sin(ebuxinon1c(1,k))^2*rmebuxinonf1);
    R1(2,3,k)=0.5*sin(2*ebuxinon1c(1,k))*(rmdf1-D1c(1,k)^2*rmebuxinonf1)*sin(beita1c(1,k));
    R1(1,3,k)=0.5*sin(2*ebuxinon1c(1,k))*(rmdf1-D1c(1,k)^2*rmebuxinonf1)*cos(beita1c(1,k));
    R1(2,1,k)=R1(1,2,k)';
    R1(3,2,k)=R1(2,3,k)';
    R1(3,1,k)=R1(1,3,k)';
end

%% 滤波初始化及参数设置%%%
if filter==0 % % 0－CV,1- CA
%% -CV- %%%
%% 滤波初始化及参数设置%%%
Xg1=zeros(6,Nt);
P1=zeros(6,6,Nt);

F1=[1 T 0 0 0 0
    0 1 0 0 0 0
    0 0 1 T 0 0
    0 0 0 1 0 0
    0 0 0 0 1 T
    0 0 0 0 0 1];
G1=[1/2*T^2 T   0       0 0       0
    0       0   1/2*T^2 T 0       0
    0       0   0       0 1/2*T^2 T]';
B1=[1/2*T^2 T 1/2*T^2 T 1/2*T^2 T]';
H1=[1 0 0 0 0 0
    0 0 1 0 0 0
    0 0 0 0 1 0];
Q1=q^2*[1 0 0
        0 1 0
        0 0 1];

Xg1(:,2)=[zx1(1,2)
          (zx1(1,2)-zx1(1,1))/T
          zy1(1,2)
          (zy1(1,2)-zy1(1,1))/T
          zz1(1,2)
          (zz1(1,2)-zz1(1,1))/T];    
P1(:,:,2)=[R1(1,1,2)     R1(1,1,2)/T      R1(1,2,2)     R1(1,2,2)/T       R1(1,3,2)    R1(1,3,2)/T 
           R1(1,1,2)/T   2*R1(1,1,2)/T^2  R1(1,2,2)/T   2*R1(1,2,2)/T^2   R1(1,3,2)/T  2*R1(1,3,2)/T^2
           R1(1,2,2)     R1(1,2,2)/T      R1(2,2,2)     R1(2,2,2)/T       R1(2,3,2)    R1(2,3,2)/T
           R1(1,2,2)/T   2*R1(1,2,2)/T^2  R1(2,2,2)/T   2*R1(2,2,2)/T^2   R1(2,3,2)/T  2*R1(2,3,2)/T^2
           R1(1,3,2)     R1(1,3,2)/T      R1(2,3,2)     R1(2,3,2)/T       R1(3,3,2)    R1(3,3,2)/T
           R1(1,3,2)/T   2*R1(1,3,2)/T^2  R1(2,3,2)/T   2*R1(2,3,2)/T^2   R1(3,3,2)/T  2*R1(3,3,2)/T^2];
%%%%%%% CV kalmnf %%%%%%
Xy1=zeros(6,Nt);
Zy1=zeros(3,Nt);
v1=zeros(3,Nt);
Py1=zeros(6,6,Nt);
S1=zeros(3,3,Nt);
K1=zeros(6,3,Nt);

for k=2:(Nt-1)
   Xy1(:,k+1)=F1*Xg1(:,k);
   Zy1(:,k+1)=H1*Xy1(:,k+1);
   v1(:,k+1)=Z1(:,k+1)-Zy1(:,k+1);
   Py1(:,:,k+1)=F1*P1(:,:,k)*F1'+G1*Q1*G1';
   S1(:,:,k+1)=H1*Py1(:,:,k+1)*H1'+R1(:,:,k+1);
   K1(:,:,k+1)=Py1(:,:,k+1)*H1'*inv(S1(:,:,k+1));
   P1(:,:,k+1)=Py1(:,:,k+1)-K1(:,:,k+1)*S1(:,:,k+1)*K1(:,:,k+1)';
   Xg1(:,k+1)=Xy1(:,k+1)+K1(:,:,k+1)*v1(:,k+1);
end

else  
%% -CA- %%%
%% 滤波初始化及参数设置%%%
Xg1=zeros(9,Nt);
P1=zeros(9,9,Nt);

F1=[1 T 1/2*T^2 0 0 0 0 0 0
    0 1 T       0 0 0 0 0 0
    0 0 1       0 0 0 0 0 0
    0 0 0 1 T 1/2*T^2 0 0 0
    0 0 0 0 1 T       0 0 0
    0 0 0 0 0 1       0 0 0
    0 0 0 0 0 0 1 T 1/2*T^2
    0 0 0 0 0 0 0 1 T
    0 0 0 0 0 0 0 0 1];
G1=[1/2*T^2 T 1 0 0 0 0 0 0
    0 0 0 1/2*T^2 T 1 0 0 0
    0 0 0 0 0 0 1/2*T^2 T 1]';
%B1=[1/2*T^2 T 1/2*T^2 T 1/2*T^2 T]';
H1=[1 0 0 0 0 0 0 0 0
    0 0 0 1 0 0 0 0 0
    0 0 0 0 0 0 1 0 0];
Q1=q^2*[1 0 0
        0 1 0
        0 0 1];

Xg1(:,3)=[zx1(1,3)
          (zx1(1,3)-zx1(1,2))/T
          (zx1(1,3)-2*zx1(1,2)+zx1(1,1))/T^2
          zy1(1,3)
          (zy1(1,3)-zy1(1,2))/T
          (zy1(1,3)-2*zy1(1,2)+zy1(1,1))/T^2
          zz1(1,3)
          (zz1(1,3)-zz1(1,2))/T
          (zz1(1,3)-2*zz1(1,2)+zz1(1,1))/T^2];    
%P1(:,:,2)=[R1(1,1,2)     R1(1,1,2)/T      R1(1,2,2)     R1(1,2,2)/T       R1(1,3,2)    R1(1,3,2)/T 
%           R1(1,1,2)/T   2*R1(1,1,2)/T^2  R1(1,2,2)/T   2*R1(1,2,2)/T^2   R1(1,3,2)/T  2*R1(1,3,2)/T^2
%           R1(1,2,2)     R1(1,2,2)/T      R1(2,2,2)     R1(2,2,2)/T       R1(2,3,2)    R1(2,3,2)/T
%           R1(1,2,2)/T   2*R1(1,2,2)/T^2  R1(2,2,2)/T   2*R1(2,2,2)/T^2   R1(2,3,2)/T  2*R1(2,3,2)/T^2
%           R1(1,3,2)     R1(1,3,2)/T      R1(2,3,2)     R1(2,3,2)/T       R1(3,3,2)    R1(3,3,2)/T
%           R1(1,3,2)/T   2*R1(1,3,2)/T^2  R1(2,3,2)/T   2*R1(2,3,2)/T^2   R1(3,3,2)/T  2*R1(3,3,2)/T^2];
P1(:,:,3)=[R1(1,1,3)     R1(1,1,3)/T      R1(1,1,3)/T^2    R1(1,2,3)       R1(1,2,3)/T      R1(1,2,3)/T^2    R1(1,3,3)      R1(1,3,3)/T      R1(1,3,3)/T^2
           R1(1,1,3)/T   2*R1(1,1,3)/T^2  3*R1(1,1,3)/T^3  R1(1,2,3)/T     2*R1(1,2,3)/T^2  3*R1(1,2,3)/T^3  R1(1,3,3)/T    2*R1(1,3,3)/T^2  3*R1(1,3,3)/T^3
           R1(1,1,3)/T^2 3*R1(1,1,3)/T^3  6*R1(1,1,3)/T^4  R1(1,2,3)/T^2   3*R1(1,2,3)/T^3  6*R1(1,2,3)/T^4  R1(1,3,3)/T^2  3*R1(1,3,3)/T^3  6*R1(1,3,3)/T^4
           R1(1,2,3)     R1(1,2,3)/T      R1(1,2,3)/T^2    R1(2,2,3)       R1(2,2,3)/T      R1(2,2,3)/T^2    R1(2,3,3)      R1(2,3,3)/T      R1(2,3,3)/T^2 
           R1(1,2,3)/T   2*R1(1,2,3)/T^2  3*R1(1,2,3)/T^3  R1(2,2,3)/T     2*R1(2,2,3)/T^2  3*R1(2,2,3)/T^3  R1(2,3,3)/T    2*R1(2,3,3)/T^2  3*R1(2,3,3)/T^3  
           R1(1,2,3)/T^2 3*R1(1,2,3)/T^3  6*R1(1,2,3)/T^4  R1(2,2,3)/T^2   3*R1(2,2,3)/T^3  6*R1(2,2,3)/T^4  R1(2,3,3)/T^2  3*R1(2,3,3)/T^3  6*R1(2,3,3)/T^4
           R1(1,3,3)     R1(1,3,3)/T      R1(1,3,3)/T^2    R1(2,3,3)       R1(2,3,3)/T      R1(2,3,3)/T^2    R1(3,3,3)      R1(3,3,3)/T      R1(3,3,3)/T^2 
           R1(1,3,3)/T   2*R1(1,3,3)/T^2  3*R1(1,3,3)/T^3  R1(2,3,3)/T     2*R1(2,3,3)/T^2  3*R1(2,3,3)/T^3  R1(3,3,3)/T    2*R1(3,3,3)/T^2  3*R1(3,3,3)/T^3
           R1(1,3,3)/T^2 3*R1(1,3,3)/T^3  6*R1(1,3,3)/T^4  R1(2,3,3)/T^2   3*R1(2,3,3)/T^3  6*R1(2,3,3)/T^4  R1(3,3,3)/T^2  3*R1(3,3,3)/T^3  6*R1(3,3,3)/T^4];
           
%%%%%%% CA kalmnf %%%%%%
Xy1=zeros(9,Nt);
Zy1=zeros(3,Nt);
v1=zeros(3,Nt);
Py1=zeros(9,9,Nt);
S1=zeros(3,3,Nt);
K1=zeros(9,3,Nt);

for k=3:(Nt-1)
   Xy1(:,k+1)=F1*Xg1(:,k);
   Zy1(:,k+1)=H1*Xy1(:,k+1);
   v1(:,k+1)=Z1(:,k+1)-Zy1(:,k+1);
   Py1(:,:,k+1)=F1*P1(:,:,k)*F1'+G1*Q1*G1';
   S1(:,:,k+1)=H1*Py1(:,:,k+1)*H1'+R1(:,:,k+1);
   K1(:,:,k+1)=Py1(:,:,k+1)*H1'*inv(S1(:,:,k+1));
   P1(:,:,k+1)=Py1(:,:,k+1)-K1(:,:,k+1)*S1(:,:,k+1)*K1(:,:,k+1)';
   Xg1(:,k+1)=Xy1(:,k+1)+K1(:,:,k+1)*v1(:,k+1);
end

end % for filter=CV

end  % for n=1:n1

%%%%%%%%%%%%%%%% 绘图 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% 航迹绘图 %%%%%%
trackplot=0;
if trackplot==1
k=1:Nt;
figure(20);
plot(k,x1);
hold on;
plot(k,zx1,'y');
hold off;

figure(21);
plot(k,y1);
hold on;
plot(k,zy1,'y');
hold off;

figure(22);
plot(k,z1);
hold on;
plot(k,zz1,'y');
hold off;

figure(23);
plot(x1,y1);
hold on;
plot(zx1,zy1,'y');
hold off;
end
%%%% 绘制误差曲线 %%%%%%
if filter==0
k=2:Nt;
figure (1)
plot(k,Xg1(1,k)-x1(1,k))
title ('egx1-b/elx1-r')
grid
hold on
plot(k,Z1(1,k)-x1(1,k),'r')
hold off

figure (2)
plot(k,Xg1(3,k)-y1(1,k))
title ('egy1-b/ely1-r')
grid
hold on
plot(k,Z1(2,k)-y1(1,k),'r')
hold off

figure (3)
plot(k,Xg1(5,k)-z1(1,k))
title ('egz1-b/elz1-r')
grid
hold on
plot(k,Z1(3,k)-z1(1,k),'r')
hold off

figure (4)
plot(k,Xg1(2,k))
title ('Vgx-b')
grid
hold off

figure (5)
plot(k,Xg1(4,k))
title ('Vgy-b')
grid
hold off

figure (6)
plot(k,Xg1(6,k))
title ('Vgz-b')
grid
hold off

else
k=3:Nt;
figure (11)
plot(k,Xg1(1,k)-x1(1,k))
title ('egx1-b/elx1-r')
grid
hold on
plot(k,Z1(1,k)-x1(1,k),'r')
hold off

figure (21)
plot(k,Xg1(4,k)-y1(1,k))
title ('egy1-b/ely1-r')
grid
hold on
plot(k,Z1(2,k)-y1(1,k),'r')
hold off

figure (31)
plot(k,Xg1(7,k)-z1(1,k))
title ('egz1-b/elz1-r')
grid
hold on
plot(k,Z1(3,k)-z1(1,k),'r')
hold off

figure (41)
plot(k,Xg1(2,k))
title ('Vgx-b')
grid
hold off

figure (51)
plot(k,Xg1(5,k))
title ('Vgy-b')
grid
hold off

figure (61)
plot(k,Xg1(8,k))
title ('Vgz-b')
grid
hold off

figure (71)
plot(k,Xg1(3,k))
title ('Agx-b')
grid
hold off

figure (81)
plot(k,Xg1(6,k))
title ('Agy-b')
grid
hold off

figure (91)
plot(k,Xg1(9,k))
title ('Agz-b')
grid
hold off

end % if filter==0


    
