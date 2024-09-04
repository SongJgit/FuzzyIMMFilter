%% fqiuzhiCoordinateTransformer 球/直坐标变换，
%% 球坐标输入量 zbeitak,zebuxinonk,zDk可以是航迹真值，也可以是测量值

function Zk=fqiuzhiCoordinateTransformer(zbeitak,zebuxinonk,zDk)

% zbeitak     方位角坐标
% zebuxinonk  高低角坐标
% zDk         距离坐标
% Zk          直角坐标向量

Zk(1,1)=zDk*cos(zebuxinonk)*cos(zbeitak);
Zk(2,1)=zDk*cos(zebuxinonk)*sin(zbeitak);
Zk(3,1)=zDk*sin(zebuxinonk);

