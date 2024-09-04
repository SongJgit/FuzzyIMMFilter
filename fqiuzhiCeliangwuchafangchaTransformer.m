% fqiuzhiCeliangwuchafangchaTransformer 球/直测量误差协方差变换
function Rk=fqiuzhiCeliangwuchafangchaTransformer(zbeitak,zebuxinonk,zDk,rmbeitak,rmebuxinonk,rmDk)

% rmbeitak    传感器方位角测量噪声标准偏差 模型值
% rmebuxinonk 传感器高低角测量噪声标准偏差 模型值
% rmDk        传感器距离测量噪声标准偏差   模型值

% rmbeitafk    传感器方位角测量噪声方差 模型值
% rmebuxinonfk 传感器高低角测量噪声方差 模型值
% rmDfk        传感器距离测量噪声方差   模型值

% Rk          传感器直角坐标测量噪声方差阵 模型值

rmbeitafk=rmbeitak^2; 
rmebuxinonfk=rmebuxinonk^2;
rmDfk=rmDk^2; 

Rk(1,1)=rmDfk*cos(zebuxinonk)^2*cos(zbeitak)^2+zDk^2*cos(zebuxinonk)^2*sin(zbeitak)^2*rmbeitafk... % 测量误差协方差球/直转换
    +zDk^2*sin(zebuxinonk)^2*cos(zbeitak)^2*rmebuxinonfk;
Rk(2,2)=rmDfk*cos(zebuxinonk)^2*sin(zbeitak)^2+zDk^2*cos(zebuxinonk)^2*cos(zbeitak)^2*rmbeitafk...
    +zDk^2*sin(zebuxinonk)^2*cos(zbeitak)^2*rmebuxinonfk;
Rk(3,3)=rmDfk*sin(zebuxinonk)^2+zDk^2*cos(zebuxinonk)^2*rmebuxinonfk;
Rk(1,2)=0.5*sin(2*zbeitak)*(rmDfk*cos(zebuxinonk)^2-zDk^2*cos(zebuxinonk)^2*rmbeitafk...
    +zDk^2*sin(zebuxinonk)^2*rmebuxinonfk);
Rk(2,3)=0.5*sin(2*zebuxinonk)*(rmDfk-zDk^2*rmebuxinonfk)*sin(zbeitak);
Rk(1,3)=0.5*sin(2*zebuxinonk)*(rmDfk-zDk^2*rmebuxinonfk)*cos(zbeitak);
Rk(2,1)=Rk(1,2)';
Rk(3,2)=Rk(2,3)';
Rk(3,1)=Rk(1,3)';


