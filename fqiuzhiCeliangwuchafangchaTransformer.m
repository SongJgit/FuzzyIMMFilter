% fqiuzhiCeliangwuchafangchaTransformer ��/ֱ�������Э����任
function Rk=fqiuzhiCeliangwuchafangchaTransformer(zbeitak,zebuxinonk,zDk,rmbeitak,rmebuxinonk,rmDk)

% rmbeitak    ��������λ�ǲ���������׼ƫ�� ģ��ֵ
% rmebuxinonk �������ߵͽǲ���������׼ƫ�� ģ��ֵ
% rmDk        �������������������׼ƫ��   ģ��ֵ

% rmbeitafk    ��������λ�ǲ����������� ģ��ֵ
% rmebuxinonfk �������ߵͽǲ����������� ģ��ֵ
% rmDfk        ���������������������   ģ��ֵ

% Rk          ������ֱ������������������� ģ��ֵ

rmbeitafk=rmbeitak^2; 
rmebuxinonfk=rmebuxinonk^2;
rmDfk=rmDk^2; 

Rk(1,1)=rmDfk*cos(zebuxinonk)^2*cos(zbeitak)^2+zDk^2*cos(zebuxinonk)^2*sin(zbeitak)^2*rmbeitafk... % �������Э������/ֱת��
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


