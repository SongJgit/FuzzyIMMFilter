%% fqiuzhiCoordinateTransformer ��/ֱ����任��
%% ������������ zbeitak,zebuxinonk,zDk�����Ǻ�����ֵ��Ҳ�����ǲ���ֵ

function Zk=fqiuzhiCoordinateTransformer(zbeitak,zebuxinonk,zDk)

% zbeitak     ��λ������
% zebuxinonk  �ߵͽ�����
% zDk         ��������
% Zk          ֱ����������

Zk(1,1)=zDk*cos(zebuxinonk)*cos(zbeitak);
Zk(2,1)=zDk*cos(zebuxinonk)*sin(zbeitak);
Zk(3,1)=zDk*sin(zebuxinonk);

