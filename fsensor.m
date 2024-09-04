%% fsensor ������������������

function [zbeitak,zebuxinonk,zDk]=fsensor(beitak,ebuxinonk,Dk,rbeitak,rebuxinonk,rDk)   

% beitak     ��λ������
% ebuxinonk  �ߵͽ�����
% Dk         ��������

% rbeitak    ��������λ�ǲ���������׼ƫ��
% rebuxinonk �������ߵͽǲ���������׼ƫ��
% rDk        �������������������׼ƫ��

zbeitak=beitak+normrnd(0,rbeitak); 
zebuxinonk=ebuxinonk+normrnd(0,rebuxinonk);
zDk=Dk+normrnd(0,rDk);