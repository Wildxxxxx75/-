%���˶�ѧ��⣬�����ƻ�����
close all;
clear all;

global d5 %�ֶ�����ĩ�˹ؽڳ���d5
global A_x A_y A_z %�Ƕȹ滮����ʼ������
global p_x p_y p_z %ĩ�˹滮·������
global q_1 q_2 q_3 q_4 q_5 q_6 q_7
q_1=0;q_2=0;q_3=0;q_4=0;q_5=0;q_6=0;q_7=0;
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
%%����ֵ
d5=170;
A_x=0;A_y=-600;A_z=-600;
%���ýǶȹ滮��A�㵽B�������
% B_x=[0  0];
% B_y=[-500  -1000];
% B_z=[-500  -1000];
%����10С��

ss=[];%�洢С����Ϣ�ľ���
%ellipsoid(x,y,z,x1,y1,z1) x,y,z���������������,x1,y1,z1������x,y,z����ķ���
for ii=1:1:10
   BanJing=randi([25 40]);%����һ��25��40֮��������������Բ�İ뾶
%  j=unidrnd(50);%����һ��50���ڵ������
   
   s1=unidrnd(50);%s���������С��ĳ�ʼ��
   s2=randi([0 50]);%s���������С��ĳ�ʼ��
   s3=randi([0 57]);%s���������С��ĳ�ʼ��
%    s4=randsrc();%�����������1
   %�������С��λ�ã�����ÿ��С��Բ������a,b,c�������60����֤ÿ����С������С��200
   aa=s1+100+ii*60;
%     aa=0;
%    bb=s4*s2+ii*60;
%    ccc=s4*s3+ii*60;
    bb=s2+ii*60;
    pp=s3+ii*60;
   [xx,yy,zz]= ellipsoid(aa,bb,pp,BanJing,BanJing,BanJing);
   surf(xx,yy,zz) %�뾶Ϊj��Բ
%    shading interp%��ֹsurf������ɫС��
   dd=(4/3)*pi*BanJing^3;%������
   ti_ji=round(dd);%�����ȡ��
   ss=[ss;aa,bb,pp,ti_ji,BanJing];
%    yan_se=ones(size(zz,1));
  
   disp(ss)%��ӡ��ÿ��С���Բ����������
   hold on
%    view(-8,38);%����ڣ�-8 ��38���Ƕȹۿ�
   alpha(0.1)%����ϵ������ͼƬ͸����Ϊ0.5��1�ǲ�͸�� 
end

AA=[ss(1,4),ss(2,4),ss(3,4),ss(4,4),ss(5,4),ss(6,4),ss(7,4),ss(8,4),ss(9,4),ss(10,4)];%��ÿ����������ȡ������A��
[pai_xu,index]= sort(AA,'descend'); %�Ծ���A�е�������Ӵ�С���У���д��ԭ���ھ���A�еڼ��У�����index�����У�Ҳ�����ھ���ss�еڼ���
%�ϱ�����Ѿ��Ӵ�С�źã������ǵڼ������ɵ���Ҳ��index��ʾ����
%�±߾͸���text���������ζԵڼ������ɵ��򣬰��������С������
for m=1:1:10
   xu_hao = num2str(m);%��ֵת�ַ�����׼��ѭ����1��10��10����
   text(ss(index(1,m),1),ss(index(1,m),2),ss(index(1,m),3),xu_hao,'Color','red','FontSize',14);%�����ţ�ԭ��ss�����е�m�е�1���������꣬
   %ss�����е�m�е�2����������Ȼ������š�
end

%%%ĩ�˹滮·�������ʼ������0
for i=1:1:length(p_x)
    p_x(i)=0;
    p_y(i)=0;
    p_z(i)=0;
end
%%%�ǶȻ���ת��
ToDeg = 180/pi;
ToRad = pi/180;
%���ó�ʼ�ؽ�ת���ĽǶ�
th1=90;%Link(2).th
th2=0;%Link(3).th
th3=0;%Link(4).th
th4=0;%Link(5).th
th5=0;%Link(6).th
th6=-90;%Link(7).th
th7=d5;%Link(8).th
grid on;
%%%���ƻ����˳�ʼ״̬
DHfk7Dof_Lnya(th1,th2,th3,th4,th5,th6,th7,0);%0Ϊ������αӰ��1����
view(64,20);
hold on
pause;
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);

for xun_huan=1:1:10


shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
rrt_SanWei(ss,index,xun_huan,ss(index(1,xun_huan),5));

% ֱ������ϵĩ��·���滮
% ��ζ���ʽֱ��ĩ�˹滮
Draw_line_5(ss(index(1,xun_huan),3),ss(index(1,xun_huan),2),100,3,ss(index(1,xun_huan),5),ss,index,xun_huan,ss(index(1,xun_huan),1),0);%x�᷽�����ƶ�
Draw_line_5(ss(index(1,xun_huan),3),ss(index(1,xun_huan),2),100,1,ss(index(1,xun_huan),5),ss,index,xun_huan,ss(index(1,xun_huan),1),0);%y�᷽�����ƶ�
% %ĩ��4-3-4����ĩ�˹滮
hold on
Draw_curve_434(ss(index(1,xun_huan),3),-400,-500,-600,0,0,0,0,2,4,ss(index(1,xun_huan),5),ss,index,xun_huan);%�滮z���
ss(index(1,xun_huan),:)=[0, 0 ,0, 0 ,0 ];%ɾ������ss��ĳһ��,���0 0 0 0 0% % %�Ƕȹ켣�滮
fan_hui=xun_huan+1;%����Ŀ���Ϊ��һ��ѭ����С��
if xun_huan<10
B_x=ss(index(1,fan_hui),1);
B_y=ss(index(1,fan_hui),2);
B_z=ss(index(1,fan_hui),3);
Draw_angle_434(B_x,B_y,B_z,ss,xun_huan,index);%%�Ƕ�434�ռ�滮 ��A��B����Ŀ���A����һ��С��켣
else
end
end
% % 