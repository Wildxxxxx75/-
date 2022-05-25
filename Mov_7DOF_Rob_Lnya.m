%正运动学求解，并绘制机器人
close all;
clear all;

global d5 %手动控制末端关节长度d5
global A_x A_y A_z %角度规划的起始点坐标
global p_x p_y p_z %末端规划路径坐标
global q_1 q_2 q_3 q_4 q_5 q_6 q_7
q_1=0;q_2=0;q_3=0;q_4=0;q_5=0;q_6=0;q_7=0;
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
%%赋初值
d5=170;
A_x=0;A_y=-600;A_z=-600;
%设置角度规划的A点到B点的坐标
% B_x=[0  0];
% B_y=[-500  -1000];
% B_z=[-500  -1000];
%生成10小球

ss=[];%存储小球信息的矩阵
%ellipsoid(x,y,z,x1,y1,z1) x,y,z代表了椭球的中心,x1,y1,z1代表了x,y,z方向的分量
for ii=1:1:10
   BanJing=randi([25 40]);%产生一个25到40之间的随机数，当作圆的半径
%  j=unidrnd(50);%产生一个50以内的随机数
   
   s1=unidrnd(50);%s是随机生成小球的初始点
   s2=randi([0 50]);%s是随机生成小球的初始点
   s3=randi([0 57]);%s是随机生成小球的初始点
%    s4=randsrc();%随机生成正负1
   %随机生成小球位置，并且每个小球圆心坐标a,b,c至少相差60，保证每两个小球间相距小于200
   aa=s1+100+ii*60;
%     aa=0;
%    bb=s4*s2+ii*60;
%    ccc=s4*s3+ii*60;
    bb=s2+ii*60;
    pp=s3+ii*60;
   [xx,yy,zz]= ellipsoid(aa,bb,pp,BanJing,BanJing,BanJing);
   surf(xx,yy,zz) %半径为j的圆
%    shading interp%防止surf画出黑色小球
   dd=(4/3)*pi*BanJing^3;%球的体积
   ti_ji=round(dd);%对体积取整
   ss=[ss;aa,bb,pp,ti_ji,BanJing];
%    yan_se=ones(size(zz,1));
  
   disp(ss)%打印出每个小球的圆心坐标和体积
   hold on
%    view(-8,38);%最后在（-8 ，38）角度观看
   alpha(0.1)%坐标系内所有图片透明度为0.5，1是不透明 
end

AA=[ss(1,4),ss(2,4),ss(3,4),ss(4,4),ss(5,4),ss(6,4),ss(7,4),ss(8,4),ss(9,4),ss(10,4)];%将每个球的体积读取到矩阵A中
[pai_xu,index]= sort(AA,'descend'); %对矩阵A中的体积按从大到小排列，并写出原来在矩阵A中第几列，存入index矩阵中，也就是在矩阵ss中第几行
%上边体积已经从大到小排好，并且是第几次生成的球也用index表示出来
%下边就根据text函数，依次对第几次生成的球，按照体积大小标记序号
for m=1:1:10
   xu_hao = num2str(m);%数值转字符串，准备循环从1到10，10个数
   text(ss(index(1,m),1),ss(index(1,m),2),ss(index(1,m),3),xu_hao,'Color','red','FontSize',14);%标记序号，原来ss矩阵中第m行第1列做横坐标，
   %ss矩阵中第m行第2列做纵坐标然后标记序号。
end

%%%末端规划路径坐标初始化，赋0
for i=1:1:length(p_x)
    p_x(i)=0;
    p_y(i)=0;
    p_z(i)=0;
end
%%%角度弧度转换
ToDeg = 180/pi;
ToRad = pi/180;
%设置初始关节转动的角度
th1=90;%Link(2).th
th2=0;%Link(3).th
th3=0;%Link(4).th
th4=0;%Link(5).th
th5=0;%Link(6).th
th6=-90;%Link(7).th
th7=d5;%Link(8).th
grid on;
%%%绘制机器人初始状态
DHfk7Dof_Lnya(th1,th2,th3,th4,th5,th6,th7,0);%0为不擦除伪影，1擦除
view(64,20);
hold on
pause;
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);

for xun_huan=1:1:10


shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
rrt_SanWei(ss,index,xun_huan,ss(index(1,xun_huan),5));

% 直角坐标系末端路径规划
% 五次多项式直线末端规划
Draw_line_5(ss(index(1,xun_huan),3),ss(index(1,xun_huan),2),100,3,ss(index(1,xun_huan),5),ss,index,xun_huan,ss(index(1,xun_huan),1),0);%x轴方向左移动
Draw_line_5(ss(index(1,xun_huan),3),ss(index(1,xun_huan),2),100,1,ss(index(1,xun_huan),5),ss,index,xun_huan,ss(index(1,xun_huan),1),0);%y轴方向左移动
% %末端4-3-4曲线末端规划
hold on
Draw_curve_434(ss(index(1,xun_huan),3),-400,-500,-600,0,0,0,0,2,4,ss(index(1,xun_huan),5),ss,index,xun_huan);%规划z轴的
ss(index(1,xun_huan),:)=[0, 0 ,0, 0 ,0 ];%删除矩阵ss的某一行,变成0 0 0 0 0% % %角度轨迹规划
fan_hui=xun_huan+1;%返回目标点为下一次循环的小球
if xun_huan<10
B_x=ss(index(1,fan_hui),1);
B_y=ss(index(1,fan_hui),2);
B_z=ss(index(1,fan_hui),3);
Draw_angle_434(B_x,B_y,B_z,ss,xun_huan,index);%%角度434空间规划 从A到B绘制目标点A到下一个小球轨迹
else
end
end
% % 