function Draw_curve_434(th_1,th_2,th_3,th_4,th1_1,th1_2,th4_1,th4_2,num1,j,ban_jing,ss,index,xun_huan)%末端直角规划
global p_x p_y p_z d5
i=6;%调节采样数量来改变绘制时间 原先i为6
n=81;%设置A到B点的采集数据个数，原先n为81
n1=length(p_x);
Posion=Creat_434_curve(th_1,th_2,th_3,th_4,th1_1,th1_2,th4_1,th4_2,num1,j);
%%%定义一次函数y=kx+b实现坐标线性转换
k=-9.0;b=100;%这里的b表示y方向初始位置，k越小，往y轴负方向距离越多
num=1;
for t=1:i:n%采集数据个数为n/i
    x(num)=0;
%     p_x(n1+num)=x(num);
     y(num)=k*t+b;
%     p_y(n1+num)=y(num);
     z(num)=Posion(t);
%      p_z(n1+num)=z(num);
    W=[1,0,0,x(num);
       0,1,0,y(num);
       0,0,1,z(num);
       0,0,0,1];
%%%数值解计算逆解
hold on
% plot3(p_x,p_y,p_z,'ko');hold on;%x,y,z为数组，绘制圆点形状轨迹

% plot3(x,y,z,'k*');hold on;%x,y,z为数组
theta2=IK_7DOF_num_solu(W);%theta2为角度制

if(t>n-i)
    DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),0);
    [xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %半径为j的圆
% shading interp%防止surf画出黑色小球
xu_hao = num2str(xun_huan);%数值转字符串，准备循环从1到10，10个数
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%标记序号，原来ss矩阵中第m行第1列做横坐标，
else
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %半径为j的圆
% shading interp%防止surf画出黑色小球
xu_hao = num2str(xun_huan);%数值转字符串，准备循环从1到10，10个数
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%标记序号，原来ss矩阵中第m行第1列做横坐标，
    DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),1);%清除
end


xiao_qiu_zhui_zong(ss,xun_huan,index);

shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
hold on
q1(num)=theta2(1);
q2(num)=theta2(2);
q3(num)=theta2(3);
q4(num)=theta2(4);
q5(num)=theta2(5);
q6(num)=theta2(6);
q7(num)=theta2(7);

num=num+1;

end
%%%末端手动控制
for i=1:1:5
    d5=theta2(7)+10*i;
    shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
    xiao_qiu_zhui_zong(ss,xun_huan,index);
    hold on
   [xx,yy,zz]= ellipsoid(x(num-1),y(num-1),z(num-1)-10*i,ban_jing,ban_jing,ban_jing);
   surf(xx,yy,zz) %半径为j的圆
   % shading interp%防止surf画出黑色小球
   xiao_qiu_zhui_zong(ss,xun_huan,index);
   hold on
   DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),d5,1);
end
[xx,yy,zz]= ellipsoid(x(num-1),y(num-1),z(num-1),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %半径为j的圆
% shading interp%防止surf画出黑色小球
xiao_qiu_zhui_zong(ss,xun_huan,index);
hold on

t=1:1:num-1;
% if(j==1)
% figure(4),
% plot(t,q1,'r','LineWidth',2);hold on;%绘制角度位置
% plot(t,q2,'b','LineWidth',2);hold on;%绘制角度位置
% plot(t,q3,'g','LineWidth',2);hold on;%绘制角度位置
% plot(t,q4,'k','LineWidth',2);hold on;%绘制角度位置
% plot(t,q5,'c','LineWidth',2);hold on;%绘制角度位置
% plot(t,q6,'m','LineWidth',2);hold on;%绘制角度位置
% plot(t,q7,'y','LineWidth',2);grid on;%绘制角度位置
% axis([1,num-1,-400,400]);
% end