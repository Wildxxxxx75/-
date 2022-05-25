function q=Creat_5_curve(q0,q1,v0,v1,acc0,acc1)%五次多项式规划
% clc
% clear
%轨迹定义条件
%时间
t0=0;
t1=2;
%位置和速度（a）
% q0=0;
% q1=10;
% v0=0;
% v1=0;
% acc0=0;%加速度
% acc1=0;
%利用公式（1-25）源自网站古月居，求系数
h=q1-q0;
T=t1-t0;
a0=q0;
a1=v0;
a2=1.0/2*acc0;
a3=1.0/(2*T*T*T)*(20*h-(8*v1+12*v0)*T+(acc1-3*acc0)*(T*T));
a4=1.0/(2*T*T*T*T)*(-30*h+(14*v1+16*v0)*T+(3*acc0-2*acc1)*(T*T));
a5=1.0/(2*T*T*T*T*T)*(12*h-6*(v1+v0)*T+(acc1-acc0)*(T*T));
%轨迹生成
t=t0:0.1:t1;
%位置
q=a0+a1*power((t-t0),1)+a2*power((t-t0),2)+a3*power((t-t0),3)+...
    a4*power(t-t0,4)+a5*power(t-t0,5);
%速度
v=a1+2*a2*power((t-t0),1)+3*a3*power((t-t0),2)+4*a4*power(t-t0,3)+...
    5*a5*power(t-t0,4);
%加速度
acc=2*a2+6*a3*power((t-t0),1)+12*a4*power(t-t0,2)+20*a5*power(t-t0,3);
%绘图
% subplot(3,2,1)
% plot(t,q,'r');
% % axis([0,8,0,11])
% ylabel('position')
% grid on
% subplot(3,2,2)
% plot(t,v,'b');
% % axis([0,8,-1,2.5])
% ylabel('velocity')
% grid on
% subplot(3,2,3)
% plot(t,acc,'g');
% ylabel('acceleration')
% grid on