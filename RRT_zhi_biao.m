function [] = RRT_zhi_biao(Angel)%输入是7个关节的角度，每一列对应一个关节的角度。输出为各个关节的角度，速度，加速度曲线
%调用方法：aa_ang_vel_acc_plot(Angel);
for joint=1:1:7
    
angel_1=Angel(:,joint);%在for循环中，分别提取各个关节的angel
delta_t=0.15;%时间差

t_angel=(0:delta_t:delta_t*(size(angel_1,1)-1))';%角度 画图的时间轴
vel_1=zeros(1,size(angel_1,1)-1);%准备容器
acc_1=zeros(1,size(vel_1,2)-1);%准备容器
for i_count=1:size(angel_1,1)-1
    vel_1(i_count)=(angel_1(i_count+1)-angel_1(i_count))/delta_t; %旋转速度
end
t_vel=(0:delta_t:delta_t*(size(vel_1,2)-1))';%速度 画图的时间轴

for i_count=1:size(vel_1,2)-1
    acc_1(i_count)=(vel_1(i_count+1)-vel_1(i_count))/delta_t; %加速度
end
    t_acc=(0:delta_t:delta_t*(size(acc_1,2)-1))'; %加速度 画图的时间轴


figure(joint+1)
plot(t_angel,angel_1,'g',t_vel,vel_1,'r',t_acc,acc_1,'b');
hold on
title('关节角度-速度-角速度');
legend('角度','速度','加速度') 
end
end

