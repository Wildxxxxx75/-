function [] = RRT_zhi_biao(Angel)%������7���ؽڵĽǶȣ�ÿһ�ж�Ӧһ���ؽڵĽǶȡ����Ϊ�����ؽڵĽǶȣ��ٶȣ����ٶ�����
%���÷�����aa_ang_vel_acc_plot(Angel);
for joint=1:1:7
    
angel_1=Angel(:,joint);%��forѭ���У��ֱ���ȡ�����ؽڵ�angel
delta_t=0.15;%ʱ���

t_angel=(0:delta_t:delta_t*(size(angel_1,1)-1))';%�Ƕ� ��ͼ��ʱ����
vel_1=zeros(1,size(angel_1,1)-1);%׼������
acc_1=zeros(1,size(vel_1,2)-1);%׼������
for i_count=1:size(angel_1,1)-1
    vel_1(i_count)=(angel_1(i_count+1)-angel_1(i_count))/delta_t; %��ת�ٶ�
end
t_vel=(0:delta_t:delta_t*(size(vel_1,2)-1))';%�ٶ� ��ͼ��ʱ����

for i_count=1:size(vel_1,2)-1
    acc_1(i_count)=(vel_1(i_count+1)-vel_1(i_count))/delta_t; %���ٶ�
end
    t_acc=(0:delta_t:delta_t*(size(acc_1,2)-1))'; %���ٶ� ��ͼ��ʱ����


figure(joint+1)
plot(t_angel,angel_1,'g',t_vel,vel_1,'r',t_acc,acc_1,'b');
hold on
title('�ؽڽǶ�-�ٶ�-���ٶ�');
legend('�Ƕ�','�ٶ�','���ٶ�') 
end
end

