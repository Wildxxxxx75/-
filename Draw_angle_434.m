function Draw_angle_434(B_x,B_y,B_z,ss,xun_huan,index)
global A_x A_y A_z
shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
xiao_qiu_zhui_zong(ss,xun_huan,index);
hold on

W_A=[1,0,0,A_x;%%起点A位姿
     0,1,0,A_y;
     0,0,1,A_z;
     0,0,0,1];
i=5;%调节采样数量来改变绘制时间
n=150;%设置A到B点的采集数据个数
for j=1:1:1
    B1_x=B_x(j);B1_y=B_y(j);B1_z=B_z(j);
    W_B=[1,0,0,B1_x;%%终点B位姿
         0,1,0,B1_y;
         0,0,1,B1_z;
         0,0,0,1];
    A_th=IK_7DOF_num_solu(W_A);%A_th为角度制,A位置逆解出来的角度
    B_th=IK_7DOF_num_solu(W_B);%A_th为角度制,B位置逆解出来的角度

    Q1_D=(B_th(1)-A_th(1))/3;
    Q2_D=(B_th(2)-A_th(2))/3;
    Q3_D=(B_th(3)-A_th(3))/3;
    Q4_D=(B_th(4)-A_th(4))/3;
    Q5_D=(B_th(5)-A_th(5))/3;
    Q6_D=(B_th(6)-A_th(6))/3;
    Q7_D=(B_th(7)-A_th(7))/3;

    Q1=Creat_434_curve(A_th(1),A_th(1)+Q1_D,A_th(1)+Q1_D*2,B_th(1),0,0,0,0,3,j);%4-3-4路径规划 关节1
    Q2=Creat_434_curve(A_th(2),A_th(2)+Q2_D,A_th(2)+Q2_D*2,B_th(2),0,0,0,0,4,j);%4-3-4路径规划 关节2
    Q3=Creat_434_curve(A_th(3),A_th(3)+Q3_D,A_th(3)+Q3_D*2,B_th(3),0,0,0,0,5,j);%4-3-4路径规划 关节3
    Q4=Creat_434_curve(A_th(4),A_th(4)+Q4_D,A_th(4)+Q4_D*2,B_th(4),0,0,0,0,6,j);%4-3-4路径规划 关节4
    Q5=Creat_434_curve(A_th(5),A_th(5)+Q4_D,A_th(5)+Q5_D*2,B_th(5),0,0,0,0,7,j);%4-3-4路径规划 关节5
    Q6=Creat_434_curve(A_th(6),A_th(6)+Q3_D,A_th(6)+Q6_D*2,B_th(6),0,0,0,0,8,j);%4-3-4路径规划 关节6
    Q7=Creat_434_curve(A_th(7),A_th(7)+Q7_D,A_th(7)+Q7_D*2,B_th(7),0,0,0,0,9,j);%4-3-4路径规划 关节7
    num=1;
    for t=0:i:n-1%采集数据个数为n/i
        shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
        xiao_qiu_zhui_zong(ss,xun_huan,index);
        DHfk7Dof_Lnya(Q1(num),Q2(num),Q3(num),Q4(num),Q5(num),Q6(num),Q7(num),1);
        num=num+i;
    end
    shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
    xiao_qiu_zhui_zong(ss,xun_huan,index);
    DHfk7Dof_Lnya(Q1(num-i),Q2(num-i),Q3(num-i),Q4(num-i),Q5(num-i),Q6(num-i),Q7(num-i),1);
%     plot3(A_x,A_y,A_z,'r*');
%     plot3(B1_x,B1_y,B1_z,'ko','LineWidth',2); grid on;pause;
    
end
%绘图
% figure(10);
% t=0:1:n-1;
% plot(t,Q1,'c-','LineWidth',2);hold on;
% plot(t,Q2,'b-','LineWidth',2);hold on;
% plot(t,Q3,'k-','LineWidth',2);hold on;
% plot(t,Q4,'g-','LineWidth',2);hold on;
% plot(t,Q5,'y-','LineWidth',2);hold on;
% plot(t,Q6,'m-','LineWidth',2);hold on;
% plot(t,Q7,'r-','LineWidth',2);hold on;
% xlabel('t');ylabel('position');
% legend('position1','position2','position3','position4','position5','position6','position7') %可依次设置成你想要的名字grid on;%绘制结束