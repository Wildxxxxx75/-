function Draw_curve_434(th_1,th_2,th_3,th_4,th1_1,th1_2,th4_1,th4_2,num1,j,ban_jing,ss,index,xun_huan)%ĩ��ֱ�ǹ滮
global p_x p_y p_z d5
i=6;%���ڲ����������ı����ʱ�� ԭ��iΪ6
n=81;%����A��B��Ĳɼ����ݸ�����ԭ��nΪ81
n1=length(p_x);
Posion=Creat_434_curve(th_1,th_2,th_3,th_4,th1_1,th1_2,th4_1,th4_2,num1,j);
%%%����һ�κ���y=kx+bʵ����������ת��
k=-9.0;b=100;%�����b��ʾy�����ʼλ�ã�kԽС����y�Ḻ�������Խ��
num=1;
for t=1:i:n%�ɼ����ݸ���Ϊn/i
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
%%%��ֵ��������
hold on
% plot3(p_x,p_y,p_z,'ko');hold on;%x,y,zΪ���飬����Բ����״�켣

% plot3(x,y,z,'k*');hold on;%x,y,zΪ����
theta2=IK_7DOF_num_solu(W);%theta2Ϊ�Ƕ���

if(t>n-i)
    DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),0);
    [xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %�뾶Ϊj��Բ
% shading interp%��ֹsurf������ɫС��
xu_hao = num2str(xun_huan);%��ֵת�ַ�����׼��ѭ����1��10��10����
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%�����ţ�ԭ��ss�����е�m�е�1���������꣬
else
[xx,yy,zz]= ellipsoid(x(num),y(num),z(num),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %�뾶Ϊj��Բ
% shading interp%��ֹsurf������ɫС��
xu_hao = num2str(xun_huan);%��ֵת�ַ�����׼��ѭ����1��10��10����
text(x(num),y(num),z(num),xu_hao,'Color','red','FontSize',14);%�����ţ�ԭ��ss�����е�m�е�1���������꣬
    DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),theta2(7),1);%���
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
%%%ĩ���ֶ�����
for i=1:1:5
    d5=theta2(7)+10*i;
    shui_ping_yuan_zhu([0,0,0]',[0,1,0]',1000,4000,0);
    xiao_qiu_zhui_zong(ss,xun_huan,index);
    hold on
   [xx,yy,zz]= ellipsoid(x(num-1),y(num-1),z(num-1)-10*i,ban_jing,ban_jing,ban_jing);
   surf(xx,yy,zz) %�뾶Ϊj��Բ
   % shading interp%��ֹsurf������ɫС��
   xiao_qiu_zhui_zong(ss,xun_huan,index);
   hold on
   DHfk7Dof_Lnya(theta2(1),theta2(2),theta2(3),theta2(4),theta2(5),theta2(6),d5,1);
end
[xx,yy,zz]= ellipsoid(x(num-1),y(num-1),z(num-1),ban_jing,ban_jing,ban_jing);
surf(xx,yy,zz) %�뾶Ϊj��Բ
% shading interp%��ֹsurf������ɫС��
xiao_qiu_zhui_zong(ss,xun_huan,index);
hold on

t=1:1:num-1;
% if(j==1)
% figure(4),
% plot(t,q1,'r','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q2,'b','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q3,'g','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q4,'k','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q5,'c','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q6,'m','LineWidth',2);hold on;%���ƽǶ�λ��
% plot(t,q7,'y','LineWidth',2);grid on;%���ƽǶ�λ��
% axis([1,num-1,-400,400]);
% end