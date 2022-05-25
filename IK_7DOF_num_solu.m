function q=IK_7DOF_num_solu(W)%数值解函数
%W为4*4 的期望位置和姿态
lamda=0.5;%范围（0,1） 0.5

e=zeros(1,6);
q=zeros(7,1);%7*1的0矩阵 六个初始关节角q=0
pref=W(1:3,4);%第四列的1-3行 期望位置
Rref=W(1:3,1:3);%3*3 1-3行 1-3列矩阵 期望姿态
ilimit=700;%修正次数越多，误差越小 
count=1;

while true
    
    count = count + 1;
    if count >= ilimit
        fprintf('iteration number %d final err %f \n',count,err);
        break
    end

    P=DHfk7Dof_kuka_nodraw(q);%初始关节角下的0-7转换矩阵P

    p=P(1:3,4);%第四列的1-3行 当前位置
    perr=pref-p;%计算位置误差perr

    R=P(1:3,1:3);%3*3  当前姿态
    Rerr=R'*Rref;%计算姿态误差Rerr
    th=acos((Rerr(1,1)+Rerr(2,2)+Rerr(3,3)-1)/2);
    
    if Rerr==eye(3)
        werr=[0 0 0]';
    else 
        %%姿态误差：角度误差
        werr=(th/2*sin(th))*[Rerr(3,2)-Rerr(2,3),Rerr(1,3)-Rerr(3,1), Rerr(2,1)-Rerr(1,2)]';
    end

    e(1:3)=perr(1:3);
    e(4:6)=werr(1:3);

     err=norm(e(1:3))+norm(e(4:6));

    if err<=1e-6%如果误差小于或等于10^-6
       fprintf(' iteration number %d final err %f \n',count,err);
      break
    end
    %%%限定滑动关节变化长度
    if(q(7)<15)
        q(7)=15;
       else
        if (q(7)>=170)
        q(7)=170;
        end
    end
    %%%转换角度值
    for i=1:1:6
        if(q(i)>360) 
            q(i)=q(i)-360;
        else
            if(q(i)<-360)
                q(i)=q(i)+360;
            end
        end
    end
    J=Jacobian7DoF_Ln(q(1),q(2),q(3),q(4),q(5),q(6),q(7));%当前角度构建雅可比矩阵
    deta_q=lamda*pinv(J)*e';%根据误差得出角度微调量deta_q  牛顿下山法
    q=q+deta_q;%更新当前角度
end

    