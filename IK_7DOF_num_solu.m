function q=IK_7DOF_num_solu(W)%��ֵ�⺯��
%WΪ4*4 ������λ�ú���̬
lamda=0.5;%��Χ��0,1�� 0.5

e=zeros(1,6);
q=zeros(7,1);%7*1��0���� ������ʼ�ؽڽ�q=0
pref=W(1:3,4);%�����е�1-3�� ����λ��
Rref=W(1:3,1:3);%3*3 1-3�� 1-3�о��� ������̬
ilimit=700;%��������Խ�࣬���ԽС 
count=1;

while true
    
    count = count + 1;
    if count >= ilimit
        fprintf('iteration number %d final err %f \n',count,err);
        break
    end

    P=DHfk7Dof_kuka_nodraw(q);%��ʼ�ؽڽ��µ�0-7ת������P

    p=P(1:3,4);%�����е�1-3�� ��ǰλ��
    perr=pref-p;%����λ�����perr

    R=P(1:3,1:3);%3*3  ��ǰ��̬
    Rerr=R'*Rref;%������̬���Rerr
    th=acos((Rerr(1,1)+Rerr(2,2)+Rerr(3,3)-1)/2);
    
    if Rerr==eye(3)
        werr=[0 0 0]';
    else 
        %%��̬���Ƕ����
        werr=(th/2*sin(th))*[Rerr(3,2)-Rerr(2,3),Rerr(1,3)-Rerr(3,1), Rerr(2,1)-Rerr(1,2)]';
    end

    e(1:3)=perr(1:3);
    e(4:6)=werr(1:3);

     err=norm(e(1:3))+norm(e(4:6));

    if err<=1e-6%������С�ڻ����10^-6
       fprintf(' iteration number %d final err %f \n',count,err);
      break
    end
    %%%�޶������ؽڱ仯����
    if(q(7)<15)
        q(7)=15;
       else
        if (q(7)>=170)
        q(7)=170;
        end
    end
    %%%ת���Ƕ�ֵ
    for i=1:1:6
        if(q(i)>360) 
            q(i)=q(i)-360;
        else
            if(q(i)<-360)
                q(i)=q(i)+360;
            end
        end
    end
    J=Jacobian7DoF_Ln(q(1),q(2),q(3),q(4),q(5),q(6),q(7));%��ǰ�Ƕȹ����ſɱȾ���
    deta_q=lamda*pinv(J)*e';%�������ó��Ƕ�΢����deta_q  ţ����ɽ��
    q=q+deta_q;%���µ�ǰ�Ƕ�
end

    