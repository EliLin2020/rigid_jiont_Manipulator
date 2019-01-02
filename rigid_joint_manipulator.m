%rigid_joint_manipulator
%2018/12/31 林祥
clc; clear; format long;
%--------------参数定义-----------------
m1=5;  m2=5; l1=5;  l2=5;  %机械臂质量和长度
I1=m1*l1^2/12;      %转动惯量  
I2=m2*l2^2/12;

l_square=7;        %正方形半边长；机械臂跟踪正方形轨道速度
t0=0;  tn=8*l_square*4; dt=4; N=(tn-t0)/dt;     %时间始终点，步长
t=t0:dt:tn;         %时间向量

tau=0.01;   %采样步长
n=dt/tau;   %采样步数
t_tau=0:tau:dt;  %采样时间向量

q=zeros(2,N+1);              %q初始化
Dq=zeros(2,N+1);            %Dq初始化

Tau=[0;0];    Tau0=[0;0];      %力矩初始化为0

%--x_desired--Square
xd1=zeros(1,N/8) + l_square;
xd2=l_square*linspace(1,-1,N/4+1);
xd3=zeros(1,N/4-1) - l_square;
xd=[ xd1,xd2,xd3,-xd2,xd1];  

%--y_desired--Square
yd1=linspace(0,l_square,N/8+1);
yd2=zeros(1,N/4-1)+l_square;
yd3=linspace(l_square,-l_square,N/4+1);
yd4=zeros(1,N/4-1)-l_square;
yd5=linspace(-l_square,0,N/8+1);
yd=[ yd1,yd2,yd3,yd4,yd5];  

%PID控制器参数
Kp1=7000;   Ki1=0;  Kd1=58000;
Kp2=7000;   Ki2=0;  Kd2=70000;
%------------------求解-------------------------
for k=1:N  %时间遍历
    
    %坐标反解，求endpoint期望达到的q1,q2
    r=xd(k)^2+yd(k)^2;
    theta1=acos((l1^2+r-l2^2)/(2*l1*sqrt(r)));
    theta2=acos((l1^2+l2^2-r)/(2*l1*l2));
    theta3=atan(yd(k)/xd(k));
    qd2=pi-theta2;
    if xd(k)>0,qd1=theta3-theta1;  %一四象限
    elseif yd(k)>0, qd1=theta3-theta1+pi; %二象限
    else, qd1=theta3-theta1-pi;  %三象限
    end
    
    q_tau=zeros(2,n+1);      q_tau(:,1)=q(:,k);     %采样周期中的q初始化
    Dq_tau=zeros(2,n+1);    Dq_tau(:,1)=Dq(:,k);     %采样周期中的Dq初始化
    ey=zeros(2,n+1);                 %误差初始化
    Tau0(:)=Tau(:);
    
    for i=1:n  %采样周期遍历，在采样周期中收敛到期望位置
        
        %PID控制率
        ey(:,i)=[qd1;qd2]-q_tau(:,i); %下一时刻期望坐标与实际坐标差值
        if i==1, Tau(1)=Tau0(1)+Kp1*ey(1,i)+Ki1*(sum(ey(1,:)))+Kd1*ey(1,i);
            Tau(2)=Tau0(2)+Kp2*ey(2,i)+Ki2*(sum(ey(2,:)))+Kd2*ey(2,i);
        else, Tau(1)=Tau0(1)+Kp1*ey(1,i)+Ki1*(sum(ey(1,:)))+Kd1*(ey(1,i)-ey(1,i-1));
            Tau(2)=Tau0(2)+Kp2*ey(2,i)+Ki2*(sum(ey(2,:)))+Kd2*(ey(2,i)-ey(2,i-1));
        end
   
        %动力学模型,忽略重力影响，视关节点为刚性模型(不考虑弹性效应，非线性效应等)
        M1=4*(I1+I2)+m2*(l1^2+l1*l2*cos(q_tau(2,i)));
        M2=4*I2+m2*(l1*l2/2*cos(q_tau(2,i)));
        M4=4*I2;
        M=[M1,M2;M2,M4];
        C=-m2*l1*l2/2*sin(q_tau(2,i))*[ Dq_tau(2,i), Dq_tau(1,i)+Dq_tau(2,i); -Dq_tau(1,i), 0];
    
        answer=RK4n(@dfun,M,C,Tau,4,t_tau(i),tau,[q_tau(:,i);Dq_tau(:,i)]);  %动力学方程，ode4求解
        q_tau(:,i+1)=[answer(1); answer(2)];
        Dq_tau(:,i+1)=[answer(3);answer(4)];
%         Dq_tau(:,i+1)=tau*(M\(Tau-C*Dq_tau(:,i)))+Dq_tau(:,i);    %Eluer法求解
%         q_tau(:,i+1)=Dq_tau(:,i)*tau+q_tau(:,i);

%         %坐标转换，求endpoint坐标
%         x=l1*cos(q_tau(1,i+1))+l2*cos(q_tau(1,i+1)+q_tau(2,i+1));
%         y=l1*sin(q_tau(1,i+1))+l2*sin(q_tau(1,i+1)+q_tau(2,i+1));
%      
%         %画图
%         figure(1);  set(gca,'Fontsize',16);
%         plot(x,y,'r.');  hold on;  grid on;
%         axis([-1,1,-1,1]*(l1+l2));
%         title('enpoint点坐标');
    end
    q(:,k+1)=q_tau(:,n+1);    %采样周期结束时的q_tau记录到q值中
    Dq(:,k+1)=Dq_tau(:,n+1); 
    
    %坐标转换，求endpoint坐标
    x=l1*cos(q(1,k+1))+l2*cos(q(1,k+1)+q(2,k+1));
    y=l1*sin(q(1,k+1))+l2*sin(q(1,k+1)+q(2,k+1));

    %画图
    figure(1);  set(gca,'Fontsize',16);
    plot(x,y,'r.');  hold on;  grid on;
    axis([-1,1,-1,1]*(l1+l2));
    title('enpoint点坐标');
%     %一个小循环周期的误差收敛过程
%     figure(2);  set(gca,'Fontsize',16);   %一个小循环周期的误差收敛过程
%     plot(linspace(0,dt,n+1),ey);  hold on;  grid on;
%     title('error');
end
plot(xd,yd,'g-');%理想情况

%-------------函数定义-------------
function dy=dfun(M,C,Tau,~,y)
dy=[y(3); y(4);  M\( Tau-C * [y(3); y(4)] ) ];
end

function y=RK4n(deriv,M,C,Tau,n,x,dx,y)  %四阶Runge-Kutta法计算一个步长, n为维数
x0=x;  y0=y;
dy1=feval(deriv,M,C,Tau,x0,y);
    y(1:n)=y0(1:n)+0.5*dx*dy1(1:n); 
dy2=feval(deriv,M,C,Tau,x0+0.5*dx,y);
    y(1:n)=y0(1:n)+0.5*dx*dy2(1:n); 
dy3=feval(deriv,M,C,Tau,x0+0.5*dx,y);
    y(1:n)=y0(1:n)+dx*dy3(1:n); 
dy4=feval(deriv,M,C,Tau,x0+dx,y);
dy(1:n)=(dy1(1:n)+2*(dy2(1:n)+dy3(1:n))+dy4(1:n))/6;  dy=dy(:);
    y(1:n)=y0(1:n)+dx*dy(1:n);      y=y(:);
end
