%ORIGINALNASH Summary of this function goes here
%   Detailed explanation goes here
close all;
tf=3;

x1=[0;0;0;2];
x2=[-3;0;0;0];
x3=[3;0;0;0];

s1=[eye(2,2) tf*eye(2,2)]*x1;
v1=[zeros(2,2) eye(2,2)]*x1;
s2=[eye(2,2) tf*eye(2,2)]*x2;
v2=[zeros(2,2) eye(2,2)]*x2;
s3=[eye(2,2) tf*eye(2,2)]*x3;
v3=[zeros(2,2) eye(2,2)]*x3;

L=[0 0 0
   -1 1 0
   -1 0 1];

mu12=[2;2];
mu13=[-2;2];

mu1=[0;0];mu2=-mu12;mu3=-mu13;

W=[tf^3/3 tf^2/2
   tf^2/2 tf];

for j=1:50
    r=j/10;

R=r*eye(3,3);

%%================================Original Nash===========================%
M=kron(eye(6,6)+kron(W,inv(R)*L),eye(2,2));

Vinv=inv(M)*([s1;s2;s3;v1;v2;v3] ...
    +kron(kron([tf^3/3;tf^2/2],inv(R)),eye(2,2))*[mu1;mu2;mu3]);

s1f=Vinv(1:2);s2f=Vinv(3:4);s3f=Vinv(5:6);
v1f=Vinv(7:8);v2f=Vinv(9:10);v3f=Vinv(11:12);

%u3=zeros(2,301);
for i=1:(10*tf+1);
u1(:,i)=[0;0];
u2(:,i)=-(tf-(i-1)/10)/r*(s2f-s1f+mu12)-1/r*(v2f-v1f);
u3(:,i)=-(tf-(i-1)/10)/r*(s3f-s1f+mu13)-1/r*(v3f-v1f);
end
%%==============================Distributed Nash =========================%

h3init=[0;0;0;0];h4init=[0;0;0;0];h5init=[0;0;0;0];
g=40;
options = odeset('RelTol', 1e-6);
% 
[the,h]=ode45(@(t,x) TerminalStateEsti(t,x,g,s1,v1,s2,v2,s3,v3,mu1,mu2,mu3,tf,W,r), ...
    [0 0.1],[h3init' h4init' h5init'],options);
i=size(the,1);
hs1f=h(i,1:2)';hv1f=h(i,3:4)';
hs2f=h(i,5:6)';hv2f=h(i,7:8)';
hs3f=h(i,9:10)';hv3f=h(i,11:12)';

for i=1:(10*tf+1);
u1h(:,i)=[0;0];
u2h(:,i)=-(tf-(i-1)/10)/r*(hs2f-hs1f+mu12)-1/r*(hv2f-hv1f);
u3h(:,i)=-(tf-(i-1)/10)/r*(hs3f-hs1f+mu13)-1/r*(hv3f-hv1f);
end


g2=7;
options = odeset('RelTol', 1e-6);
% 
[th20,h20]=ode45(@(t,x) TerminalStateEsti(t,x,g2,s1,v1,s2,v2,s3,v3,mu1,mu2,mu3,tf,W,r), ...
    [0 0.1],[h3init' h4init' h5init'],options);
i=size(th20,1);
hs1f20=h20(i,1:2)';hv1f20=h20(i,3:4)';
hs2f20=h20(i,5:6)';hv2f20=h20(i,7:8)';
hs3f20=h20(i,9:10)';hv3f20=h20(i,11:12)';

for i=1:(10*tf+1);
u1h20(:,i)=[0;0];
u2h20(:,i)=-(tf-(i-1)/10)/r*(hs2f20-hs1f20+mu12)-1/r*(hv2f20-hv1f20);
u3h20(:,i)=-(tf-(i-1)/10)/r*(hs3f20-hs1f20+mu13)-1/r*(hv3f20-hv1f20);
end


[th1,P1]=ode45(@(t,x) Control(t,x,hs1f,hs2f,hs3f,hv1f,hv2f,hv3f,mu12,mu13,tf,r), ...
    [0 tf],[x1' x2' x3']);

[th2,P2]=ode45(@(t,x) Control(t,x, ...
    hs1f20,hs2f20,hs3f20,hv1f20,hv2f20,hv3f20,mu12,mu13,tf,r), ...
    [0 tf],[x1' x2' x3']);
%%================================Plot====================================%
% figure;
% subplot(3,2,1);
% plot(0:0.1:tf,u1(1,:),'-b','LineWidth',2);hold;plot(0:0.1:tf,u1h20(1,:),'or','LineWidth',2);
% plot(0:0.1:tf,u1h(1,:),'+g','LineWidth',2);title('u_{1x}');
% 
% subplot(3,2,2);
% plot(0:0.1:tf,u1(2,:),'-b','LineWidth',2);hold;plot(0:0.1:tf,u1h20(2,:),'or','LineWidth',2);
% plot(0:0.1:tf,u1h(2,:),'+g','LineWidth',2);title('u_{1y}');
% 
% subplot(3,2,3);
% plot(0:0.1:tf,u2(1,:),'-b','LineWidth',2);hold;plot(0:0.1:tf,u2h20(1,:),'or','LineWidth',2);
% plot(0:0.1:tf,u2h(1,:),'+g','LineWidth',2);title('u_{2x}');
% 
% subplot(3,2,4);
% plot(0:0.1:tf,u2(2,:),'-b','LineWidth',2);hold;plot(0:0.1:tf,u2h20(2,:),'or','LineWidth',2);
% plot(0:0.1:tf,u2h(2,:),'+g','LineWidth',2);title('u_{2y}');
% 
% subplot(3,2,5);
% plot(0:0.1:tf,u3(1,:),'-b','LineWidth',2);hold;plot(0:0.1:tf,u3h20(1,:),'or','LineWidth',2);
% plot(0:0.1:tf,u3h(1,:),'+g','LineWidth',2);title('u_{3x}');
% 
% subplot(3,2,6);
% plot(0:0.1:tf,u3(2,:),'-b','LineWidth',2);hold;plot(0:0.1:tf,u3h20(2,:),'or','LineWidth',2);
% plot(0:0.1:tf,u3h(2,:),'+g','LineWidth',2);title('u_{3y}');
% legend('Original Nash Control','Distributed Design k_i=20', ...
%     'Distributed Design k_i=40');


[th,P]=ode45(@(t,x) Control(t,x,s1f,s2f,s3f,v1f,v2f,v3f,mu12,mu13,tf,r), ...
    [0 tf],[x1' x2' x3']);
% 
% [th,P]=ode45(@(t,x) Control(t,x,q1,q2,Minv,g,h1,h2), ...
%     [0 3],[x0' x1' x2' x3' x4' x5' h3' h4' h5'],options);

%figure;
% p1=plot(P(:,1),P(:,2),'-b','LineWidth',2);hold;
% plot(P(:,5),P(:,6),'-b','LineWidth',2);
% plot(P(:,9),P(:,10),'-b','LineWidth',2);
i0=size(th,1);
eta0(j)=(  norm([P(i0,1);P(i0,2)]-[P(i0,5);P(i0,6)]-mu12)+ ...
            norm([P(i0,1);P(i0,2)]-[P(i0,9);P(i0,10)]-mu13));

i2=size(th2,1);        
% p2=plot(P2(:,1),P2(:,2),'or','LineWidth',2);
% plot(P2(:,5),P2(:,6),'or','LineWidth',2);
% plot(P2(:,9),P2(:,10),'or','LineWidth',2);
eta2(j)=(  norm([P2(i2,1);P2(i2,2)]-[P2(i2,5);P2(i2,6)]-mu12)+ ...
            norm([P2(i2,1);P2(i2,2)]-[P2(i2,9);P2(i2,10)]-mu13));

i1=size(th1,1);
% p3=plot(P1(:,1),P1(:,2),'+g','LineWidth',2);
% plot(P1(:,5),P1(:,6),'+g','LineWidth',2);
% plot(P1(:,9),P1(:,10),'+g','LineWidth',2);
eta1(j)=(  norm([P1(i1,1);P1(i1,2)]-[P1(i1,5);P1(i1,6)]-mu12)+ ...
            norm([P1(i1,1);P1(i1,2)]-[P1(i1,9);P1(i1,10)]-mu13));
% plot(P(:,13),P(:,14),'-k','LineWidth',2);
% plot(P(:,17),P(:,18),'-c','LineWidth',2);
% plot(P(:,21),P(:,22),'-m','LineWidth',2);
% xlabel('x');
% ylabel('y')
% legend([p1 p2 p3],{'Original Nash Control','Distributed Design k_i=20', ...
%     'Distributed Design k_i=40'})
end

plot(0.1:0.1:5,eta0,'-b');hold;plot(0.1:0.1:5,eta2,'-r');plot(0.1:0.1:5,eta1,'-g')
% plot(P(tf,1),P(tf,2),'or');plot(P(1,1),P(1,2),'xr');
% plot(P(tf,5),P(tf,6),'ob');plot(P(1,5),P(1,6),'xb');
% plot(P(tf,9),P(tf,10),'og');plot(P(1,9),P(1,10),'xg');
% plot(P(tf,13),P(tf,14),'ok');plot(P(1,13),P(1,14),'xk');
% plot(P(tf,17),P(tf,18),'oc');plot(P(1,17),P(1,18),'xc');
% plot(P(tf,21),P(tf,22),'om');plot(P(1,21),P(1,22),'xm');