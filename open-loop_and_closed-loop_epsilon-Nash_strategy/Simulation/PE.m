function [ h1 h2 h3final h4final h5final] = PE( q1,q2,g,r)

M=kron([r*9*q1-8 -9
    -9 r*9*q1-8],eye(2,2));
Minv=inv(M);

x0=[0;3;0;0];
x1=[-2;1;0;0];
x2=[1;1;0;0];
x3=[-1;0;0;0];
x4=[-3;0;0;0];
x5=[2;0;0;0];

h1=[-2;1]-r*9*q1*kron([1 0],eye(2,2))*Minv*[-2;-2;1;-2];
h2=[1;1]-r*9*q1*kron([0 1],eye(2,2))*Minv*[-2;-2;1;-2];
h3=[0;0];h4=[0;0];h5=[0;0];

options = odeset('RelTol', 1e-6);
% 
[th,h]=ode45(@(t,x) TerminalStateEsti(t,x,g,q2,h1,h2),[0 10],[h3' h4' h5'],options);
i=size(th,1);
h3final=h(i,1:2);h4final=h(i,3:4);h5final=h(i,5:6);

%h3final=[0 -0.1165];

% 
[th,P]=ode45(@(t,x) Control(t,x,q1,q2,Minv,h1,h2,h3final',h4final',h5final',r), ...
    [0 3],[x0' x1' x2' x3' x4' x5'],options);
% 
% [th,P]=ode45(@(t,x) Control(t,x,q1,q2,Minv,g,h1,h2), ...
%     [0 3],[x0' x1' x2' x3' x4' x5' h3' h4' h5'],options);


tf=size(th,1);


plot(P(:,1),P(:,2),'-r','LineWidth',2);hold;
plot(P(:,5),P(:,6),'-b','LineWidth',2);
plot(P(:,9),P(:,10),'-g','LineWidth',2);
plot(P(:,13),P(:,14),'-k','LineWidth',2);
plot(P(:,17),P(:,18),'-c','LineWidth',2);
plot(P(:,21),P(:,22),'-m','LineWidth',2);
% xlabel('x');
% ylabel('y')
legend('Evader','Pursuer 1','Pursuer 2','Pursuer 3','Pursuer 4','Pursuer 5')

plot(P(tf,1),P(tf,2),'or');plot(P(1,1),P(1,2),'xr');
plot(P(tf,5),P(tf,6),'ob');plot(P(1,5),P(1,6),'xb');
plot(P(tf,9),P(tf,10),'og');plot(P(1,9),P(1,10),'xg');
plot(P(tf,13),P(tf,14),'ok');plot(P(1,13),P(1,14),'xk');
plot(P(tf,17),P(tf,18),'oc');plot(P(1,17),P(1,18),'xc');
plot(P(tf,21),P(tf,22),'om');plot(P(1,21),P(1,22),'xm');


figure
for i=1:tf
error1(i)=norm([P(i,5)-P(i,1);P(i,6)-P(i,2)]);
error2(i)=norm([P(i,9)-P(i,1);P(i,10)-P(i,2)]);
error3(i)=norm([P(i,13)-P(i,1);P(i,14)-P(i,2)]);
error4(i)=norm([P(i,17)-P(i,1);P(i,18)-P(i,2)]);
error5(i)=norm([P(i,21)-P(i,1);P(i,22)-P(i,2)]);
end
plot(th,error1,'-b','LineWidth',2);hold
plot(th,error2,'-g','LineWidth',2)
plot(th,error3,'-k','LineWidth',2)
plot(th,error4,'-c','LineWidth',2)
plot(th,error5,'-m','LineWidth',2)
xlabel('t');
bk1=13;
bk2=10;
ylabel('||y_i-y_0||^2')
legend('||y_1-y_0||^2',bk1,bk2,'||y_2-y_0||^2',bk1,bk2, ...
    '||y_3-y_0||^2',bk1,bk2,'||y_4-y_0||^2',bk1,bk2,'||y_5-y_0||^2')

end

function xdot=TerminalStateEsti(t,x,g,q2,h1,h2)

h3=x(1:2);h4=x(3:4);h5=x(5:6);
%h4=x(5:6);h4=x(5:6);

xdot(1:2) = g*([-1;0]-h3-9*q2*(4*h3-h1-h2-h4-h5));
xdot(3:4) = g*([-3;0]-h4-9*q2*(2*h4-h1-h3));
xdot(5:6) = g*([2;0]-h5-9*q2*(2*h5-h2-h3));

xdot=xdot';

end
function xdot=Control(t,x,q1,q2,Minv,h1,h2,h3,h4,h5,r)



%h3=x(25:26);h4=x(27:28);h5=x(29:30);

% xdot(25:26) = g*([-1;0]-h3-9*q2*(4*h3-h1-h2-h4-h5));
% xdot(27:28) = g*([-3;0]-h4-9*q2*(2*h4-h1-h3));
% xdot(29:30) = g*([2;0]-h5-9*q2*(2*h5-h2-h3));

u0=-(3-t)*kron([1 1],eye(2,2))*Minv*[-2;-2;1;-2];
u1=-r*(3-t)*q1*kron([1 0],eye(2,2))*Minv*[-2;-2;1;-2];
u2=-r*(3-t)*q1*kron([0 1],eye(2,2))*Minv*[-2;-2;1;-2];
u3=-(3-t)*q2*(4*h3-h1-h2-h4-h5);
u4=-(3-t)*q2*(2*h4-h1-h3);
u5=-(3-t)*q2*(2*h5-h2-h3);

xdot(1:4)= kron([0 1
    0 0],eye(2,2))*x(1:4)+kron([0;1],eye(2,2))*u0;
xdot(5:8)= kron([0 1
    0 0],eye(2,2))*x(5:8)+kron([0;1],eye(2,2))*u1;
xdot(9:12)= kron([0 1
    0 0],eye(2,2))*x(9:12)+kron([0;1],eye(2,2))*u2;
xdot(13:16)= kron([0 1
    0 0],eye(2,2))*x(13:16)+kron([0;1],eye(2,2))*u3;
xdot(17:20)= kron([0 1
    0 0],eye(2,2))*x(17:20)+kron([0;1],eye(2,2))*u4;
xdot(21:24)= kron([0 1
    0 0],eye(2,2))*x(21:24)+kron([0;1],eye(2,2))*u5;


xdot=xdot';

end

