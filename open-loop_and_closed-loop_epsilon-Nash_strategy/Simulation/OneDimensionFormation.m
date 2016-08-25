clear;
close all;
% Terminal time
tf=100;
% Initial Positions
p=[1;-3;10];
v=[-1;0;3];
% Laplacian Matrix
L=[0 0 0
   -1 1 0
   -1 0 1];
% Original Nash Control
state1initial=[p(1) v(1) p(2) v(2) p(3) v(3)];

% Information Structure
IS = 'OL'; % OL -- Open-loop, CL -- Closed-loop 
perturbation = 1; % 1 -- perturbation on, 2 -- perturbation off
[time1,state1]=ode45(@(t,x) DoubleIntegrator(t,x,L,tf,p,v,IS),[0 tf/2],state1initial);
if perturbation == 1
    state2initial=state1(length(time1),:)/2;
end
if strcmp(IS,'CL')
    [time2,state2]=ode45(@(t,x) DoubleIntegrator(t,x,L,tf,p,v,IS),[tf/2 tf],state2initial);
end
if strcmp(IS,'OL')
    [time2,state2]=ode45(@(t,x) DoubleIntegrator(t,x,L,tf,p,v,IS),[tf/2 tf],state2initial);
end
time=[time1;time2];
state=[state1;state2];
               
subplot(2,1,1);
plot(time,state(:,1),'-or');hold
plot(time,state(:,3),'-og');
plot(time,state(:,5),'-ob');
title 'State Trajectories'
legend('Agent 1', 'Agent 2', 'Agent 3')
grid on
subplot(2,1,2);hold
for i=1:length(time)
    t=time(i);
    if strcmp(IS,'OL')
        M=inv([eye(3,3)-tf^3/6*L -tf*eye(3,3)-tf^2/2*L;tf^2/2*L eye(3,3)+tf*L]);
        u(i,:)=([(t-tf)*L -L]*M*[p;v])';
    end
    if strcmp(IS,'CL')
        M=inv([eye(3,3)-(tf-t)^3/6*L -(tf-t)*eye(3,3)-(tf-t)^2/2*L;(tf-t)^2/2*L eye(3,3)+(tf-t)*L]);
        u(i,:)=([(t-tf)*L -L]*M*state(i,[1 3 5 2 4 6])')';
    end
end
plot(time,u(:,1),'-or');
plot(time,u(:,2),'-og');
plot(time,u(:,3),'-ob');
legend('Agent 1', 'Agent 2', 'Agent 3')
title 'Control Trajectory'
grid on
% subplot(4,1,2);
% plot(u(1,:));title 'u1'
% subplot(4,1,3);
% plot(u(2,:));title 'u2'
% subplot(4,1,4);
% plot(u(3,:));title 'u3'