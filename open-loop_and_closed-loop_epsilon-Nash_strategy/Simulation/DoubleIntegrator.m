function [xdot,u] = DoubleIntegrator( t,x,L,tf,p,v, IS)
   
if strcmp(IS,'CL')
    M=inv([eye(3,3)-(tf-t)^3/6*L -(tf-t)*eye(3,3)-(tf-t)^2/2*L;(tf-t)^2/2*L eye(3,3)+(tf-t)*L]);
    u=[(t-tf)*L -L]*M*x([1 3 5 2 4 6]);
end

if strcmp(IS,'OL')
    M=inv([eye(3,3)-tf^3/6*L -tf*eye(3,3)-tf^2/2*L;tf^2/2*L eye(3,3)+tf*L]);
    u=[(t-tf)*L -L]*M*[p;v];
end

xdot(1:2)= [0 1;0 0]*x(1:2)+[0;1]*u(1);
xdot(3:4)= [0 1;0 0]*x(3:4)+[0;1]*u(2);
xdot(5:6)= [0 1;0 0]*x(5:6)+[0;1]*u(3);

xdot=xdot';

end