function xdot = TerminalStateEsti( t,x,g,s1,v1,s2,v2,s3,v3,mu1,mu2,mu3,tf,W,r )
%TERMINALSTATEESTI Summary of this function goes here
%   Detailed explanation goes here
xdot(1:4) = g*([s1;v1]+1/r*kron([tf^3/3;tf^2/2],mu1)-x(1:4));% ...
   % -1/r*kron(W,eye(2,2))*(x(1:4)-x(5:8)+x(1:4)-x(9:12)));
xdot(5:8) = g*([s2;v2]+1/r*kron([tf^3/3;tf^2/2],mu2)-x(5:8) ...
    -1/r*kron(W,eye(2,2))*(x(5:8)-x(1:4)));
xdot(9:12) = g*([s3;v3]+1/r*kron([tf^3/3;tf^2/2],mu3)-x(9:12) ...
    -1/r*kron(W,eye(2,2))*(x(9:12)-x(1:4)));

xdot=xdot';

end

