function [ x] = Test1
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
a=1;b=0;c=0;d=2;

[T, x]=ode45(@myfun,[1,10],[a,b,c,d]);

plot(T,x(:,1))
end

function xdot = myfun( t,x )
%TEST1 Summary of this function goes here
%   Detailed explanation goes here
%a=x(1);b=x(2);c=x(3);d=x(4);

xdot(1)=2*(x(2)-x(1));
xdot(2)=2*(x(1)-x(2)+x(3)-x(2));
xdot(3)=2*(x(2)-x(3)+x(4)-x(3));
xdot(4)=2*(x(3)-x(4));

xdot=xdot';
end

