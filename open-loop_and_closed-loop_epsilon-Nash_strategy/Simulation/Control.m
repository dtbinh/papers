function xdot = Control( t,x,s1f,s2f,s3f,v1f,v2f,v3f,mu12,mu13,tf,r)
%CONTROL Summary of this function goes here
%   Detailed explanation goes here
u1=[0;0];
u2=-(tf-t)/r*(s2f-s1f+mu12)-1/r*(v2f-v1f);
u3=-(tf-t)/r*(s3f-s1f+mu13)-1/r*(v3f-v1f);


xdot(1:4)= kron([0 1
    0 0],eye(2,2))*x(1:4)+kron([0;1],eye(2,2))*u1;
xdot(5:8)= kron([0 1
    0 0],eye(2,2))*x(5:8)+kron([0;1],eye(2,2))*u2;
xdot(9:12)= kron([0 1
    0 0],eye(2,2))*x(9:12)+kron([0;1],eye(2,2))*u3;

xdot=xdot';

end

