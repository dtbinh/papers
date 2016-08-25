function xdot = ControlPareto( t,x,s1f,s2f,s3f,v1f,v2f,v3f,mu12,mu13,tf,r,alpha1,alpha2,alpha3)
%CONTROL Summary of this function goes here
%   Detailed explanation goes here
u1=-(tf-t)/(alpha1*r)*((alpha1+alpha2)*(s1f-s2f-mu12)+(alpha1+alpha3)*(s1f-s3f-mu13)) ...
    -1/(alpha1*r)*((alpha1+alpha2)*(v1f-v2f)+(alpha1+alpha3)*(v1f-v3f));
u2=-(tf-t)/(alpha2*r)*(alpha1+alpha2)*(s2f-s1f+mu12)-1/(alpha2*r)*(alpha1+alpha2)*(v2f-v1f);
u3=-(tf-t)/(alpha3*r)*(alpha1+alpha3)*(s3f-s1f+mu13)-1/(alpha3*r)*(alpha1+alpha3)*(v3f-v1f);


xdot(1:4)= kron([0 1
    0 0],eye(2,2))*x(1:4)+kron([0;1],eye(2,2))*u1;
xdot(5:8)= kron([0 1
    0 0],eye(2,2))*x(5:8)+kron([0;1],eye(2,2))*u2;
xdot(9:12)= kron([0 1
    0 0],eye(2,2))*x(9:12)+kron([0;1],eye(2,2))*u3;

xdot=xdot';

end

