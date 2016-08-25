%for i=1:1000000

d1=rand(1,2);
d2=rand(1,2);
d3=rand(1,2);
D1=d1'*d1;
D2=d2'*d2;
D3=d3'*d3;
D=blkdiag(D1,D2,D3);

L=[2 -1 -1;-1 1 0;0 -1 1];

f=rand(1,2);
F=f'*f;

[V E]=eig(D*kron(L,F))
eig(L)
[eig(D1*F) eig(D2*F) eig(D3*F)]

% a1=rand(1,2);
% A1=a1'*a1;
% a2=rand(1,2);
% A2=a2'*a2;
% 
% e=eig(A1*A2)
% h=eig(A2*A1)
% sqrtA1=sqrtm(A1);
% f=eig(sqrtA1*A2*sqrtA1)
% if min(e)<-0.00001
%     min(e)
% end
%end

% s1=rand(1,1)^2;
% s2=rand(1,1)^2;
% s3=rand(1,1)^2;
% s4=rand(1,1)^2;
% s5=rand(1,1)^2;
% s6=rand(1,1)^2;
% L=[s1+s2 -s1 -s2;-s3 s3+s4 -s4;-s5 -s6 s5+s6];
% [V,J]=jordan(L);
% J