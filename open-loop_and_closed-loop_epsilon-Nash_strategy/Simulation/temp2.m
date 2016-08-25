clear
D=diag([5 4 3 2 10 56]);
L=[1 -1 0;0 1 -1;-1 0 1];
F=[3 1;1 3];

X=D*kron(L,F);

% df1=eig(D(1:2,1:2)*F);
% df2=eig(D(3:4,3:4)*F);
%Leig=eig(L);
%[Leig(1)*df1;Leig(2)*df2]

s=eig((D(1:2,1:2)+D(3:4,3:4)+D(5:6,5:6))*F)
eig(X)