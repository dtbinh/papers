A1=random('normal',0,1,2,2);
A=A1'*A1;

D11=random('normal',0,1,2,2);
D1=D11'*D11;
D21=random('normal',0,1,2,2);
D2=D21'*D21;
D31=random('normal',0,1,2,2);
D3=D31'*D31;

L=[2*A       -A -A
   zeros(2,2) A -A
   -A zeros(2,2) A];
D=blkdiag(D1,D2,D3);
DL=D*L;
eig(DL)

[eig(2*D1*A) eig(D2*A) eig(D3*A)]
