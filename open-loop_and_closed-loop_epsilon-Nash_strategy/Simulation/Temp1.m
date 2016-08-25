clear;clc; 
syms tf t L11 L12 L21 L22
A1=[0 1; 0 0];A2=A1;
B1=[0;1];B2=B1;
Phi1=expm(A1*(tf-t));
Phi2=Phi1;
Phi10=expm(A1*tf);
Phi20=Phi10;
D1=int(Phi1*B1*B1'*Phi1',t,0,tf);
D2=D1;
D=blkdiag(D1,D2);

L=[L11 L12;L21 L22];
Phi0=blkdiag(Phi10,Phi20);


C1=inv(Phi0)*(eye(4,4)+D*kron(L,eye(2,2)));

temp=C1;
C1(2,:)=temp(3,:);
C1(3,:)=temp(2,:);
C1(4,:)=temp(4,:);
temp=C1;
C1(:,2)=temp(:,3);
C1(:,3)=temp(:,2);
C1(:,4)=temp(:,4);
C2=[eye(2,2)-tf^3/6*L -tf*eye(2,2)-tf^2/2*L;
    tf^2/2*L            eye(2,2)+tf*L];

% C3=inv(Phi0)*D