clear
% 
% for j=1:100000
    a1=random('Normal',0,1,1,2);
    A1=a1'*a1;
    a2=random('Normal',0,1,1,2);
    A2=a2'*a2;
    b1=random('Normal',0,1,1,2);
    B1=b1'*b1;
    b2=random('Normal',0,1,1,2);
    B2=b2'*b2;
    c1 =random('Normal',0,1,1,2);
    C1=c1'*c1;
    c2 =random('Normal',0,1,1,2);
    C2=c2'*c2;
%     L=10000*[A1+A2 -A1 -A2
%        -B1 B1+B2 -B2
%        -C1 -C2   C1+C2];
%     L=10000*[A1 zeros(2,2) -A1
%        -B1 B1+B1 -B1
%        -C1 -C1   C1+C1];
   L=[A1 zeros(2,2) -A1
       -A1 A1+A1 -A1
       zeros(2,2) -A1   A1];
%    L=[A1+A1 -A1 -A1
%        zeros(2,2) A1 -A1
%        -A1 zeros(2,2) A1];
%      L=100000*[A1 -A1
%                 -C1   C1];
    D1=random('Normal',0,1,2,2);
    D2=random('Normal',0,1,2,2);
    D3=random('Normal',0,1,2,2);
%    D=10000*blkdiag(D1'*D1,D1'*D1,D1'*D1);
    D=blkdiag(D1'*D1,D2'*D2,D3'*D3);
    
%    [eig(D) eig(L) eig(D*L)]
    value=eig(D*L)
%     E1 =rand(1);
%     E2 =rand(1);
%     E = [E1^2 0;0 E2];
%     value = E*A1;
    mineig=min(real(value))
%     if mineig<-0.000001
%             mineig
%         break
%     end
% end
% 
% syms A1 A2 B1 B2 C1 C2
% L=[A1+A2 -A1 -A2
%    -B1 B1+B2 -B2
%    -C1 -C2   C1+C2];
% eig(L)

