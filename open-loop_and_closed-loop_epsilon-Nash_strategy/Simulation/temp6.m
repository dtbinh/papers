clear;
s=rand(1,3).^2;
sdiag=diag(s);
df=rand(1,3).^2;
v=rand(3,3);
DF=inv(v)*diag(df)*v;

eig(sdiag*DF)