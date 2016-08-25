s=[1 0 0;1 1 0;1 1 1];
syms L1 L2 L3 L4 L5 L6
L=[L1+L2 -L1 -L2;-L3 L3+L4 -L4;-L5 -L6 L5+L6];
[a,b]=eig(L)