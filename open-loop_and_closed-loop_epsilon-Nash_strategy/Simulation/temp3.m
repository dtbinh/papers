syms l1 l2 l3 l4 l5 l6 lambda

L=[l1+l2 -l1 -l2
   -l3 l3+l4 -l4
   -l5 -l6 l5+l6];
det(lambda*eye(3,3)-L)/lambda