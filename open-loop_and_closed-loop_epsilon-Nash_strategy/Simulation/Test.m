function [ c] = Test( input_args )
%TEST Summary of this function goes here
%   Detailed explanation goes here
a=kron([37 -9 -9
    -9 19 0
    -9 0 19],eye(2,2));
b=[0;0;-2;0;2;0]+9*[0;-0.25;-0.1;-0.125;0.1;-0.125];

c=inv(a)*b;

x=[0;-0.25]-(4*[0;-0.1165]-[-0.1526;-0.1144]-[0.1526;-0.1144]);



end

