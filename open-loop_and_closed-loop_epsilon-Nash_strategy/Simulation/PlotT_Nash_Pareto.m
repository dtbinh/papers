p1=plot(Nash(:,1),Nash(:,2),'-b','LineWidth',2);hold
plot(Nash(:,5),Nash(:,6),'-b','LineWidth',2);
plot(Nash(:,9),Nash(:,10),'-b','LineWidth',2);
p2=plot(Pareto(:,1),Pareto(:,2),'-r','LineWidth',2);
plot(Pareto(:,5),Pareto(:,6),'-r','LineWidth',2);
plot(Pareto(:,9),Pareto(:,10),'-r','LineWidth',2);
xlabel('x');
ylabel('y')
legend([p1 p2],{'Nash','Pareto'})
xlabel('x');ylabel('y');