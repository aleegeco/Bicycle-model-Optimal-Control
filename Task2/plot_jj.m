figure
hold on
subplot(2,1,1)
plot(1:length(JJ_fixed),log(JJ_fixed),"LineWidth",1.5)
xlabel("Iteration")
ylabel("$\log(J)$","interpreter","latex")
title("Constant")
grid on,zoom on
subplot(2,1,2)
plot(1:length(JJ_arm),log(JJ_arm),"r","LineWidth",1.5)
xlabel("Iteration")
ylabel("$\log(J)$","interpreter","latex")
title("Armijo")
grid on, zoom on
hold off