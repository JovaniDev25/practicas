h= figure;
j = 450;
d1= zeros(1,j);
d2 = zeros(1,j);
for i = 1:j
 d1(i) = 11;
d2(i) = 20;
end 
subplot(3,1,1)
plot(xm,dx,"LineWidth",1.2, 'DisplayName','$\psi_d$')
hold on
plot(xm,dy,"LineWidth",2.4, 'DisplayName','$\psi$')
hold off
title(['Posicion Angular ', posS + posD + "." +  posC], 'interpreter', 'latex','FontSize',24)
grid on
% xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$\psi(k)$" , 'interpreter', 'latex','FontSize',24)

subplot(3,1,2)
plot(xm,dk,"LineWidth",2.4,DisplayName='$Kp$')
hold on 
plot(xm,d1,"LineWidth",2.4,DisplayName='$Kd$')
hold on
plot(xm,d2,"LineWidth",2.4,DisplayName='$Ki$')
title('Ganancias', 'interpreter', 'latex','FontSize',24)
% set(legend2,'Interpreter','latex');
% xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$K$" , 'interpreter', 'latex','FontSize',14)    


subplot(3,1,3)
plot(xm,dz,"LineWidth",2.4)
title(['Error ', errS + errD + "." +  errC], 'interpreter', 'latex','FontSize',28)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$deg$" , 'interpreter', 'latex','FontSize',14)   
%  
% subplot(4,1,4)
% plot(xm,do,"LineWidth",1.2)
% title('Out PID', 'interpreter', 'latex','FontSize',18)
% xlabel("$k$", 'interpreter', 'latex','FontSize',14)
% ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 