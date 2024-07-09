h= figure;

subplot(4,1,1)
plot(xm,dx,"LineWidth",1.2)
hold on
plot(xm,dy,"LineWidth",2.4)
hold off
grid on
title(['Posicion Angular ', posS + posD + "." +  posC], 'interpreter', 'latex','FontSize',20)
% xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$\psi(k)$" , 'interpreter', 'latex','FontSize',14)

subplot(4,1,2)
plot(xm,dv,"LineWidth",2.0)
title(['Velocidad ', velS + velD + "." +  velC], 'interpreter', 'latex','FontSize',20)
% xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$deg/s$" , 'interpreter', 'latex','FontSize',14)    


subplot(4,1,3)
plot(xm,dz,"LineWidth",2.0)
title(['Error ', errS + errD + "." +  errC], 'interpreter', 'latex','FontSize',20)
% xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$deg$" , 'interpreter', 'latex','FontSize',14)   
 
subplot(4,1,4)
plot(xm,do+400,"LineWidth",2.0)
title('Out PID', 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 