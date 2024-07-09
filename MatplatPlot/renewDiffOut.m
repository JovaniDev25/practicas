m = figure


subplot(4,2,1)
 plot(xm,do+400,"LineWidth",2.2, Color="red")
 grid on
 title('Out PID 1 ', 'interpreter', 'latex','FontSize',18)
%  xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,2)
 plot(xm,df-1000,"LineWidth",2.2, Color="red")
  grid on
 title('Out PID 2', 'interpreter', 'latex','FontSize',18)
%  xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,3)
 plot(xm,do+400,"LineWidth",2.2)
  grid on
 title('Out PID 3 ', 'interpreter', 'latex','FontSize',18)
%  xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,4)
 plot(xm,df-1000,"LineWidth",2.2)
  grid on
 title('Out PID 4', 'interpreter', 'latex','FontSize',18)
%  xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,5)
 plot(xm,do+400,"LineWidth",2.2)
  grid on
 title('Out PID 5 ', 'interpreter', 'latex','FontSize',18)
%  xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,6)
 plot(xm,df-1000,"LineWidth",2.2)
  grid on
 title('Out PID 6', 'interpreter', 'latex','FontSize',18)
%  xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,7)
 plot(xm,do+400,"LineWidth",1.2)
  grid on
 title('Out PID 7 ', 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,8)
 plot(xm,df-1000,"LineWidth",1.2)
  grid on
 title('Out PID 8', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 