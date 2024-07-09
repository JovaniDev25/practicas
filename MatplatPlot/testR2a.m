clc
clear
close all 

%m = modbus('tcpip', '192.168.100.47');
%m = modbus('tcpip', '192.168.43.90');
m = modbus('tcpip', '192.168.100.30');
m.Timeout = 3;
serverId = 1;

j = 1000;
xm = 1:1:j;
x = zeros(1,j);
dz = zeros(1,j);
dx = zeros(1,j);
dy = zeros(1,j);

ml = zeros(1,j);
mr = zeros(1,j);

f = figure(WindowKeyPressFcn=@figureCallback);
for i = 1:j
        data = read(m, 'holdingregs', 2, 4, serverId, 'int16');
        ref = data(1);
        pos = data(2);
        kp = data(3);
        out = data(4);
        dx(i) = ref;
        x(i) = pos;
        dy(i) = kp;
        dz(i) = out;

%         posD = int2str(data(1));
%         posC = int2str(abs(data(2)));
%         
%         velD = int2str(data(3));
%         velC = int2str(abs(data(4)));
%         
%         dz(i) = str2num(posD + "." +  posC);
%         dy(i) = str2num(velD + "." +  velC);
%         ml(i) = data(5);
%         mr(i) = data(6);   
     
subplot(3,1,1)
plot(xm,x,"LineWidth",1.2)
hold on
plot(xm,dx,"LineWidth",1.2)
hold off
title('Posicion Angular', 'interpreter', 'latex','FontSize',18)
xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$x(k)$" , 'interpreter', 'latex','FontSize',14)

subplot(3,1,2)
plot(xm,dy,"LineWidth",1.2)
title('Ganancia Kp', 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$Kp$" , 'interpreter', 'latex','FontSize',14)    

subplot(3,1,3)
plot(xm,dz,"LineWidth",1.2)
title('Out PID', 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 



% subplot(4,1,3)
% plot(xm,ml,"LineWidth",1.2)
% title('Motor R', 'interpreter', 'latex','FontSize',18)
% xlabel("k", 'interpreter', 'latex','FontSize',14)
% ylabel("$PWM$" , 'interpreter', 'latex','FontSize',14)
% subplot(4,1,4)
% plot(xm,mr,"LineWidth",1.2)
% title('Motor L', 'interpreter', 'latex','FontSize',18)
% xlabel("$k$", 'interpreter', 'latex','FontSize',14)
% ylabel("$PWM$" , 'interpreter', 'latex','FontSize',14)

end

clear m
clear serverId