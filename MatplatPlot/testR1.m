clc
clear
close all 

%m = modbus('tcpip', '192.168.100.47');
m = modbus('tcpip', '192.168.43.90');

 m.Timeout = 3; 
serverId = 1;

j = 200;
x = zeros(1,j);
dz = zeros(1,j);
dy = zeros(1,j);

ml = zeros(1,j);
mr = zeros(1,j);

for i = 1:j
    data = read(m, 'holdingregs', 2, 6, serverId, 'int16');
    posD = int2str(data(1));
    posC = int2str(abs(data(2)));
    velD = int2str(data(3));
    velC = int2str(abs(data(4)));
    dz(i) = str2num(posD + "." +  posC);
    dy(i) = str2num(velD + "." +  velC);
    ml(i) = data(5);
    mr(i) = data(6);
    %display(dz(i));
end

xm = 1:1:j;
f1 = figure;
title('IMU', 'interpreter', 'latex','FontSize',20)

subplot(2,1,1)
plot(xm,dz,"LineWidth",1.2)
title('Posicion Angular', 'interpreter', 'latex','FontSize',18)
xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$x(k)$" , 'interpreter', 'latex','FontSize',14)

subplot(2,1,2)
plot(xm,dy,"LineWidth",1.2)
title('Velocidad Angular', 'interpreter', 'latex','FontSize',18)
%legend('a1','b1');
xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$v(k)$" , 'interpreter', 'latex','FontSize',14)

% hold on 
% plot(xm,dy)

f2 = figure;
subplot(1,2,1)
plot(xm,ml)
subplot(1,2,2)
plot(xm,mr)


clear m
clear serverId