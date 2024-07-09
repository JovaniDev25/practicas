clc
clear
close all 

%m = modbus('tcpip', '192.168.100.47');
m = modbus('tcpip', '192.168.100.30');
m.Timeout = 3;
serverId = 1;

j = 2000;
xm = 1:1:j;
x = zeros(1,j);
dz = zeros(1,j);
dx = zeros(1,j);
dxp = zeros(1,j);
dy = zeros(1,j);
gain = zeros(1,j);
ml = zeros(1,j);
mr = zeros(1,j);
ner = zeros(1,j);
kp = zeros(1,j);
ki = zeros(1,j);
kd= zeros(1,j);

f = figure(WindowKeyPressFcn=@figureCallback);
for i = 1:j
        data = read(m, 'holdingregs', 2, 17, serverId, 'int16');
        if data(1) == 1
            refS = "";
        else 
            refS ="-";
        end
        refD = int2str(data(2));
        refC = int2str(abs(data(3)));
        dz(i) = str2num(refS + refD + "." +  refC);
        
        if data(4) == 1
            posS = "";
        else 
            posS ="-";
        end
        posD = int2str(data(5));
        posC = int2str(abs(data(6)));
        dx(i) = str2num(posS + posD + "." +  posC);

        if data(13) == 1
            velS = "";
        else 
            velS ="-";
        end
        velD = int2str(data(14));
        velC = int2str(abs(data(15)));
        dxp(i) = str2num(velS + velD + "." +  velC);


        
        if data(7) == 1
            errS = "";
        else 
            errS ="-";
        end
        errD = int2str(data(8));
        errC = int2str(abs(data(9)));
        dy(i) = str2num(errS + errD + "." +  errC);
        mr(i) = data(10);
        ml(i) = data(11);
        ner(i) = data(12);
        kp(i) = data(13);
        ki(i) = data(14);
        kd(i) = data(15);

        %ref = data(7);
        %kp = data(8);
        %refD = int2str(data(3));
        %refC = int2str(abs(data(4)));
        %dz(i) = str2num(posD + "." +  posC);
        %dy(i) = str2num(velD + "." +  velC);
        %ml(i) = data(5);
        %mr(i) = data(6);   
        %gain(i) = kp;


subplot(4,1,1)
plot(xm,dz,"LineWidth",1.2)
hold on
plot(xm,dx,"LineWidth",1.2)
hold off
title(['Posicion Angular ',num2str(dx(i))], 'interpreter', 'latex','FontSize',18)
xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$x(k)$" , 'interpreter', 'latex','FontSize',14)

subplot(4,1,2)
plot(xm,dy,"LineWidth",1.2)
title(['Error Angular ',num2str(dy(i))], 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$Xe(k)$" , 'interpreter', 'latex','FontSize',14)         

subplot(4,1,3)
plot(xm,ml,"LineWidth",1.2)
hold on 
plot(xm,mr,"LineWidth",1.2)
hold off
title('Motor R y L', 'interpreter', 'latex','FontSize',18)
xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$PWM$" , 'interpreter', 'latex','FontSize',14)

subplot(4,1,4)
% plot(xm,ner,"LineWidth",1.2)
% hold on 
plot(xm,dxp,"LineWidth",1.2)
% hold on 
% plot(xm,kd,"LineWidth",1.2)
% hold on 
% plot(xm,ki,"LineWidth",1.2)
% hold off
title('Ganancias', 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$PWM$" , 'interpreter', 'latex','FontSize',14)

% subplot(4,1,4)
% plot(xm,gain,"LineWidth",1.2)
% title('Ganancia Kp', 'interpreter', 'latex','FontSize',18)
% xlabel("$k$", 'interpreter', 'latex','FontSize',14)
% ylabel("$Kp$" , 'interpreter', 'latex','FontSize',14)

end

clear m
clear serverId