clc
clear
close all 

%p9 d11 i20 o1200

%m = modbus('tcpip', '192.168.43.90');
m = modbus('tcpip', '192.168.100.30');




m.Timeout = 3;
serverId = 1;

j =450;
xm = 1:1:j;
x = zeros(1,j);
dz = zeros(1,j);
dx = zeros(1,j);
dy = zeros(1,j);
dv = zeros(1,j);
do = zeros(1,j);
df = zeros(1,j);
dkp = zeros(1,j);
dki = zeros(1,j);
dkd = zeros(1,j);


ml = zeros(1,j);
mr = zeros(1,j);

f = figure(WindowKeyPressFcn=@figureCallback);
for i = 1:j
        data = read(m, 'holdingregs', 2, 28, serverId, 'int16');
        
        if data(1) > 0
          setS = "";
        else
          setS = "-";
        end
        setD = int2str(abs(data(2)));
        setC = int2str(abs(data(3)));

        if data(4) > 0
          posS = "";
        else
          posS = "-";
        end
        posD = int2str(abs(data(5)));
        posC = int2str(abs(data(6)));

        if data(7) > 0
          errS = "";
        else
          errS = "-";
        end
        errD = int2str(abs(data(8)));
        errC = int2str(abs(data(9)));


        if data(10) > 0
          velS = "";
        else
          velS = "-";
        end
        velD = int2str(abs(data(11)));
        velC = int2str(abs(data(12)));

        if data(13) > 0
          outS = "";
        else
          outS = "-";
        end
        outD = int2str(abs(data(14)));
        outC = int2str(abs(data(15)));

        if data(16) > 0
          offS = "";
        else
          offS = "-";
        end
        offD = int2str(abs(data(17)));
        offC = int2str(abs(data(18)));

        doStr =  str2num(outS + outD + "." +  outC);
        dfStr =  str2num(offS + offD + "." +  offC);

        do(i) = dfStr + doStr;
        df(i) = dfStr - doStr ;

        if data(19) > 0
          kpS = "";
        else
          kpS = "-";
        end
        kpD = int2str(abs(data(20)));
        kpC = int2str(abs(data(21)));

       if data(22) > 0
          kiS = "";
        else
          kiS = "-";
        end
        kiD = int2str(abs(data(23)));
        kiC = int2str(abs(data(24)));


       if data(25) > 0
          kdS = "";
        else
          kdS = "-";
        end
        kdD = int2str(abs(data(26)));
        kdC = int2str(abs(data(27)));



        dx(i) =  str2num(setS + setD + "." +  setC);
        dy(i) =  str2num(posS + posD + "." +  posC);
        dz(i) =  str2num(errS + errD + "." +  errC);
        dv(i) =  str2num(velS + velD + "." +  velC);
        do(i) =  str2num(outS + outD + "." +  outC);
        dkp(i) =  str2num(kpS + kpD + "." +  kpC);
        dki(i) =  str2num(kiS + kiD + "." +  kiC);
        dkd(i) =  str2num(kdS + kdD + "." +  kdC);
 
subplot(4,1,1)
plot(xm,dx,"LineWidth",1.2)
hold on
plot(xm,dy,"LineWidth",1.2)
hold off
title(['Posicion Angular ', posS + posD + "." +  posC], 'interpreter', 'latex','FontSize',18)
xlabel("k", 'interpreter', 'latex','FontSize',14)
ylabel("$\psi(k)$" , 'interpreter', 'latex','FontSize',14)

subplot(4,1,2)
plot(xm,dkp,"LineWidth",1.2)
hold on 
plot(xm,dki,"LineWidth",1.2)
hold on
plot(xm,dkd,"LineWidth",1.2)
hold off
tit = 'Kp ' +  kpS + kpD + "." +  kpC + '  Ki '+  kiS + kiD + "." +  kiC+'  Kd ' +  kdS + kdD + "." +  kdC;
title(tit, 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$g$" , 'interpreter', 'latex','FontSize',14)    


subplot(4,1,3)
plot(xm,dz,"LineWidth",1.2)
title(['Error ', errS + errD + "." +  errC], 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$deg$" , 'interpreter', 'latex','FontSize',14)   
 
subplot(4,1,4)
plot(xm,do,"LineWidth",1.2)
title('Out PID', 'interpreter', 'latex','FontSize',18)
xlabel("$k$", 'interpreter', 'latex','FontSize',14)
ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

end

g = figure;

subplot(4,2,1)
 plot(xm,do+300,"LineWidth",2.1, Color="red")
 title('Out PID 1 ', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,2)
 plot(xm,df-1300,"LineWidth",2.1, Color="red")
 title('Out PID 2', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,3)
 plot(xm,do,"LineWidth",1.2)
 title('Out PID 3 ', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,4)
 plot(xm,df,"LineWidth",1.2)
 title('Out PID 4', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,5)
 plot(xm,do,"LineWidth",1.2)
 title('Out PID 5 ', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,6)
 plot(xm,df,"LineWidth",1.2)
 title('Out PID 6', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,7)
 plot(xm,do,"LineWidth",1.2)
 title('Out PID 7 ', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,8)
 plot(xm,df,"LineWidth",1.2)
 title('Out PID 8', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

save('testSave')

clear m
clear serverId