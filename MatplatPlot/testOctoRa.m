clc
clear
close all 

%m = modbus('tcpip', '192.168.100.47');
%m = modbus('tcpip', '192.168.43.90');

m = modbus('tcpip', '192.168.240.137');
% 192.168.228.137


% m = modbus('tcpip','192.168.240.130');

m.Timeout = 3;
serverId = 1;

j = 100;
xm = 1:1:j;
x = zeros(1,j);
dz = zeros(1,j);
dx = zeros(1,j);
dy = zeros(1,j);
dv = zeros(1,j);
do = zeros(1,j);
df = zeros(1,j);

ml = zeros(1,j);
mr = zeros(1,j);

f = figure(WindowKeyPressFcn=@figureCallback);
for i = 1:j
        data = read(m, 'holdingregs', 2, 19, serverId, 'int16');
       
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

 subplot(4,2,1)
 plot(xm,do,"LineWidth",2.1, Color="red")
 title('Out PID 1 ', 'interpreter', 'latex','FontSize',18)
 xlabel("$k$", 'interpreter', 'latex','FontSize',14)
 ylabel("$pmw$" , 'interpreter', 'latex','FontSize',14) 

 subplot(4,2,2)
 plot(xm,df,"LineWidth",2.1, Color="red")
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

end

clear m
clear serverId