clc
clear

simin  = 0;
numSteps =0;
for i = 1:1001 
    numSteps = numSteps+1;
    pause(0.01);
    time = 0.01*numSteps;
    data = sin(2*pi/3*time);
    simin = timeseries(data,time)*10;
end
