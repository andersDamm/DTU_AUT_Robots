data = importdata('irCalibRW.dat',' ');
V75 = [];
V65 = [];
V55 = [];
V45 = [];
V35 = [];
V25 = [];
V15 = [];

% Determine where the data is located
for i=1:size(data,1)
    if data(i,1) < 0.5
        
    elseif data(i,1)<1.5
        V75 = [V75; data(i,2:4)];
    elseif data(i,1)<2.5
        V65 = [V65; data(i,2:4)];
    elseif data(i,1)<3.5
        V55 = [V55; data(i,2:4)];
    elseif data(i,1)<4.5
        V45 = [V45; data(i,2:4)];
    elseif data(i,1)<5.5
        V35 = [V35; data(i,2:4)];
    elseif data(i,1)<6.5
        V25 = [V25; data(i,2:4)];
    elseif data(i,1)<7.5
        V15 = [V15; data(i,2:4)];
    end
end

means = [mean(V75);mean(V65);mean(V55);mean(V45);mean(V35);mean(V25);mean(V15)];
dist = [75; 65; 55; 45; 35; 25; 15]; %x-værdier
LeftFront = [dist means(:,1)]; %means er y-værdier
MiddleFront = [dist means(:,2)];
RightFront = [dist means(:,3)];

fun = @(x,xdata)x(1)./xdata + x(2);

x0 = [1600,1000];
x = lsqcurvefit(fun,x0,dist',means(:,2)')

times = linspace(dist(1),dist(end));
plot(dist',means(:,2)','ko',times,fun(x,times),'b-')
legend('Data','Fitted exponential')
title('Data and Fitted Curve')


