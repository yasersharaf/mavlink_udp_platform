clear
close all

for i = 2
    output = load('log.txt');
    output = output((1+i):3:end,:);
    output = output(1900:end,:);
    output(:,1)=[];
    output(:,1)=output(:,2);
    % output=output(50:end,:);
%     zeroInd = find(output(:,3)<0.1);
%     output(zeroInd,:)=[];
    upto = size(output,1);
    output(:,2)=(output(:,2)-output(1,2))/1e3;
    outputF = output;
    for i =2:length(outputF)
    outputF(i,[3,5])=outputF(i,[3,5])/min(200,i)+(min(200,i)-1)/min(200,i)*outputF(i-1,[3,5]);
    end
    figure(1);
    scatter(output(1:upto,2),output(1:upto,5))
    hold on
    % plot(outputF(1:upto,2),outputF(1:upto,5),'LineWidth',3)
    xlabel('time(s)')
    ylabel('voltage(v)')
    legend('measured','filtered')
    axis([0 500 0 13])
    axis([0 500 8 13])
    axis([0 500 6 13])
    figure(2)
    plot(output(1:upto,2),output(1:upto,3))
    hold on
%     scatter(outputF(1:upto,2),outputF(1:upto,3),'LineWidth',3)
    xlabel('time(s)')
    ylabel('pilot_thorttle')
    legend('measured','filtered')
    ylabel('pilot thorttle')
    figure(3)
    scatter(output(750:upto,5),output(750:upto,3),'LineWidth',2)
    hold on
%     scatter(outputF(750:upto,5),outputF(750:upto,3),'LineWidth',2)
    fplot(@(z) -0.04894*z + 0.8600,[9 12])
    fplot(@(z) -0.04497*z + 0.7885,[9 12])
    fplot(@(z) -0.04497*z + 0.6914,[9 12])
    figure(4)
    scatter(output(1:upto,2),output(1:upto,4))
% 
%     figure(5)
%     scatter(output(1:upto,2),output(1:upto,6)/100)
%     scatter(output(1:upto,2),output(1:upto,7)/100)

    figure(6)
    plot(output(1:upto,2),output(1:upto,10))
    hold on 
    plot(output(1:upto,2),output(1:upto,11))

    
    
end
