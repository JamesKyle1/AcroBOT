clear all
close all
clc


%% =====================================================================
%  ====================  COM Parameter Change  =========================
%  =====================================================================

n = 729;
nn = n/9;

t = cell(1,n);
th1 = cell(1,n);
th2 = cell(1,n);
th3 = cell(1,n);
th1d = cell(1,n);
th2d = cell(1,n);
th3d = cell(1,n);

comX = cell(1,n);
comY = cell(1,n);

m = [];
l = [];
c = [];

maxY = [];


for i = 1:n
    fileName = "L_1_COM_" + i + ".txt";
    filePath = "Mar_23_2023_Single_Length_Set_3/" + fileName;

    currParams = readmatrix(filePath,"Range",[3 1 3 9]);
    l = [l; currParams(1:3)];
    c = [c; currParams(4:6)];
    m = [m; currParams(7:9)];
    
    data = readmatrix(filePath,"NumHeaderLines",5);

    q1 = data(:,2);
    q2 = data(:,3);
    q3 = data(:,4);
    
    t{i} = data(:,1);
    th1{i} = data(:,2);
    th2{i} = data(:,3);
    th3{i} = data(:,4);
    th1d{i} = data(:,5);
    th2d{i} = data(:,6);
    th3d{i} = data(:,7);
    
    comX{i} = (m(i,1)*c(i,1)*cos(q1) + m(i,2)*(l(i,1)*cos(q1) + c(i,2)*cos(q1+q2)) + m(i,3)*(l(i,1)*cos(q1) + l(i,2)*cos(q1+q2) + c(i,3)*cos(q1+q2+q3)))/sum(m(i,:));
    comY{i} = (m(i,1)*c(i,1)*sin(q1) + m(i,2)*(l(i,1)*sin(q1) + c(i,2)*sin(q1+q2)) + m(i,3)*(l(i,1)*sin(q1) + l(i,2)*sin(q1+q2) + c(i,3)*sin(q1+q2+q3)))/sum(m(i,:));


    maxY = [maxY; max(comY{i})];

end


%% 
figure();
hold on
for i = 1:81
    plot(t{i},th1{i});
end
hold off
ylabel('$\theta [rad]$');
xlabel('Time [s]');
title('$\theta_1 $')

figure();
hold on
for i = 1:81
    plot(t{i},th2{i});
end
hold off
ylabel('$\theta [rad]$');
xlabel('Time [s]');
title('$\theta_2 $')

figure();
hold on;
for i = 1:81
    plot(t{i},th3{i});
end
hold off;
ylabel('$\theta [rad]$');
xlabel('Time [s]');
title('$\theta_3 $')



%% Max height of COM

colorSort = [maxY c parula(n)];
colorSort = sortrows(colorSort,1,'ascend');

figure();
hold on;
maxHeight = max(maxY);
minHeight = min(maxY);
range = maxHeight - minHeight;
for i = 1:n
    plot3(colorSort(i,2),colorSort(i,3),colorSort(i,4),'.','MarkerSize',50,'Color',colorSort(i,5:7));
end
hold off;
view(3)
xlabel('$C_1 [m]$','interpreter','latex');
ylabel('$C_2 [m]$');
zlabel('$C_3 [m]$');
colormap('parula');
%% 


maxYMat = reshape(maxY,9,nn)';

figure();
tiledlayout('flow');
for i = 1:9
    nexttile();
    hold on;
    plot(c(9*(i-1) + 1:9*i,3),maxYMat(9*(i-1) + 1:9*i,:),'linewidth',3);
    xlabel('$COM \, \, 3 \, \, [m]$','Interpreter','latex');
    ylabel('Max Heigh [m]');
    title("COM 1 = " + c(81*(i-1) + 1,1) + " m");
%     legend("COM 2 = " + c(5,2) + " m","COM 2 = " + c(10,2) + " m",...
%         "COM 2 = " + c(15,2) + " m","COM 2 = " + c(20,2) + " m",...
%         "COM 2 = " + c(25,2) + " m",'location','nw','fontsize',14);
end


dataSortMat = [c maxY];
dataSortMat = sortrows(dataSortMat,[3 2],'ascend');
c = flip(dataSortMat(:,1:3),2);
maxY2 = dataSortMat(:,4);
maxYMat2 = reshape(maxY2,9,nn)';

figure();
tiledlayout('flow');
for i = [1 5 9]
    nexttile();
    hold on;
    plot(c(9*(i-1) + 1:9*i,3),maxYMat2(9*(i-1) + 1:9*i,:),'linewidth',3);
    xlabel('$COM \, \, 1 \, \, [m]$','Interpreter','latex');
    ylabel('Max Heigh [m]');
    title("COM 3 = " + c(nn*(i-1) + 1,1) + " m");
    legend("COM 2 = " + c(9,2) + " m","COM 2 = " + c(18,2) + " m",...
        "COM 2 = " + c(27,2) + " m","COM 2 = " + c(36,2) + " m",...
        "COM 2 = " + c(45,2) + " m","COM 2 = " + c(54,2) + " m",...
        "COM 2 = " + c(63,2) + " m","COM 2 = " + c(72,2) + " m",...
        "COM 2 = " + c(81,2) + " m",'location','nw','fontsize',14);
end

nexttile();
hold on;
maxHeight = max(maxY);
minHeight = min(maxY);
range = maxHeight - minHeight;
for i = 1:n
    plot3(colorSort(i,2),colorSort(i,3),colorSort(i,4),'.','MarkerSize',50,'Color',colorSort(i,5:7));
end
hold off;
view(3)
xlabel('$C_1 [m]$','interpreter','latex');
ylabel('$C_2 [m]$');
zlabel('$C_3 [m]$');
colormap('parula');


% [X, Y] = meshgrid(c(1:5:25,2),c(1:5,3));
% surf(X,Y);

%%
% n = 120;
% dt = 0.01;
% x = [t{n} th1{n} th2{n} th3{n} th1d{n} th2d{n} th3d{n}];
% 
% animateAcroBOT(x, dt, gymnastSYS);


%%
clear all;
close all;
clc

%% =====================================================================
%  ====================  COM Parameter Change  =========================
%  =====================================================================

folderName = "Apr_2_2023_Single_COM_Set_2/";
n = size(dir([folderName + '/*.txt']),1);

t = cell(1,n);
th1 = cell(1,n);
th2 = cell(1,n);
th3 = cell(1,n);
th1d = cell(1,n);
th2d = cell(1,n);
th3d = cell(1,n);

comX = cell(1,n);
comY = cell(1,n);

m = [];
l = [];
c = [];

maxY = [];


for i = 1:n
    fileName = "L_" + i + "_COM_1.txt";
    filePath = folderName + fileName;

    currParams = readmatrix(filePath,"Range",[3 1 3 9]);
    l = [l; currParams(1:3)];
    c = [c; currParams(4:6)];
    m = [m; currParams(7:9)];
    
    data = readmatrix(filePath,"NumHeaderLines",5);

    q1 = data(:,2);
    q2 = data(:,3);
    q3 = data(:,4);
    
    t{i} = data(:,1);
    th1{i} = data(:,2);
    th2{i} = data(:,3);
    th3{i} = data(:,4);
    th1d{i} = data(:,5);
    th2d{i} = data(:,6);
    th3d{i} = data(:,7);
    
    comX{i} = (m(i,1)*c(i,1)*cos(q1) + m(i,2)*(l(i,1)*cos(q1) + c(i,2)*cos(q1+q2)) + m(i,3)*(l(i,1)*cos(q1) + l(i,2)*cos(q1+q2) + c(i,3)*cos(q1+q2+q3)))/sum(m(i,:));
    comY{i} = (m(i,1)*c(i,1)*sin(q1) + m(i,2)*(l(i,1)*sin(q1) + c(i,2)*sin(q1+q2)) + m(i,3)*(l(i,1)*sin(q1) + l(i,2)*sin(q1+q2) + c(i,3)*sin(q1+q2+q3)))/sum(m(i,:));


    maxY = [maxY; max(comY{i})];

end


%% 
figure();
hold on
for i = 1:n
    plot(t{i},th1{i});
end
hold off
ylabel('$\theta [rad]$','interpreter','latex');
xlabel('Time [s]');
title('$\theta_1 $','interpreter','latex')

figure();
hold on
for i = 1:n
    plot(t{i},th2{i});
end
hold off
ylabel('$\theta [rad]$','interpreter','latex');
xlabel('Time [s]');
title('$\theta_2 $','interpreter','latex')

figure();
hold on;
for i = 1:n
    plot(t{i},th3{i});
end
hold off;
ylabel('$\theta [rad]$','interpreter','latex');
xlabel('Time [s]');
title('$\theta_3 $','interpreter','latex')



%% Max height of COM

colorSort = [maxY l];
colorSort = sortrows(colorSort,1,'ascend');
colorSort = [colorSort parula(n)];

figure();
hold on;
maxHeight = max(maxY);
minHeight = min(maxY);
range = maxHeight - minHeight;
for i = 1:n
    plot3(colorSort(i,2),colorSort(i,3),colorSort(i,4),'.','MarkerSize',50,'Color',colorSort(i,5:7));
end
hold off;
view(3)
xlabel('$L_1 [m]$','interpreter','latex');
ylabel('$L_2 [m]$','interpreter','latex');
zlabel('$L_3 [m]$','interpreter','latex');
colormap('parula');


figure();
plot(t{end},th3{end});
ylabel('$\theta [rad]$');
xlabel('Time [s]');
title('$\theta_3 $')
%% 


maxYMat = reshape(maxY,9,nn)';

figure();
tiledlayout('flow');
for i = 1:9
    nexttile();
    hold on;
    plot(l(9*(i-1) + 1:9*i,3),maxYMat(9*(i-1) + 1:9*i,:),'linewidth',3);
    xlabel('$COM \, \, 3 \, \, [m]$','Interpreter','latex');
    ylabel('Max Heigh [m]');
    title("COM 1 = " + c(81*(i-1) + 1,1) + " m");
%     legend("COM 2 = " + c(5,2) + " m","COM 2 = " + c(10,2) + " m",...
%         "COM 2 = " + c(15,2) + " m","COM 2 = " + c(20,2) + " m",...
%         "COM 2 = " + c(25,2) + " m",'location','nw','fontsize',14);
end


dataSortMat = [c maxY];
dataSortMat = sortrows(dataSortMat,[3 2],'ascend');
c = flip(dataSortMat(:,1:3),2);
maxY2 = dataSortMat(:,4);
maxYMat2 = reshape(maxY2,9,nn)';

figure();
tiledlayout('flow');
for i = [1 5 9]
    nexttile();
    hold on;
    plot(c(9*(i-1) + 1:9*i,3),maxYMat2(9*(i-1) + 1:9*i,:),'linewidth',3);
    xlabel('$COM \, \, 1 \, \, [m]$','Interpreter','latex');
    ylabel('Max Heigh [m]');
    title("COM 3 = " + c(nn*(i-1) + 1,1) + " m");
    legend("COM 2 = " + c(9,2) + " m","COM 2 = " + c(18,2) + " m",...
        "COM 2 = " + c(27,2) + " m","COM 2 = " + c(36,2) + " m",...
        "COM 2 = " + c(45,2) + " m","COM 2 = " + c(54,2) + " m",...
        "COM 2 = " + c(63,2) + " m","COM 2 = " + c(72,2) + " m",...
        "COM 2 = " + c(81,2) + " m",'location','nw','fontsize',14);
end

nexttile();
hold on;
maxHeight = max(maxY);
minHeight = min(maxY);
range = maxHeight - minHeight;
for i = 1:n
    plot3(colorSort(i,2),colorSort(i,3),colorSort(i,4),'.','MarkerSize',50,'Color',colorSort(i,5:7));
end
hold off;
view(3)
xlabel('$C_1 [m]$','interpreter','latex');
ylabel('$C_2 [m]$');
zlabel('$C_3 [m]$');
colormap('parula');


% [X, Y] = meshgrid(c(1:5:25,2),c(1:5,3));
% surf(X,Y);




