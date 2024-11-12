clear; clc;
%% Import CSV file into the workspace
magDataRAW = readtable("magData.csv");
% Use curly braces instead of parenthesis to store numerica values
% from table
x = magDataRAW{:,"Var1"};
y = magDataRAW{:,"Var2"};
z = magDataRAW{:,"Var3"};
D = [x, y, z]; % Data

%% Get Calibration Coefficients and Offsets
[A, b, expmfs] = magcal(D);
% A is coefficient correction matrix
% b is offset matrix
% expmfs is strength of magnetic field (radius of sphere)

%% Correct Data
C = (D-b)*A;
% C is the corrected data

%% Plot Corrected Vs Raw Data
figure(1)
plot3(x(:),y(:),z(:),"LineStyle","none","Marker","X","MarkerSize",8)
hold on
grid(gca,"on")
plot3(C(:,1),C(:,2),C(:,3),"LineStyle","none","Marker","o", ...
      "MarkerSize",8,"MarkerFaceColor","r")
axis equal
xlabel("uT")
ylabel("uT")
zlabel("uT")
legend("Uncalibrated Samples","Calibrated Samples","Location","southoutside")
title("Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off