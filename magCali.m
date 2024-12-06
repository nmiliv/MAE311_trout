clear; clc;

%% ICM Calibration
%% Import CSV file of calibration test into the workspace
magDataRAW_ICM = readtable("caliData_ICM_mag.csv");
% Use curly braces instead of parenthesis to store numerical values
% from table
xICM = magDataRAW_ICM{:,"Var1"};
yICM = magDataRAW_ICM{:,"Var2"};
zICM = magDataRAW_ICM{:,"Var3"};
D_ICM = [xICM, yICM, zICM]; % Data

%% Get Calibration Coefficients and Offsets
[A_ICM, b_ICM, expmfs_ICM] = magcal(D_ICM);
% A is coefficient correction matrix
% b is offset matrix
% expmfs is strength of magnetic field (radius of sphere)

% Create ideal sphere
[Xideal, Yideal, Zideal] = sphere(25);
Xideal = Xideal*expmfs_ICM;
Yideal = Yideal*expmfs_ICM;
Zideal = Zideal*expmfs_ICM;


%% Correct Data
C_ICM = (D_ICM-b_ICM)*A_ICM;
% C is the corrected data

%% Plot Raw Data Vs. Ideal Sphere
figure(1); clf(1);
plot3(xICM(:),yICM(:),zICM(:),"LineStyle","none","Marker","X","MarkerSize",8)
hold on
grid(gca,"on")
mesh(Xideal, Yideal, Zideal, 'EdgeAlpha', 1, 'FaceAlpha', 0.2)
axis equal
xlabel("X [uT]")
ylabel("Y [uT]")
zlabel("Z [uT]")
legend("Uncalibrated Data","Ideal Sphere","Location","southoutside")
title("ICM: Uncalibrated vs Ideal Sphere" + newline + "Magnetometer Measurements")
hold off

%% Plot Raw Data Vs. Ideal Sphere
figure(2); clf(2);
plot3(C_ICM(:,1),C_ICM(:,2),C_ICM(:,3),"LineStyle","none","Marker","o", ...
      "MarkerSize",8,"MarkerFaceColor","r", "MarkerEdgeColor","k")
hold on
grid(gca,"on")
mesh(Xideal, Yideal, Zideal, 'EdgeAlpha', 1, 'FaceAlpha', 0.2)
axis equal
xlabel("X [uT]")
ylabel("Y [uT]")
zlabel("Z [uT]")
legend("Calibrated Data","Ideal Sphere","Location","southoutside")
title("ICM: Calibrated vs Ideal Sphere" + newline + ...
        "Magnetometer Measurements")
hold off

%% Plot Corrected Vs Raw Data
figure(3); clf(3);
plot3(xICM(:),yICM(:),zICM(:),"LineStyle","none","Marker","X", ...
                                "MarkerSize",8)
hold on
grid(gca,"on")
plot3(C_ICM(:,1),C_ICM(:,2),C_ICM(:,3),"LineStyle","none","Marker","o", ...
      "MarkerSize",8,"MarkerFaceColor","r", "MarkerEdgeColor","k")
mesh(Xideal, Yideal, Zideal, 'EdgeAlpha', 1, 'FaceAlpha', 0.2)
axis equal
xlabel("X [uT]")
ylabel("Y [uT]")
zlabel("Z [uT]")
legend("Uncalibrated Samples","Calibrated Samples", "Ideal Sphere", ...
        "Location","southoutside")
title("ICM: Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off

%% ICM Flight Data Adjusted
%% Import CSV file of calibration test into the workspace
magDataRAW_Flight = readtable("flightData_ICM_mag.csv");
% Use curly braces instead of parenthesis to store numerical values
% from table
xFlight = magDataRAW_Flight{:,"Var1"};
yFlight = magDataRAW_Flight{:,"Var2"};
zFlight = magDataRAW_Flight{:,"Var3"};
D_Flight = [xFlight, yFlight, zFlight]; % Data

%% Correct Data
C_Flight = (D_Flight-b_ICM)*A_ICM;
% C is the corrected data

%% Plot Corrected Flight Data Vs Raw Flight Data
% adjust axis to match rocket orientation
% y on mag = z vertical axis
figure(4); clf(4);
plot3(zFlight(:),xFlight(:),yFlight(:),"LineStyle","none","Marker","X","MarkerSize",8)
hold on
grid(gca,"on")
plot3(C_Flight(:,3),C_Flight(:,1),C_Flight(:,2),"LineStyle","none","Marker","o", ...
      "MarkerSize",8,"MarkerFaceColor","r")
axis equal

% plot rocket orientation
quiver3(0,0,0,0,0,150,'LineWidth',2)

% plot ideal sphere
mesh(Xideal, Yideal, Zideal, 'EdgeAlpha', 1, 'FaceAlpha', 0.2)

xlabel("Z [uT]")
ylabel("X [uT]")
zlabel("Y [uT]")
legend("Uncalibrated Samples","Calibrated Samples", "Rocket Orientation", ...
    "Ideal Sphere", "Location","southoutside")
title("Flight: Uncalibrated vs Calibrated" + newline + "Magnetometer Measurements")
hold off

%% Statistical Analysis

% % Method 1) error unit vector
% dev = abs(D_ICM-C_ICM);
% dev_sqrd = dev.^2;
% sum_devxyz = sum(dev_sqrd);
% my_stdev = sqrt(sum_devxyz./(size(D_ICM,1)-1));
% z = 1.96; % z-value for 95% confidence interval
% error = my_stdev.*z;
% error_magnitude = sqrt(sum(error.^2))
% error_norm = error./error_magnitude;

% method 2) calculate angle error
% xy - plane
Dangle_xy = rad2deg(atan2(D_ICM(:,2),D_ICM(:,1)));
Cangle_xy = rad2deg(atan2(C_ICM(:,2),C_ICM(:,1)));
dev_xy = Dangle_xy-Cangle_xy;
dev_sqrd_xy = dev_xy.^2;
sum_devxy = sum(dev_sqrd_xy);
my_stdev = sqrt(sum_devxy./(size(D_ICM,1)-1));


% Generate Histogram
figure(5)
mu    =  mean(dev_xy);            % mean
sigma =  std(dev_xy);             % standard deviation
bins  = 80;                     % Number of bins for the histogram

% Generage and Plot histogram of Temp
histogram(dev_xy, bins, 'FaceColor', 'blue')
xlabel 'angle difference [deg]'
ylabel '# of data points'
title 'Distribution of The Difference Between RAW and CALIBRATED data'
subtitle 'error = +- 51.78 deg; c=95%'
% PROBLEMS: The calc. for Dangle_xy is meaningless if the sample D is
%           not centered at the origin when calculating angle using
%           atan.
%           
%           The distribution of the difference between the raw data
%           and the calibrated data appears to be close to a gaussian
%           distribution. Upon inspection it appears +-50 degree error
%           for 95% confidence is reasonable. Therefore, the analysis
%           is correct ONLY IF Dangle_xy is meaningful data???
%           IF Dangle_xy is JUNK , then everything that comes after
%           is junk as well.
% 

%% ICM Calibration Angle Measurement
%% Import CSV file of calibration test into the workspace
DataRAW_ICM = readtable("caliData_ICM_angle.csv");
% Use curly braces instead of parenthesis to store numerical values
% from table
xICMmag = DataRAW_ICM{:,"Var1"};
yICMmag = DataRAW_ICM{:,"Var2"};
zICMmag = DataRAW_ICM{:,"Var3"};
xICMacc = DataRAW_ICM{:,"Var4"};
yICMacc = DataRAW_ICM{:,"Var5"};
zICMacc = DataRAW_ICM{:,"Var6"};
M_ICM = [xICMmag, yICMmag, zICMmag]; % Magnetometer Data
Acc_ICM = [xICMacc, yICMacc, zICMacc]; % Accelerometer Data

MC_ICM = (M_ICM-b_ICM)*A_ICM; % Corrected Magnetometer Data
q = ecompass(Acc_ICM,MC_ICM); % ANGLE???
eulerAnglesDegrees = eulerd(q,"ZYX","frame");

% Expected Angle Difference
figure(6);clf(6);
plot(eulerAnglesDegrees(:,1))
title 'About Z Axis'
figure(7);clf(7);
plot(eulerAnglesDegrees(:,2))
title 'About Y Axis'
figure(8);clf(8);
plot(eulerAnglesDegrees(:,3))
title 'About X Axs'


%% Orientation 3 Vs 4 Vs 5
Orientation3_ICM = readtable("caliData_ICM_orientation3.csv");
Orientation4_ICM = readtable("caliData_ICM_orientation4.csv");
Orientation5_ICM = readtable("caliData_ICM_orientation5.csv");

% Use curly braces instead of parenthesis to store numerical values
% from table
x3 = Orientation3_ICM{:,"Var1"};
y3 = Orientation3_ICM{:,"Var2"};
z3 = Orientation3_ICM{:,"Var3"};
D3_ICM = [x3, y3, z3]; % Raw Data
D3C_ICM = (D3_ICM-b_ICM)*A_ICM; % Calibrated Data

x4 = Orientation4_ICM{:,"Var1"};
y4 = Orientation4_ICM{:,"Var2"};
z4 = Orientation4_ICM{:,"Var3"};
D4_ICM = [x4, y4, z4]; % Raw Data
D4C_ICM = (D4_ICM-b_ICM)*A_ICM; % Calibrated Data

x5 = Orientation5_ICM{:,"Var1"};
y5 = Orientation5_ICM{:,"Var2"};
z5 = Orientation5_ICM{:,"Var3"};
D5_ICM = [x5, y5, z5]; % Raw Data
D5C_ICM = (D5_ICM-b_ICM)*A_ICM; % Calibrated Data

figure(9);clf(9);hold on
plot3(D3C_ICM(:,1),D3C_ICM(:,2),D3C_ICM(:,3),"LineStyle","none", ...
                                        "Marker","O","MarkerSize",8, ...
                                        "Color","green")
plot3(D4C_ICM(:,1),D4C_ICM(:,2),D4C_ICM(:,3),"LineStyle","none", ...
                                        "Marker","O","MarkerSize",8, ...
                                        "Color","red")
% plot3(D5C_ICM(:,1),D5C_ICM(:,2),D5C_ICM(:,3),"LineStyle","none", ...
%                                         "Marker","O","MarkerSize",8, ...
%                                         "Color","blue")
plot3(D3_ICM(:,1),D3_ICM(:,2),D3_ICM(:,3),"LineStyle","none", ...
                                        "Marker","X","MarkerSize",8, ...
                                        "Color","green")
plot3(D4_ICM(:,1),D4_ICM(:,2),D4_ICM(:,3),"LineStyle","none", ...
                                        "Marker","X","MarkerSize",8, ...
                                        "Color","red")
% plot3(D5_ICM(:,1),D5_ICM(:,2),D5_ICM(:,3),"LineStyle","none", ...
%                                         "Marker","X","MarkerSize",8, ...
%                                         "Color","blue")
mesh(Xideal, Yideal, Zideal, 'EdgeAlpha', 1, 'FaceAlpha', 0.2)
grid(gca,"on")
axis equal
xlabel("X [uT]")
ylabel("Y [uT]")
zlabel("Z [uT]")
% legend("Orientation 3 Calibrated","Orientation 4 Calibrated",...
    % "Orientation 3 Raw","Orientation 4 Raw", ...
    % "Ideal Sphere","Location","southoutside")
title("ICM: Orientation 3, 4" + newline + ...
        "Magnetometer Measurements Calibrarted")
hold off

%% Angle Between Orientation 4 and 5 [Method 1: angle 2 vectors]
D3C_ICM_mean = mean(D3C_ICM,1); % mean of each column
D3C_ICM_unit = D3C_ICM./expmfs_ICM; % unit vector of orientation 3
D3C_ICM_meanunit = D3C_ICM_mean./expmfs_ICM; % unit vector of mean
D4C_ICM_mean = mean(D4C_ICM,1); % mean of each column
D4C_ICM_unit = D4C_ICM./expmfs_ICM; % unit vector of orientation 4
D4C_ICM_meanunit = D4C_ICM_mean./expmfs_ICM; % unit vector of mean
D5C_ICM_mean = mean(D5C_ICM,1); % mean of each column
D5C_ICM_unit = D5C_ICM./expmfs_ICM; % unit vector of orientation 5
theta45 = acosd(sum(D4C_ICM_meanunit.*D5C_ICM_unit,2));
theta34 = acosd(sum(D4C_ICM_meanunit.*D3C_ICM_meanunit,2));

%% Angle Between Orientation 4 and 5 [Method 2: orientation]
Accelerometer_BNO = readtable("caliData_BNO_accelerometer.csv");
Accelerometer3_ICM = readtable("caliData_ICM_accelerometer3.csv");
Accelerometer4_ICM = readtable("caliData_ICM_accelerometer4.csv");
Accelerometer5_ICM = readtable("caliData_ICM_accelerometer5.csv");

% Use curly braces instead of parenthesis to store numerical values
% from table
ax = Accelerometer_BNO{:,"Var1"};
ay = Accelerometer_BNO{:,"Var2"};
az = Accelerometer_BNO{:,"Var3"};
Acc_BNO = [ax, ay, az]; % Raw Data
Acc_BNO_mean = mean(Acc_BNO, 1);
Acc_BNO_meanunit = Acc_BNO_mean./(9.81);

ax3 = Accelerometer3_ICM{:,"Var1"};
ay3 = Accelerometer3_ICM{:,"Var2"};
az3 = Accelerometer3_ICM{:,"Var3"};
Acc3_ICM = [ax3, ay3, az3]; % Raw Data
Acc3_ICM_mean = mean(Acc3_ICM, 1);
Acc3_ICM_meanunit = Acc3_ICM_mean./(9.81);

ax4 = Accelerometer4_ICM{:,"Var1"};
ay4 = Accelerometer4_ICM{:,"Var2"};
az4 = Accelerometer4_ICM{:,"Var3"};
Acc4_ICM = [ax4, ay4, az4]; % Raw Data
Acc4_ICM_mean = mean(Acc4_ICM, 1);
Acc4_ICM_meanunit = Acc4_ICM_mean./(9.81);

ax5 = Accelerometer5_ICM{:,"Var1"};
ay5 = Accelerometer5_ICM{:,"Var2"};
az5 = Accelerometer5_ICM{:,"Var3"};
Acc5_ICM = [ax5, ay5, az5]; % Raw Data
Acc5_ICM_mean = mean(Acc5_ICM, 1);
Acc5_ICM_meanunit = Acc5_ICM_mean./(9.81);

q3 = ecompass(Acc3_ICM,D3C_ICM_unit); % ANGLE???
eulerAnglesDegrees3 = eulerd(q3,"ZYX","frame");

q4 = ecompass(Acc4_ICM_mean,D4C_ICM_meanunit); % ANGLE???
eulerAnglesDegrees4 = eulerd(q4,"ZYX","frame");

q5 = ecompass(Acc5_ICM,D5C_ICM_unit); % ANGLE???
eulerAnglesDegrees5 = eulerd(q5,"ZYX","frame");

eulerAngleDegrees45 = eulerAnglesDegrees5-eulerAnglesDegrees4;
eulerAngleDegrees34 = eulerAnglesDegrees4-eulerAnglesDegrees3;



% mag3 projected onto ground plane
% cross3 = cross(Acc3_ICM_meanunit, D3C_ICM_meanunit);
proj3 = D3C_ICM_mean - dot(D3C_ICM_mean, Acc3_ICM_meanunit).*Acc3_ICM_meanunit;
proj3_mag = sqrt(proj3(1)^2+proj3(2)^2+proj3(3)^2);
proj3_unit = proj3./proj3_mag;
% mag4 projected onto ground plane
% cross4 = cross(Acc3_ICM_meanunit, D4C_ICM_meanunit);
proj4 = D4C_ICM_mean - dot(D4C_ICM_mean, Acc4_ICM_meanunit).*Acc4_ICM_meanunit;
proj4_mag = sqrt(proj4(1)^2+proj4(2)^2+proj4(3)^2);
proj4_unit = proj4./proj4_mag;
% angle between
Theta12_GP = acosd(sum(proj1_unit.*proj2_unit));

% mag3 projected onto xy-plane
% cross3 = cross(Acc3_ICM_meanunit, D3C_ICM_meanunit);
xy = [0 0 -1];
proj3add = D3C_ICM_mean - dot(D3C_ICM_mean, xy).*xy;
proj3_magadd = sqrt(proj3add(1)^2+proj3add(2)^2+proj3add(3)^2);
proj3_unitadd = proj3add./proj3_magadd;
% mag3 projected onto xy
% cross4 = cross(Acc3_ICM_meanunit, D4C_ICM_meanunit);
proj4add = D4C_ICM_mean - dot(D4C_ICM_mean, xy).*xy;
proj4_magadd = sqrt(proj4add(1)^2+proj4add(2)^2+proj4add(3)^2);
proj4_unitadd = proj4add./proj4_magadd;
% angle between
Theta34_XY = acosd(sum(proj3_unitadd.*proj4_unitadd));
trig_in = sum(proj3_unitadd.*proj4_unitadd);

% mag5 projected onto ground plane
proj5 = D5C_ICM_mean - dot(D5C_ICM_mean, Acc5_ICM_meanunit).*Acc5_ICM_meanunit;
proj5_mag = sqrt(proj5(1)^2+proj5(2)^2+proj5(3)^2);
proj5_unit = proj5./proj5_mag;
% angle between
Theta45_GP = acosd(sum(proj5_unit.*proj4_unit));

% % mag5 projected onto BNO ground plane
% proj5 = D5C_ICM_mean - dot(D5C_ICM_mean, Acc_BNO_meanunit).*Acc_BNO_meanunit
% proj5_mag = sqrt(proj5(1)^2+proj5(2)^2+proj5(3)^2)
% proj5_unit = proj5./proj5_mag
% 
% % mag5 projected onto BNO ground plane
% proj4 = D4C_ICM_mean - dot(D4C_ICM_mean, Acc_BNO_meanunit).*Acc_BNO_meanunit
% proj4_mag = sqrt(proj4(1)^2+proj4(2)^2+proj4(3)^2)
% proj4_unit = proj4./proj4_mag
% % angle between
% Theta45_BNOGP = acosd(sum(proj5_unit.*proj4_unit))

figure(9);hold on
% plot earth plane of orientation 3
% w3 = null(Acc3_ICM_mean); % Find two orthonormal vectors which are orthogonal to v
% [P3,Q3] = meshgrid(-50:50); % Provide a gridwork (you choose the size)
% X3 = w3(1,1)*P3+w3(1,2)*Q3; % Compute the corresponding cartesian coordinates
% Y3 = w3(2,1)*P3+w3(2,2)*Q3; %   using the two vectors in w
% Z3 = w3(3,1)*P3+w3(3,2)*Q3;
% surf(X3,Y3,Z3,"FaceColor","none","EdgeColor","green","EdgeAlpha",0.2)
% 
% w4 = null(Acc4_ICM_mean); % Find two orthonormal vectors which are orthogonal to v
% [P4,Q4] = meshgrid(-50:50); % Provide a gridwork (you choose the size)
% X4 = w4(1,1)*P4+w4(1,2)*Q4; % Compute the corresponding cartesian coordinates
% Y4 = w4(2,1)*P4+w4(2,2)*Q4; %   using the two vectors in w
% Z4 = w4(3,1)*P4+w4(3,2)*Q4;
% surf(X4,Y4,Z4,"FaceColor","none","EdgeColor","red","EdgeAlpha",0.2)
% 
% w5 = null(Acc5_ICM_mean); % Find two orthonormal vectors which are orthogonal to v
% [P5,Q5] = meshgrid(-50:50); % Provide a gridwork (you choose the size)
% X5 = w5(1,1)*P5+w5(1,2)*Q5; % Compute the corresponding cartesian coordinates
% Y5 = w5(2,1)*P5+w5(2,2)*Q5; %   using the two vectors in w
% Z5 = w5(3,1)*P5+w5(3,2)*Q5;
% surf(X5,Y5,Z5,"FaceColor","none","EdgeColor","blue","EdgeAlpha",0.2)

[myX3, myY3] = meshgrid(-50:50,-50:50);
% ax+by+cz=0
myZ3 = (Acc3_ICM_meanunit(:,1).*myX3 + Acc3_ICM_meanunit(:,2).*myY3)./(-Acc3_ICM_meanunit(:,3));
surf(myX3,myY3,myZ3,"FaceColor","none","EdgeColor","magenta","EdgeAlpha",0.2)
% accel vector
scale_accVec = 10; % Scale factor for the accel vector magnitude
Acc3_ICM_vec = Acc3_ICM_mean.*scale_accVec;
quiver3(0,0,0,Acc3_ICM_vec(1),Acc3_ICM_vec(2),Acc3_ICM_vec(3), ...
                                    'LineWidth',2,"Color","magenta","MaxHeadSize",10)
% projected orientation 3 on ground plane
quiver3(0,0,0,proj3(1),proj3(2),proj3(3), ...
                'LineWidth',2,"Color","green","LineStyle","--")
% projected orientation 4 on ground plane
quiver3(0,0,0,proj4(1),proj4(2),proj4(3), ...
                'LineWidth',2,"Color","red","LineStyle","--")
% orientation 3 vector
quiver3(0,0,0,D3C_ICM_mean(1),D3C_ICM_mean(2),D3C_ICM_mean(3), ...
                'LineWidth',2,"Color","green","MaxHeadSize",10)
% orientation 4 vector
quiver3(0,0,0,D4C_ICM_mean(1),D4C_ICM_mean(2),D4C_ICM_mean(3), ...
                'LineWidth',2,"Color","red","MaxHeadSize",10)

% [myX, myY] = meshgrid(-50:50,-50:50);
% % ax+by+cz=0
% myZ = (Acc_BNO_meanunit(:,1).*myX + Acc_BNO_meanunit(:,2).*myY)./(-Acc_BNO_meanunit(:,3));
% surf(myX,myY,myZ,"FaceColor","none","EdgeColor","blue","EdgeAlpha",0.2)

legend("Orientation 1 Calibrated","Orientation 2 Calibrated",...
    "Orientation 1 Raw","Orientation 2 Raw", ...
    "Ideal Sphere","Ground Plane","Accel Vector", ...
    "Orientation 1 Projection On Ground PLane","Orientation 2 Projection On Ground Plane", ...
    "Orientaion 1 Vector","Orientation 2 Vector", ...
    "Location","best")
hold off

figure(11);clf(11);hold on
% ground plane
[myX3, myY3] = meshgrid(-50:50,-50:50);
% ax+by+cz=0
myZ3 = (Acc3_ICM_meanunit(:,1).*myX3 + Acc3_ICM_meanunit(:,2).*myY3)./(-Acc3_ICM_meanunit(:,3));
surf(myX3,myY3,myZ3,"FaceColor","none","EdgeColor","magenta","EdgeAlpha",0.2)
% orientation 3 vector
quiver3(0,0,0,D3C_ICM_mean(1),D3C_ICM_mean(2),D3C_ICM_mean(3), ...
                'LineWidth',2,"Color","green","LineStyle","--")
% orientation 4 vector
quiver3(0,0,0,D4C_ICM_mean(1),D4C_ICM_mean(2),D4C_ICM_mean(3), ...
                'LineWidth',2,"Color","red","LineStyle","--")
legend("Ground Plane",...
    "Orientation 1 Projection On Ground PLane", ...
    "Orientation 2 Projection On Ground Plane", ...
    "Location","best")
grid(gca,"on")
axis equal
title("Angle Between Orientation 1 and 2 " + newline + ...
        "projected Onto Ground PLane")
view(2)
hold off


%% Angle Between Orientation 4 and 5 [Method 3: custom heading function]
for i = 1:size(D3C_ICM,1)
    heading3(i) = calculateHeadingWithTilt(D3C_ICM(i,:),Acc3_ICM(i,:));
end
for i = 1:size(D4C_ICM,1)
    heading4(i) = calculateHeadingWithTilt(D4C_ICM(i,:),Acc4_ICM(i,:));
end
for i = 1:size(D5C_ICM,1)
    heading5(i) = calculateHeadingWithTilt(D5C_ICM(i,:),Acc5_ICM(i,:));
end

heading45 = mean(heading5) - mean(heading4);
heading35 = mean(heading5) - mean(heading3);
heading34 = mean(heading4) - mean(heading3);


%% Calculate Expected Angle Between 4 and 5
s1 = 196.5; % [in]
s2 = 211; % [in]
s3 = 288; % [in]
theta34_expected = acosd((s1^2+s3^2-s2^2)/(2*s1*s3));

%% Plot Magnetometer Raw and Calibrated From CU Data on Sphere
figure(10);clf(10);hold on
plot3(MC_ICM(:,1),MC_ICM(:,2),MC_ICM(:,3),"LineStyle","none", ...
                                        "Marker","O","MarkerSize",8, ...
                                        "Color","red")
plot3(M_ICM(:,1),M_ICM(:,2),M_ICM(:,3),"LineStyle","none", ...
                                        "Marker","O","MarkerSize",8, ...
                                        "Color","blue")
mesh(Xideal, Yideal, Zideal, 'EdgeAlpha', 1, 'FaceAlpha', 0.2)
grid(gca,"on")
axis equal
xlabel("X [uT]")
ylabel("Y [uT]")
zlabel("Z [uT]")
legend("Orientation Calibrated","Orientation Calibrated","Sphere","Location","southoutside")
title("ICM: Orientation" + newline + ...
        "Magnetometer Measurements Calibrarted")
hold off

%% statistical uncertainty calc
% calculate standard deviation
D3C_stdev = std(D3C_ICM_unit, 0, 1);
D4C_stdev = std(D4C_ICM_unit, 0, 1);

z = 1.96; % z-value for 95% confidence interval

% calculate +- error in mag reading
error1_mag = D1C_stdev.*z;
error2_mag = D2C_stdev.*z;
error1_magNorm = error1_mag./D1C_ICM_meanunit;
error2_magNorm = error2_mag./D2C_ICM_meanunit;

% calculate +- error in accel reading
Acc1_ICM_unit = Acc1_ICM./sqrt(Acc1_ICM_mean(1)^2+Acc1_ICM_mean(2)^2+Acc1_ICM_mean(3)^2);
Acc_stdev = std(Acc1_ICM_unit, 0, 1);
error_acc = Acc_stdev.*z;
error_accNorm = error_acc./Acc1_ICM_meanunit;
error_accNorm(2) = 0.01;

% calculate error propagated during projection calc
error1_proj = sqrt(sum(sqrt(error1_magNorm.^2 + error_accNorm.^2))^2+error_accNorm.^2)./proj1;
error2_proj = sqrt(sum(sqrt(error2_magNorm.^2 + error_accNorm.^2))^2+error_accNorm.^2)./proj2;

% calculate error propagated after dotting 1 and 2
error_dot = sum(sqrt(error1_proj.^2 + error2_proj.^2));

% calculate error propogated after inverse cos
error_trigRad = abs(-1/sqrt(1-(trig_in)^2))*error_dot; % [rad]
error_trigDeg = rad2deg(error_trigRad); % [deg]

%%
function heading = calculateHeadingWithTilt(mag, accel)
    % Input:
    %   mag   - 3x1 vector [X_m, Y_m, Z_m] from the magnetometer
    %   accel - 3x1 vector [X_a, Y_a, Z_a] from the accelerometer
    % Output:
    %   heading - Tilt-compensated heading angle in degrees

    % Extract magnetometer and accelerometer readings
    X_m = mag(1); Y_m = mag(2); Z_m = mag(3);
    X_a = accel(1); Y_a = accel(2); Z_a = accel(3);

    % Calculate Roll (phi) and Pitch (theta) angles
    roll = atan2(Y_a, Z_a); % Roll angle
    pitch = atan2(-X_a, sqrt(Y_a^2 + Z_a^2)); % Pitch angle

    % Rotation matrix for pitch correction (around X-axis)
    R_x = [1, 0, 0;
           0, cos(pitch), -sin(pitch);
           0, sin(pitch), cos(pitch)];

    % Rotation matrix for roll correction (around Y-axis)
    R_y = [cos(roll), 0, sin(roll);
           0, 1, 0;
          -sin(roll), 0, cos(roll)];

    % Apply tilt correction
    R = R_y * R_x; % Combined rotation matrix
    correctedMag = R * [X_m; Y_m; Z_m]; % Corrected magnetic field vector

    % Compute the heading (angle in the horizontal plane)
    heading = atan2(correctedMag(2), correctedMag(1)); % Heading in radians

    % Convert heading to degrees
    heading = rad2deg(heading);

    % Normalize heading to [0, 360)
    if heading < 0
        heading = heading + 360;
    end
end