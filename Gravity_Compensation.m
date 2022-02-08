clc
clear

load('gravityCompData001.mat');


%% Gravity Compensation ...

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%% IMU Moving Flatform - Orientation - IMU Coordinate

Sensor_Orientation = dataAll.IMU_Moving;

figure
plot(Sensor_Orientation)

grid on

legend('Roll', 'Pitch', 'Yaw')

xlabel('Time [ms]')
ylabel('Orientation [deg]')
title('IMU Coordinate')

%% Rotation Matrix

% Rotation matrix : Sensor -> Moving Flatform
R = rotx(deg2rad(-90)) * rotz(deg2rad(-90));

% Sensor -> Moving
Rot_M_S = R';

%% IMU Moving Flatform - Orientation - Moving Flatform Coordinate

figure

for i=1:length(Sensor_Orientation)
    
    Rot_S(:,:,i) = rotx(deg2rad(Sensor_Orientation(i,1)))*roty(deg2rad(Sensor_Orientation(i,2)));
    Rot_M(:,:,i) = Rot_M_S * Rot_S(:,:,i);
end

eulDeg_Moving = rad2deg(rotm2eul(Rot_M, 'XYZ'));

plot(eulDeg_Moving)

grid on

legend('Roll', 'Pitch', 'Yaw')

xlabel('Time [ms]')
ylabel('Orientation [deg]')
title('Moving Flatform Coordinate')


%% Load Cell - Before Gravity Compensation

Force = dataAll.FT_Sensor(:,3);

figure

plot(Force)

grid on

xlabel('Time [ms]')
ylabel('Force [N]')

legend('Raw')

%% Gravity Compensation

mass = 1.44;    % [g]

gravity_acc = 9.80665;   % [m/s^2]

mg = mass * gravity_acc * 0.001;    % [N]

mg_mat = [0 0 mg]';

for i=1:length(Rot_M)
    mg_mat_rot(:,i) = Rot_M(:,:,i) * mg_mat;
end

Force_Gravity = mg_mat_rot(3,:)';
Force_Gravity = Force_Gravity - Force_Gravity(1);


Force = dataAll.FT_Sensor(:,3);


fs = 1000;

N = 1;
fc = 2;

h  = fdesign.lowpass('N,F3dB', N, fc, fs);
Hd = design(h, 'butter');


Force = Force - Force(1);

Force_Comp = Force-Force_Gravity;

Force_Comp_filter = filter(Hd, Force_Comp);;

figure

plot(Force)
hold on
plot(Force_Gravity)
plot(Force_Comp)
plot(Force_Comp_filter, 'LineWidth', 1.3)

grid on

xlabel('Time [ms]')
ylabel('Force [N]')

legend('Raw Force', 'Gravity Force', 'Compensated Force', 'Compensated Force Filter')