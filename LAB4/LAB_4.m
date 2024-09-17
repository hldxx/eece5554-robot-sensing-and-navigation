clc;
clear all;

%reading the rosbag and select the rosbag, then output the rosmsg to struct
bag = rosbag('C:\Users\Admin\Desktop\eece5554\LAB4data.bag');
bag_select_imu = select (bag,"Time",[bag.StartTime bag.EndTime], "Topic","/imu");
bag_select_gps = select (bag,"Time",[bag.StartTime bag.EndTime], "Topic","/gps");
msgs_imu = readMessages(bag_select_imu,"DataFormat","struct");
msgs_gps = readMessages(bag_select_gps,"DataFormat","struct");

endvalue_imu = length(msgs_imu);
endvalue_gps = length(msgs_gps);

%sending the values to gps 
i=1;
j=1;
time_gps = ones(1,endvalue_gps);
altitude = ones(1,endvalue_gps);
latitude = ones(1,endvalue_gps);
longitude = ones(1,endvalue_gps);
utmEasting = ones(1,endvalue_gps);
utmNorthing = ones(1,endvalue_gps);
%sending the values to imu
time_imu = ones(1,endvalue_imu);
qx = ones(1,endvalue_imu);
qy = ones(1,endvalue_imu);
qz = ones(1,endvalue_imu);
qw = ones(1,endvalue_imu);
angularx = ones(1,endvalue_imu);
angulary = ones(1,endvalue_imu);
angularz = ones(1,endvalue_imu);
linearx = ones(1,endvalue_imu);
lineary = ones(1,endvalue_imu);
linearz = ones(1,endvalue_imu);
mfx = ones(1,endvalue_imu);
mfy = ones(1,endvalue_imu);
mfz = ones(1,endvalue_imu);
yaw = ones(1,endvalue_imu);
pitch = ones(1,endvalue_imu);
roll = ones(1,endvalue_imu);




%sending gps
while i<=endvalue_gps
    altitude(1,i) = msgs_gps{i,1}.Altitude;
    latitude(1,i) = msgs_gps{i,1}.Latitude;
    longitude(1,i) = msgs_gps{i,1}.Longitude;
    utmEasting(1,i) = msgs_gps{i,1}.UtmEasting;
    utmNorthing(1,i) = msgs_gps{i,1}.UtmNorthing;
    time_gps(1,i)=i;
    i = i+1;
end

%sending imu

while j<=endvalue_imu
    qx(1,j) = msgs_imu{j,1}.IMU.Orientation.X;
    qy(1,j) = msgs_imu{j,1}.IMU.Orientation.Y;
    qz(1,j) = msgs_imu{j,1}.IMU.Orientation.Z;
    qw(1,j) = msgs_imu{j,1}.IMU.Orientation.W;
    angularx(1,j) = msgs_imu{j,1}.IMU.AngularVelocity.X;
    angulary(1,j) = msgs_imu{j,1}.IMU.AngularVelocity.Y;
    angularz(1,j) = msgs_imu{j,1}.IMU.AngularVelocity.Z;
    linearx(1,j) = msgs_imu{j,1}.IMU.LinearAcceleration.X;
    lineary(1,j) = msgs_imu{j,1}.IMU.LinearAcceleration.Y;
    linearz(1,j) = msgs_imu{j,1}.IMU.LinearAcceleration.Z;
    mfx(1,j) = msgs_imu{j,1}.MagField.MagneticField_.X;
    mfy(1,j) = msgs_imu{j,1}.MagField.MagneticField_.Y;
    mfz(1,j) = msgs_imu{j,1}.MagField.MagneticField_.Z;
    [yaw(1,j),pitch(1,j),roll(1,j)] = quat2euler(qx(1,j),qy(1,j),qz(1,j),qw(1,j));
    time_imu(1,j)=j;
    j = j+1;
end

%plotting
%plot (latitude(485:525),longitude(485:525));
%title('walking trajectory (lat,lon)');
%xlabel('latitude');
%ylabel('longitude');

plot (utmNorthing,utmEasting);
title('walking trajectory (utm)');
xlabel('utmNorthing');
ylabel('utmEasting');

%%%%%%%%the magnetic data before calibration%%%%%%%%%%%%%%%%%%
figure;
plot(mfx(19411:21011),mfy(19411:21011));
title('Magnatometer data before calibration');
xlabel('Magnetic field in x(tesla)');
ylabel('Magnetic field in y(tesla)');
grid on;
hold on;
%%%%%%%%the magnetic data after calibration%%%%%%%%%%%%%%%%%%%%

x = mfx(19411:21011);
y = mfy(19411:21011);

x_mean = mean(x);
y_mean = mean(y);

x_offset = (max(x) + min(x)) / 2;
y_offset = (max(y) + min(y)) / 2;

x_corr = x - x_offset;
y_corr = y - y_offset;

[x_corr2, y_corr2] = correct_soft_iron(x_corr, y_corr);

%figure; 
plot(x_corr2, y_corr2); 
title('Magnatometer data after calibration');
xlabel('Magnetic field in x(tesla)');
ylabel('Magnetic field in y(tesla)');
hold on;

%figure; 
plot(x_corr, y_corr); 
title('Magnatometer data calibration');
xlabel('Magnetic field in x(tesla)');
ylabel('Magnetic field in y(tesla)');
hold off;

%%%%%%%%%%%yaw_correction%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xx = mfx(22000:end);
yy = mfy(22000:end);

xx_corr = xx-x_offset;
yy_corr = yy-y_offset;

[xx_corr2,yy_corr2] = correct_soft_iron(xx_corr,yy_corr);

yaw_corrected = unwrap(atan2(yy_corr2, xx_corr2));
yaw_raw = yaw(22000:end);

figure;
plot(time_imu(22000:end),yaw_raw,'DisplayName', 'Yaw');
hold on;
plot(time_imu(22000:end),yaw_corrected,'DisplayName', 'Yaw_corrected');
title('Comparison of yaw raw with the corrected yaw');
ylabel('yaw (radian)');
xlabel('time (1/40sec)');
grid on;
legend show;

%%%%%%%%%%%%%gyro_integrated%%%%%%%%%%%%%%%%%%%%%%

angularz_selected = angularz(22000:end);


yaw_gyro = cumtrapz(time_imu(22000:end),angularz_selected);
yaw_gyro = wrapToPi(yaw_gyro);%control it to -pi and pi

figure;  
plot(time_imu(22000:end), yaw_gyro, 'DisplayName', 'Yaw_gyro'); 
hold on;  
plot(time_imu(22000:end),yaw_raw,'DisplayName', 'Yaw');
hold on;
plot(time_imu(22000:end), yaw_corrected, 'DisplayName', 'Yaw_corrected');  
title('yaw_corrected and yaw Integrated from Gyro');
ylabel('yaw ( radian)');
xlabel('time (1/40sec)');
grid on;  
legend show;  

%%%%%%%%%%%%%filter%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lp_fc = 0.05;
order = 4;
nyq = 0.5 * 20;
lp_fc_norm = lp_fc / nyq;
[b, a] = butter(order, lp_fc_norm, 'low');
filtered_magnetometer = filtfilt(b, a, yaw_corrected)';

hp_fc = 0.00015;
order = 4;
nyq = 0.5 * 20;
hp_fc_norm = hp_fc / nyq;
[b, a] = butter(order, hp_fc_norm, 'high');
filtered_yaw_integrated = filtfilt(b, a, yaw_gyro);

weight = 0.05;

complimentary_filtered = weight*(filtered_yaw_integrated) + (1-weight)*(filtered_magnetometer);

figure;
hold on;
plot(time_imu(22000:end), filtered_magnetometer, 'DisplayName', 'Low_pass');
plot(time_imu(22000:end), filtered_yaw_integrated, 'DisplayName', 'High_pass');
plot(time_imu(22000:end), complimentary_filtered, 'DisplayName', 'Complimentary');
hold off;

title('Yaw after applying filter');
ylabel('yaw (radian)');
xlabel('time (1/40 sec)');
legend('show');
grid on;


%%%%%%%%%%%%%velocity from gps%%%%%%%%%%%%%%%%%%%%%%%%

vel_gps = GPS_DistanceCalc(latitude,longitude,time_gps);
vel_gps = vel_gps(:,1);
vel_gps(end+1) = 0;

% 
% figure;
% plot(time_gps, vel_gps, 'DisplayName', "gps vel forward");
% title('Velocity estimate from the GPS');
% ylabel('Velocity (m/sec)');
% xlabel('time (sec)');
% legend('show');

%%%%%%%%%%%%%%%%%%velocity from forward velocity%%%%%%%%%%%%%

fs = 40;
time_1 = 0:1/fs:(length(time_imu) / fs - 1/fs);%%%convert imu time to real time

new_linearx3 = zeros(size(linearx));
new_linearx3 = cumtrapz(linearx)*(1/fs);

lp_fc = 0.18;
order = 4;
nyq = 0.5 * fs;
lp_fc_norm = lp_fc / nyq;
[b, a] = butter(order, lp_fc_norm, 'low');
linearx = filtfilt(b, a, linearx);
linearx = linearx.*cos(pitch) + linearx.*sin(pitch);

new_linearx = linearx;

% High-pass filter
hp_fc = 0.0015; 
[b, a] = butter(order, hp_fc / nyq, 'high');
new_linearx = filtfilt(b, a, linearx);

new_linearx = detrend(new_linearx, 'constant');
new_linearx = new_linearx - new_linearx(1);
new_linearx2 = zeros(size(new_linearx));
new_linearx2 = cumtrapz(new_linearx)*(1/fs);


detrend_vel = detrend(new_linearx2, 'constant');
linearx_final = new_linearx2;



figure;
hold on;
plot(time_1, new_linearx3, 'DisplayName', 'before_adjustment');
plot(time_gps, vel_gps, 'DisplayName', "gps vel forward");
plot(time_1, linearx_final, 'DisplayName', 'after_adjustment');
xlabel('Time (in sec)');
title('Velocity estimate from accelerometer before and after adjustment');
ylabel('Velocity (in m/s)');
grid on;
legend show;
hold off;

%%%%%%%%%%%%%%%%%%%%%Dead Reckoning%%%%%%%%%%%%%%%

% For IMU estimate
distance_imu = zeros(size(linearx_final));
distance_imu = cumtrapz(new_linearx3) * 1/40;

% For GPS distance
distance_gps = zeros(size(vel_gps));
distance_gps = cumtrapz(vel_gps);

% Plotting
figure;
hold on;
plot(time_1, distance_imu, 'DisplayName', 'imu estimate');
plot(time_gps, distance_gps, 'DisplayName', 'gps_distance');
xlabel('Time (sec)');
ylabel('Distance (m)');
title('Displacement');
legend;
hold off;



% Calculate Y_dot_dot, X_dot, and wX_dot
Y_dot_dot = lineary;
X_dot = linearx_final;
W = angularz;
wX_dot = W .* X_dot;

% Filter settings
lp_fc = 0.18;
order = 4;
nyq = 0.5 * fs;
lp_fc_norm = lp_fc / nyq;
[b, a] = butter(order, lp_fc_norm, 'low');

% Filtering wX_dot and Y_dot_dot
wX_dot_filtered = filtfilt(b, a, wX_dot);
Y_dot_dot_filtered = filtfilt(b, a, Y_dot_dot);

% Plotting
figure;
hold on;
plot(time_imu, wX_dot_filtered, 'DisplayName', 'w.X_dot');
plot(time_imu, detrend(Y_dot_dot_filtered), 'DisplayName', 'Y_dot_dot');
xlabel('Time');
grid on;
legend;
hold off;




% Calculate Cf_yaw, Vn, and Ve

x_all_corr = mfx-x_offset;
y_all_corr = mfy-y_offset;

[x_all_corr2,y_all_corr2] = correct_soft_iron(x_all_corr,y_all_corr);

yaw_all_corrected = unwrap(atan2(y_all_corr2,x_all_corr2));

yaw_all_gyro = wrapToPi(cumtrapz(time_imu,angularz));

lp_fc = 0.05;
order = 4;
nyq = 0.5 * 20;
lp_fc_norm = lp_fc / nyq;
[b, a] = butter(order, lp_fc_norm, 'low');
filtered_all_magnetometer = filtfilt(b, a, yaw_all_corrected)';

hp_fc = 0.00015;
order = 4;
nyq = 0.5 * 20;
hp_fc_norm = hp_fc / nyq;
[b, a] = butter(order, hp_fc_norm, 'high');
filtered_all_yaw_integrated = filtfilt(b, a, yaw_all_gyro);

weight = 0.05;

complimentary_all_filtered = weight*(filtered_all_yaw_integrated) + (1-weight)*(filtered_all_magnetometer);

Cf_yaw = complimentary_all_filtered ;
Vn = cos(Cf_yaw) .* linearx_final - sin(Cf_yaw) .* linearx_final;
Ve = sin(Cf_yaw) .* linearx_final + cos(Cf_yaw) .* linearx_final;

% Calculate Xe and Xn
Xe = zeros(size(Ve));
Xe = cumtrapz(Ve) * 1/fs;
Xe = Xe /10; 

Xn = zeros(size(Vn));
Xn = cumtrapz(Vn) * 1/fs;
Xn = Xn /10;
dis_x = utmNorthing - utmNorthing(1,1);
dis_y = utmEasting - utmEasting(1,1);

% Plotting
figure;
hold on;
plot(Xe, Xn, 'DisplayName', 'imu_distance');
plot(dis_x,dis_y, 'DisplayName', 'gps_distance');
legend;
hold off;
%%%%%%%%%%%%%%%%%%xc%%%%%%%%%%%%%%%

w_dot = diff(angularz) ./ diff(time_imu);

% Calculate xc
xc = (Y_dot_dot(2:end) - wX_dot(2:end)) ./ w_dot;

% Calculate x_c
x_c = median(xc);

x_c;


function [yaw, pitch, roll] = quat2euler(q1,q2,q3,q4)
   
    qw = q1;
    qx = q2;
    qy = q3;
    qz = q4;
    
    % Compute yaw, pitch, and roll in radians
    yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
    pitch = asin(2*(qw*qy - qz*qx));
    roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2));
end

function [x_corr2, y_corr2] = correct_soft_iron(x_corr, y_corr)
    ellipse_t = fit_ellipse(x_corr, y_corr);
    a = ellipse_t.long_axis / 2;
    b = ellipse_t.short_axis / 2;
    angle = ellipse_t.phi;
    
    data_corr = [x_corr; y_corr]';
    
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    data_rotated = (R * data_corr')';
    
    S =(0.2172/1.15049)* [1/a, 0; 0, 1/b];
    data_soft_iron_corrected = (S * data_rotated')';
    
    
    x_corr2 = data_soft_iron_corrected(:, 1);
    y_corr2 = data_soft_iron_corrected(:, 2);
end

function [vel]= GPS_DistanceCalc(lat, lon, TimeVec) %#ok<*DEFNU>
LatLon = [lat',lon'];
dLatLon = diff(LatLon);
DistDeg = hypot(dLatLon(:,1), dLatLon(:,2));       % Distance (Degree Differences)
% d2m = 39983551.2/360;   %LAT
d2m = 29795778/360; %LON
% Degrees-To-Metres Conversion (Approximate)
DistMtr = DistDeg * d2m;                           % Distance (Metre Differences)
dTime = diff(TimeVec);                                % Sampling Time Differences
vel = DistMtr ./ dTime;      
end
