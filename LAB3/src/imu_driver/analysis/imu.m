clc;
clear all;

%reading the rosbag and select the rosbag, then output the rosmsg to struct
bag = rosbag('C:\Users\Admin\Desktop\eece5554\imu2.bag');
bag_select = select (bag,"Time",[bag.StartTime bag.EndTime], "Topic","/imu");
msgs = readMessages(bag_select,"DataFormat","struct");

endvalue = length(msgs);

%creating the values
time = zeros (1,endvalue);
qx = zeros (1,endvalue);
qy = zeros (1,endvalue);
qz = zeros (1,endvalue);
qw = zeros (1,endvalue);
angularx = zeros (1,endvalue);
angulary = zeros (1,endvalue);
angularz = zeros (1,endvalue);
linearx = zeros (1,endvalue);
lineary = zeros (1,endvalue);
linearz = zeros (1,endvalue);
mfx = zeros (1,endvalue);
mfy = zeros (1,endvalue);
mfz = zeros (1,endvalue);
yaw = zeros (1,endvalue);
pitch = zeros (1,endvalue);
roll = zeros (1,endvalue);
avg = zeros (1,12);
stdev = zeros (1,12);
i=1;

%sending the values:
while i<=endvalue
    qx(1,i) = msgs{i,1}.IMU.Orientation.X;
    qy(1,i) = msgs{i,1}.IMU.Orientation.Y;
    qz(1,i) = msgs{i,1}.IMU.Orientation.Z;
    qw(1,i) = msgs{i,1}.IMU.Orientation.W;
    angularx(1,i) = msgs{i,1}.IMU.AngularVelocity.X;
    angulary(1,i) = msgs{i,1}.IMU.AngularVelocity.Y;
    angularz(1,i) = msgs{i,1}.IMU.AngularVelocity.Z;
    linearx(1,i) = msgs{i,1}.IMU.LinearAcceleration.X;
    lineary(1,i) = msgs{i,1}.IMU.LinearAcceleration.Y;
    linearz(1,i) = msgs{i,1}.IMU.LinearAcceleration.Z;
    mfx(1,i) = msgs{i,1}.MagField.MagneticField_.X;
    mfy(1,i) = msgs{i,1}.MagField.MagneticField_.Y;
    mfz(1,i) = msgs{i,1}.MagField.MagneticField_.Z;
    [yaw(1,i),pitch(1,i),roll(1,i)] = quat2euler(qx(1,i),qy(1,i),qz(1,i),qw(1,i));
    time(1,i)=i;
    i = i+1;
end

avg(1,1)=mean(yaw);
avg(1,2)=mean(pitch);
avg(1,3)=mean(roll);
avg(1,4)=mean(angularx);
avg(1,5)=mean(angulary);
avg(1,6)=mean(angularz);
avg(1,7)=mean(linearx);
avg(1,8)=mean(lineary);
avg(1,9)=mean(linearz);
avg(1,10)=mean(mfx);
avg(1,11)=mean(mfy);
avg(1,12)=mean(mfz);

stdev(1,1)=std(yaw);
stdev(1,2)=std(pitch);
stdev(1,3)=std(roll);
stdev(1,4)=std(angularx);
stdev(1,5)=std(angulary);
stdev(1,6)=std(angularz);
stdev(1,7)=std(linearx);
stdev(1,8)=std(lineary);
stdev(1,9)=std(linearz);
stdev(1,10)=std(mfx);
stdev(1,11)=std(mfy);
stdev(1,12)=std(mfz);
%plot

figure(1)

subplot(3,4,1);
plot(time,yaw);
title('yaw changes');
xlabel('time(1/40s)');
ylabel('yaw(rad)');

subplot(3,4,2);
plot(time,pitch);
title('pitch changes');
xlabel('time(1/40s)');
ylabel('pitch(rad)');

subplot(3,4,3);
plot(time,roll);
title('roll changes');
xlabel('time(1/40s)');
ylabel('roll(rad)');

subplot(3,4,4);
plot(time,angularx);
title('angular velocityx changes');
xlabel('time(1/40s)');
ylabel('rad/s');

subplot(3,4,5);
plot(time,angulary);
title('angular velocityy changes');
xlabel('time(1/40s)');
ylabel('rad/s');

subplot(3,4,6);
plot(time,angularz);
title('angular velocityz changes');
xlabel('time(1/40s)');
ylabel('rad/s');

subplot(3,4,7);
plot(time,angularx);
title('linear accelerationx changes');
xlabel('time(1/40s)');
ylabel('m/s^2');

subplot(3,4,8);
plot(time,angulary);
title('linear accelerationy changes');
xlabel('time(1/40s)');
ylabel('m/s^2');

subplot(3,4,9);
plot(time,angularz);
title('linear accelerationz changes');
xlabel('time(1/40s)');
ylabel('m/s^2');

subplot(3,4,10);
plot(time,mfx);
title('magneticfieldx changes');
xlabel('time(1/40s)');
ylabel('gauss');

subplot(3,4,11);
plot(time,mfy);
title('magneticfieldy changes');
xlabel('time(1/40s)');
ylabel('gauss');

subplot(3,4,12);
plot(time,mfz);
title('magneticfieldz changes');
xlabel('time(1/40s)');
ylabel('gauss');

figure(2)

subplot(2,2,1);
scatter3(yaw,pitch,roll,"filled");
xlabel('yaw'); ylabel('pitch'); zlabel('roll');
title('eular angle');

subplot(2,2,2);
scatter3(angularx,angulary,angularz,"filled");
xlabel('avx'); ylabel('avy'); zlabel('avz');
title('angular velocity');

subplot(2,2,3);
scatter3(linearx,lineary,linearz,"filled");
xlabel('x'); ylabel('y'); zlabel('z');
title('linear acceleration');

subplot(2,2,4);
scatter3(mfx,mfy,mfz,"filled");
xlabel('x'); ylabel('y'); zlabel('z');
title('magneticfield');

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

