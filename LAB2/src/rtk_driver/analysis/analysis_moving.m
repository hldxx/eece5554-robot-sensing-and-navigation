clc;
clear all;

%reading the rosbag and select the rosbag, then output the rosmsg to struct
bag = rosbag('/home/xuhe/Documents/MATLAB/moving.bag');
bag_select = select (bag,"Time",[bag.StartTime bag.EndTime], "Topic","/rtk");
msgs = readMessages(bag_select,"DataFormat","struct");

%creating the variables
time = ones(1,139);
altitude = ones(1,139);
latitude = ones(1,139);
longitude = ones(1,139);
utmEasting = ones(1,139);
utmNorthing = ones(1,139);
i=1;
%sending the msgs value to variables
while i<140
    altitude(1,i) = msgs{i,1}.Altitude;
    latitude(1,i) = msgs{i,1}.Latitude;
    longitude(1,i) = msgs{i,1}.Longitude;
    utmEasting(1,i) = msgs{i,1}.UtmEasting;
    utmNorthing(1,i) = msgs{i,1}.UtmNorthing;
    time(1,i)=i;
    i = i+1;
end

%plot

figure(1);

subplot(2,2,1:2);
plot (time,altitude);
title('altitude change in walking data');
xlabel('time(s)');
ylabel('altitude(m)');

subplot(2,2,3);
plot (latitude,longitude);
title('walking trajectory (lat,lon)');
xlabel('latitude');
ylabel('longitude');

subplot(2,2,4);
plot (utmNorthing,utmEasting);
title('walking trajectory (utm)');
xlabel('utmNorthing');
ylabel('utmEasting');
