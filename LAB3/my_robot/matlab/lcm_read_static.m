% add the lcm.jar file to the matlabpath - need to only do this once
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath '../lcm-spy/exlcm/gps.java'
javaaddpath '../lcm-spy/exlcm/imu.java'
javaaddpath '../lcm-spy/my_types.jar'


% open log file for reading

log_file = lcm.logging.Log('../log/lcm-log-isac-level-table', 'r'); 

% now read the file 
% here we are assuming that the channel we are interested in is RDI. Your channel 
% name will be different - something to do with GPS
% also RDI has fields altitude and ranges - GPS will probably have lat, lon, utmx,
% utmy etc
utm_x = [];
utm_y = [];
utm_z = [];
imu_data = [];
n_gps = 1;
n_imu = 1;

%while true
for i = 1:10000
    try
   ev = log_file.readNext();
   
   % channel name is in ev.channel
   if strcmp(ev.channel, 'IMU')
       
      % build rdi object from data in this record
      imu = exlcm.imu(ev.data);

      % now you can do things like depending upon the rdi_t struct that was defined
      imu_data(n_imu, :) = [imu.yaw, imu.pitch, imu.roll,...
                         imu.magnetic_x, imu.magnetic_y, imu.magnetic_z,...
                         imu.acceleration_x, imu.acceleration_y, imu.acceleration_z,...
                         imu.angular_rate_x, imu.angular_rate_y, imu.angular_rate_z];
      n_imu = n_imu + 1;
    end
   
  catch err   % exception will be thrown when you hit end of file
     break;
  end
end

%% 4. Collect data with IMU and GPS puck
% 4.1 Plot each time series and figure out the noise characteristics of each of the values reported by the IMU.

figure();
plot(imu_data(:, 7:9));
title('Accelerometers reading');
xlabel('data index');
ylabel('m^2/s');
legend('X', 'Y', 'Z');

figure();
plot(1:10:size(imu_data,1),imu_data(1:10:end, 10:12))
title('Angular rate gyros reading');
xlabel('data index');
ylabel('rad/s');
ylim([-5e-3,5e-3]);
legend('X', 'Y', 'Z');

figure();
plot(imu_data(:, 4:6))
title('Magnetometers reading');
xlabel('data index');
ylabel('Gauss');
legend('X', 'Y', 'Z');

figure();
for i = 1:3
    subplot(3,3,i);
    hist(imu_data(:,i+6));
    switch i
        case 1
            title('Accalerometer XYZ');
        case 2
            title('Gyroscop XYZ');
        case 3
            title('Megnetometer XYZ');
    end
    subplot(3,3,i+3);
    hist(imu_data(:,i+9));
    subplot(3,3,i+6);
    hist(imu_data(:,i+3));
end
sgtitle('Noise analysis');
