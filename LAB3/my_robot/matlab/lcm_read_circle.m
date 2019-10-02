%% Reading LCM log file
% add the lcm.jar file to the matlabpath - need to only do this once
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath '../lcm-spy/exlcm/GPS_readings.java'
javaaddpath '../lcm-spy/exlcm/IMU_readings.java'
javaaddpath '../lcm-spy/my_types.jar'


% open log file for reading

log_file = lcm.logging.Log('../log/circle', 'r'); 

% now read the file 
% here we are assuming that the channel we are interested in is RDI. Your channel 
% name will be different - something to do with GPS
% also RDI has fields altitude and ranges - GPS will probably have lat, lon, utmx,
% utmy etc
utm_t = [];
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
      imu = exlcm.IMU_readings(ev.data);

      % now you can do things like depending upon the rdi_t struct that was defined
      imu_data(n_imu, :) = [imu.timestamp, imu.yaw, imu.pitch, imu.roll,...
                         imu.magnetic_x, imu.magnetic_y, imu.magnetic_z,...
                         imu.acceleration_x, imu.acceleration_y, imu.acceleration_z,...
                         imu.angular_velocity_x, imu.angular_velocity_y, imu.angular_velocity_z];
      n_imu = n_imu + 1;
   end
   
   if strcmp(ev.channel, 'GPS')
       
      % build rdi object from data in this record
      gps = exlcm.GPS_readings(ev.data);
      utm_t(n_gps) = gps.timestamp;
      utm_x(n_gps) = gps.easting;
      utm_y(n_gps) = gps.northing;
      utm_z(n_gps) = gps.altitude;
      n_gps = n_gps + 1;
    end
  catch err   % exception will be thrown when you hit end of file
     break;
  end
end


%% Megnetometer and GPS signal visualization

figure();
plot3(imu_data(:, 5), imu_data(:, 6), imu_data(:, 7));
xlabel('x reading (Gauss)');
ylabel('y reading (Gauss)');
zlabel('z reading (Gauss)');
title('Raw megnetometer');

utm_x = utm_x - utm_x(1);
utm_y = utm_y - utm_y(1);
utm_t = utm_t - utm_t(1);
utm_t = utm_t / 1e6;
figure();
plot3(utm_x, utm_y, utm_z, '.');
xlabel('easting (m)');
ylabel('northing (m)');
zlabel('altitude (m)');
legend('GPS signal');
title('GPS signal');


%% 5. Estimate the heading (yaw)
% 5.1 Megnetometer calibaration

megnetometer_raw_xy = imu_data(1:1000,5:6);
[r_major,r_minor,x0,y0,phi] = ellipse_fit(megnetometer_raw_xy(:,1),megnetometer_raw_xy(:,2));

% calibration follows steps in
% https://www.sensorsmag.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects

% offset
offset = [x0,y0];
megnetometer_xy = megnetometer_raw_xy - offset;

% rotation
R = [cos(phi),  sin(phi);
     -sin(phi), cos(phi)];
megnetometer_xy = (R * megnetometer_xy')';
size(megnetometer_xy)

% scale
scale = r_minor / r_major;
megnetometer_xy(:,1) = megnetometer_xy(:,1) * scale;

% rotate back
megnetometer_xy = (R' * megnetometer_xy')';

figure();
hold;
plot(megnetometer_raw_xy(:,1),megnetometer_raw_xy(:,2));
plot(megnetometer_xy(:,1),megnetometer_xy(:,2));
axis equal;
title('Megnetometer before and after calibration');
xlabel('x reading (Gauss)');
ylabel('y reading (Gauss)');
legend('raw','calibrated');

save('./megnetometer_calibration','offset','R');
