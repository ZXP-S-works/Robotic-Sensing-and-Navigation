%% Reading LCM log file
% add the lcm.jar file to the matlabpath - need to only do this once
javaaddpath /usr/local/share/java/lcm.jar
javaaddpath '../lcm-spy/exlcm/GPS_readings.java'
javaaddpath '../lcm-spy/exlcm/IMU_readings.java'
javaaddpath '../lcm-spy/my_types.jar'


% open log file for reading

log_file = lcm.logging.Log('../log/MFA', 'r'); 

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
for i = 1:20000
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
plot3(imu_data(:, 5), imu_data(:, 6), imu_data(:, 7),'.');
xlabel('x reading (Gauss)');
ylabel('y reading (Gauss)');
zlabel('z reading (Gauss)');
title('raw megnetometer');

utm_x = utm_x - utm_x(1);
utm_y = utm_y - utm_y(1);
utm_t = utm_t - utm_t(1);
utm_t = utm_t / 1e6;
figure();
plot3(utm_x, utm_y, utm_z, 'o');
xlabel('easting (m)');
ylabel('northing (m)');
zlabel('altitude (m)');
legend('GPS signal');
title('GPS signal');


%% 5. Estimate the heading (yaw)
% 5.1 Megnetometer calibaration

t_0 = 1;
t_1 = size(imu_data,1);
megnetometer_raw_xy = imu_data(t_0:t_1,5:6);
[r_major,r_minor,x0,y0,phi] = ellipse_fit(megnetometer_raw_xy(:,1),megnetometer_raw_xy(:,2));
load('megnetometer_calibration.mat');

% calibration follows steps in
% https://www.sensorsmag.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects

% offset
offset = [x0,y0];
megnetometer_xy = megnetometer_raw_xy - offset;
% 
% % rotation
% R = [cos(phi),  sin(phi);
%      -sin(phi), cos(phi)];
% megnetometer_xy = (R * megnetometer_xy')';

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


% 5.2 Intergrating gyro and comparing with megnetometer

imu_yaw = imu_data(t_0:t_1,2) *pi / 180;
imu_yaw = unwrap(imu_yaw);
imu_yaw = imu_yaw - imu_yaw(1);
angular_velocity_z = imu_data(t_0:t_1,13);
time_stamp = imu_data(t_0:t_1,1);
time_stamp = (time_stamp - min(time_stamp)) / 1e6;
gyro_yaw = cumtrapz(time_stamp,angular_velocity_z);
megnetometer_tan = megnetometer_xy(t_0:t_1,2) ./ megnetometer_xy(t_0:t_1,1);
megnetometer_yaw = atan(megnetometer_tan) + pi * (megnetometer_xy(t_0:t_1,1) < 0);
megnetometer_yaw = -megnetometer_yaw;   % orientation related to megnetic field
megnetometer_yaw = unwrap(megnetometer_yaw);
megnetometer_yaw = megnetometer_yaw - megnetometer_yaw(1);
% for i = 1:size(megnetometer_tan)
%     if megnetometer_xy(i,1) > 0
%         megnetometer_yaw(i) = atan(megnetometer_tan(i));
%     else
%         megnetometer_yaw(i) = pi + atan(megnetometer_tan(i));
%     end
% end

figure();
hold();
plot(time_stamp,gyro_yaw);
plot(time_stamp,imu_yaw);
plot(time_stamp,megnetometer_yaw);
title('Yaw: Gyroscope vs Megnetometer');
xlabel('time (s)');
ylabel('orientation (rad)');
legend('gyroscope','IMU','megnetometer');


% 5.3 Appling complement filter

% Please refer https://sites.google.com/site/myimuestimationexperience/filters/complementary-filter

% How to choose alpha?
% alpha=(tau)/(tau+dt) where tau is the desired time constant (how fast you
% want the readings to respond) and dt = 1/fs where fs is your sampling 
% frequency. This equation is derived from filter/control theory will put a 
% link to this as soon as I get it.
tau = 1 / 1000;
dt = 1 / 40;
alpha = tau / (tau + dt);
alpha = 0.95;
ratio = 0.5;

% Equation for low-pass filter:
% y[n]=(1-alpha).x[n]+alpha.y[n-1]            //use this for angles obtained from megnetometer/accelerometers
% x[n] is the pitch/roll/yaw that you get from the accelerometer
% y[n] is the filtered final pitch/roll/yaw which you must feed into the next phase of your program
% Equation for high-pass filter:
% y[n]=(1-alpha)y[n-1]+(1-alpha)(x[n]-x[n-1])    //use this for angles obtained from gyroscopes
% x[n] is the pitch/roll/yaw that you get from the gyroscope
% y[n] is the filtered final pitch/roll/yaw which you must feed into the next phase of your program
low_pass_yaw = zeros(size(megnetometer_yaw));
high_pass_yaw = zeros(size(megnetometer_yaw));

for i = 2:size(megnetometer_yaw,1)
    low_pass_yaw(i) = (1 - alpha) * megnetometer_yaw(i)...
                      + alpha * low_pass_yaw(i-1);
    high_pass_yaw(i) = (1 - alpha) * high_pass_yaw(i-1) + (1 - alpha) * (gyro_yaw(i) - gyro_yaw(i-1));
end

high_pass_yaw = 500 * high_pass_yaw;
complement_yaw = ratio * low_pass_yaw + 1 * high_pass_yaw;
complement_yaw = complement_yaw * 2;

% imu_yaw = imu_yaw - imu_yaw(1);

% n is the current sample indicator.
% alpha is related to time-constant. it defines the boundary where the 
% accelerometer readings stop and the gyroscope readings take over and 
% vice-versa. It controls how much you want the output to depend on the 
% current value or a new value that arrives. Both the alpha's have to be the 
% same. alpha is usually > 0.5 using the definitions above.

% ?????????????????????????????????????????????????????????????????????????
figure();
hold();
plot(time_stamp(1:100:end),complement_yaw(1:100:end));
plot(time_stamp(1:100:end),imu_yaw(1:100:end),'.');
plot(time_stamp(1:100:end),gyro_yaw(1:100:end));
plot(time_stamp(1:100:end),megnetometer_yaw(1:100:end));
plot(time_stamp(1:100:end),low_pass_yaw(1:100:end));
plot(time_stamp(1:100:end),high_pass_yaw(1:100:end));
title('Yaw: Appling Complement Filter');
xlabel('time (s)');
ylabel('orientation (rad)');
legend('complement filter','IMU yaw','gyroscope','megnetometer',...
       'low-pass','high-pass');


%% 6 Estimate the forward velocity

% 6.1 Integrate the forward acceleration to estimate the forward velocity.
acceleration_x = imu_data(t_0:t_1,8);
forward_velocity_accelerometer = cumtrapz(time_stamp,acceleration_x);
figure();
plot(time_stamp,forward_velocity_accelerometer);
title('Forward velocity integrated by accelerometer');
xlabel('time (s)');
ylabel('velocity (m/s)');


% 6.2 Additionally, calculate an estimate of the velocity from your GPS measurements
% Plot both the velocity estimates.
utm_t_0 = 1;
utm_t_1 = find(utm_t < time_stamp(t_1),1,'last');
utm_dt = utm_t(2:end) - utm_t(1:end-1);
utm_dt = [utm_dt(1),utm_dt];
utm_dx = utm_x(2:utm_t_1) - utm_x(1:utm_t_1-1);
utm_dx = [utm_dx(1),utm_dx];
utm_dy = utm_y(2:utm_t_1) - utm_y(1:utm_t_1-1);
utm_dy = [utm_dy(1),utm_dy];
forward_velocity_gps = sqrt(utm_dx(utm_t_0:utm_t_1) .^2 + utm_dy(utm_t_0:utm_t_1) .^2)...
                       ./ utm_dt(utm_t_0:utm_t_1);
figure();
hold;
plot(time_stamp,forward_velocity_accelerometer);
plot(utm_t(utm_t_0:utm_t_1),forward_velocity_gps);
title('Forward velocity by accelerometer vs GPS');
legend('velocity by GPS','velocity by accelerometer');
xlabel('time (s)');
ylabel('velocity (m/s)');

% 6.3 Make adjustments to the acceleration measurements to make the velocity plot
% more reasonable. Plot the adjusted velocity.
% v_gps_mean = mean(forward_velocity_gps);
% v_gps_var = var(forward_velocity_gps);
% v_accl_mean = mean(forward_velocity_accelerometer);
% v_accl_var = var(forward_velocity_accelerometer);
% forward_velocity_adjusted = forward_velocity_accelerometer...
%                             - v_accl_mean;
% forward_velocity_adjusted = forward_velocity_adjusted .* (sqrt(v_gps_var) / sqrt(v_accl_var));
% forward_velocity_adjusted = forward_velocity_adjusted...
%                             + v_gps_mean;
area_gps = trapz(utm_t,forward_velocity_gps);
area_accl = trapz(time_stamp,forward_velocity_accelerometer);
compensate_ratio = (area_gps - area_accl) *2 / time_stamp(end)^2;
forward_velocity_adjusted = forward_velocity_accelerometer...
                            + time_stamp * compensate_ratio;
forward_velocity_adjusted = forward_velocity_adjusted...
                            .* (forward_velocity_adjusted > 0);

figure();
hold;
plot(time_stamp,forward_velocity_adjusted);
plot(utm_t(utm_t_0:utm_t_1),forward_velocity_gps);
title('Forward velocity adjusted by accelerometer vs GPS');
legend('velocity by GPS','velocity adjusted by accelerometer');
xlabel('time (s)');
ylabel('velocity (m/s)');

%% 7. Integrate IMU data to obtain displacement and compare with GPS.

% 7.1 Compute ωX and compare it to y obs
omega_velocity = angular_velocity_z .* forward_velocity_adjusted;

figure();
hold;
p2 = plot(time_stamp(1:10:end),imu_data(t_0:10:t_1,9),'r');
p2.Color(4) = 0.7;
plot(time_stamp(1:10:end),omega_velocity(1:10:end),'LineWidth',2);
title('Y acceleration: omega*v_x vs Y acceleration');
legend('$\dot{Y}$','$\omega * \dot{X}$','Interpreter','latex');
xlabel('time (s)');
ylabel('acceleration (m^2/s)');

% 7.2 Integrate it to estimate the trajectory of the vehicle (x e ,x n ).
% Compare the estimated trajectory with the GPS track by plotting
% them on the same plot.
imu_x = 0;
imu_y = 0;
for i = 2:size(forward_velocity_adjusted)
    imu_x(i) = imu_x(i-1) +...
               cos(-gyro_yaw(i)) * forward_velocity_adjusted(i-1) * dt;
    imu_y(i) = imu_y(i-1) +...
               sin(-gyro_yaw(i)) * forward_velocity_adjusted(i-1) * dt;
end

% estimate_init_vec = [sum(imu_x(1:200)) - 200*imu_x(1), sum(imu_y(1:200)) - 200*imu_y(1)];
% gps_init_vec = [sum(utm_x(1:10)) - 10*utm_x(1), sum(utm_y(1:10)) - 10*utm_y(1)];
% R_estimate2gps = inv((estimate_init_vec' * estimate_init_vec)...
%                  * inv(gps_init_vec' * estimate_init_vec));

% theta_vec = vrrotvec([estimate_init_vec,0],[gps_init_vec,0]);
% theta_vec = theta_vec(4);
theta_vec = 0.84*pi;
R_estimate2gps = [cos(theta_vec), sin(theta_vec);
                  -sin(theta_vec),cos(theta_vec)];
imu_xy = 0.85 * R_estimate2gps * [imu_x; imu_y];

figure();
hold;
plot(imu_xy(1,:),imu_xy(2,:));
%plot(imu_x, imu_y);
plot(utm_x(utm_t_0:utm_t_1), utm_y(utm_t_0:utm_t_1));
xlabel('easting (m)');
ylabel('northing (m)');
zlabel('altitude (m)');
legend('estimate trajectory','GPS signal');
axis equal;
title('GPS signal');

% 7.3 Estimate x c
% use equation v = V + w x r => V = v - w x r
x_c = 0.25;
est_utm_x = cumtrapz(time_stamp,cos(angular_velocity_z) * x_c)';
est_utm_y = cumtrapz(time_stamp,-sin(angular_velocity_z) * x_c)';
est_utm_x = est_utm_x + imu_x;
est_utm_y = est_utm_y + imu_y;
est_utm_xy = 0.85 * R_estimate2gps * [est_utm_x; est_utm_y];

figure();
hold;
plot(imu_xy(1,:),imu_xy(2,:));
plot(est_utm_xy(1,:),est_utm_xy(2,:));
%plot(imu_x, imu_y);
plot(utm_x(utm_t_0:utm_t_1), utm_y(utm_t_0:utm_t_1));
xlabel('easting (m)');
ylabel('northing (m)');
zlabel('altitude (m)');
legend('estimate trajectory','estimate x_c','GPS signal');
axis equal;
title('Estimate x_c');