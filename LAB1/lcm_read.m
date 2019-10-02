% add the lcm.jar file to the matlabpath - need to only do this once
javaaddpath /usr/local/share/java/lcm.jar
%javaaddpath './gps_lcm/gps.class'
javaaddpath '/home/zxp/lcm-1.3.1/examples/lcm-spy/exlcm/gps.java'
javaaddpath '/home/zxp/lcm-1.3.1/examples/lcm-spy/my_types.jar'


% Let’s assume the logging file is lcm-l.02 in the dir below
% open log file for reading

log_file = lcm.logging.Log('./log/lcm-log-2019-01-31-19:10:58', 'r'); 

% now read the file 
% here we are assuming that the channel we are interested in is RDI. Your channel 
% name will be different - something to do with GPS
% also RDI has fields altitude and ranges - GPS will probably have lat, lon, utmx,
% utmy etc
utm_x = [];
utm_y = [];
i = 0;

%while true
for i = 1:1000 
   try
   ev = log_file.readNext();
   
   % channel name is in ev.channel
   % there may be multiple channels but in this case you are only interested in RDI channel
   if strcmp(ev.channel, 'GPS')
       
      % build rdi object from data in this record
      gps = exlcm.gps(ev.data);

      % now you can do things like depending upon the rdi_t struct that was defined
      lat = gps.lat;
      lon = gps.lon;
      alt = gps.alt;
      utm_x(i) = gps.utm_x;
      utm_y(i) = gps.utm_y;
      timestamp = gps.time;  % (timestamp in microseconds since the epoch)
      i = i + 1;
    end
  catch err   % exception will be thrown when you hit end of file
     break;
  end
end

utm_x = utm_x(5:250);
utm_y = utm_y(5:250);
utm_x = utm_x - min(utm_x);
utm_y = utm_y - min(utm_y);
xlabel('m');
ylabel('m');
legend('GPS signal');
plot(utm_x, utm_y,'.');
% plot(utm_x(1:111), utm_y(1:111),'.r');
% hold;
% plot(utm_x(112:end), utm_y(112:end),'.b');