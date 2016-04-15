% Example script for plotting eeva data.

close all; % plots

if ~exist('d', 'var')
   disp('Variable d doesnt exist. Make sure to load .mat file first.')
   return
end

rad2deg = 180 / pi;
time = d(:,1);
tilt = d(:,2) * rad2deg;
wave = d(:,3);
left_current = d(:,4);
left_speed = d(:,8);

figure(1)
plot(time, tilt)
grid on
title('Tilt versus Time')
xlabel('Time (sec)')
ylabel('Tilt (deg)')

figure(2)
plot(time, wave, time, left_speed)
grid on
title('Wheel Velocity Tracking')
xlabel('Time (sec)')
ylabel('Wheel Velocity (m/s)')

figure(3)
plot(time, left_current)
grid on
title('Motor Current versus Time')
xlabel('Time (sec)')
ylabel('Current (amps)')
