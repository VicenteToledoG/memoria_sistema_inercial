function [phi theta psi] = EulerAccel(ax, ay, az, mx, my, mz)

 % Alternative implementation
mu = 0.001;
theta = atan2(-ax, sqrt(ay^2 + az^2));
phi = atan2(ay, sign(az) * sqrt(az^2 + mu * ax^2));
psi=0;