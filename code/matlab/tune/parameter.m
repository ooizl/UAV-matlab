kf = 5*10-8;
km = 5*10-8;

%drag coefficients
k1 = 1.1*10^-7;
k2 = 1.1*10^-7;
k3 = 1.1*10^-7;

%rotational drag coefficients
c1 = 1.1*10^-7;
c2 = 1.1*10^-7;
c3 = 1.1*10^-7;

% distance of each rotor from the vehicle’s center of gravity m
L=0.25;

%mass kg
m = 1.535;

%Gravitational acceleration constant m/kg2
g = 9.81;

%moment of inertia kg/m2
Ix = 0.04;
Iy = 0.04;
Iz = 0.1;
%position reference
x_ref = 0
y_ref = 0

save('parameters.mat')