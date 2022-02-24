% Script which computes a lane change based on a simple discrete PI
% controller. We will use the results obtained here in order to properly
% initialize the DDP algorithm with a state-input trajectory since for our
% dynamic model it results to be very sensitive to the initialization we
% gave it. 
% The PI is used only on Fx, instead on the steering angle we will use a
% geometric pointer.

clear, close all
load("params.mat")
Ts = 70;
dt = params.model.dt; 
nsteps = Ts/dt;
 
% Initialization parameters for PI controller

xx_d = zeros(3, nsteps);
xx = zeros(6,nsteps);
x = [0;0;0;7.5;0;0]; % initial state
store = zeros(6,nsteps);

weight = 0.7;   % this param from 0 to 1 represents the importance for the 
                % veichle of being parallel to the trajectory w.r.t. being
                % on it.
Kp = 2000;       % proportional gain
Ki = 0.45*Kp;       % Integral gain

a = Kp + Ki*dt/2; % discrete PI controller formulation u(k) = a*e(k) + b*e(k-1)
b = -Kp + Ki*dt/2;

% Parameters for trapezoidal reference trajectory

v0 = 8;
v1 = 10;

v = linspace(v0, v1, nsteps); % we impose a linear velocity for the reference generation

% trapezoidal reference trajectory generation
for i=2:nsteps
    [xx_d(2,i-1), dy ] = trapezoidal_curve(xx_d(1,i-1));
    xx_d(1,i) = xx_d(1,i-1) + v(i)*dt*(1 - dy)^0.5;
    xx_d(3,i) = atan(dy);
end
xx_d(2,nsteps)=trapezoidal_curve(xx_d(1,nsteps));


% Controller Start

% Initialization of error and Input vector
error = zeros(1,nsteps);
uu = zeros(2,nsteps);



% % FIRST ITERATION OUTSIDE THE CYCLE

%  Geometrical Pointer towards the reference curve
phi_pointer = atan2(xx_d(2,i)- x(2), xx_d(1,i)- x(1));

% Computation of the steering angle
uu(1,1) = -x(3) + ((1- weight)*phi_pointer + weight*xx_d(3,1));

%  Computation of the error in terms of distance w.r.t. the reference curve
error(1,1) = (norm(xx_d([1,2],1)-x([1, 2])));

% First step computation of the Fx
uu(2,1) = a*error(1,1) + 0;

% Computation of x_plus
x_plus = dynamics_ale(x,uu(:,1),params,zeros(6,1));

x = x_plus;
xx(:,1) = x_plus;

for i=2:nsteps
    
    phi_pointer = atan2(xx_d(2,i)- x(2), xx_d(1,i)- x(1));
    
    uu(1,i) = -x(3) + ((1- weight)*phi_pointer + weight*xx_d(3,i));
    
    error(1,i) = (norm(xx_d([1,2],i)-x([1, 2])));   % error as norm of the distance of the vehicle w.r.t. the reference
    
    uu(2,i) = a*error(1,i) + b*error(1,i-1) ; % discrete PI for Fx - A*error(t) + B*error(t+1)
    
    x_plus = dynamics_ale(x,uu(:,i),params,zeros(6,1)); % call dynamics to compute the next state
    
    x = x_plus;
    xx(:,i) = x_plus; % store
end



figure
hold on
title("Lane Change PI controller")
plot(1:nsteps, xx(2,:),'LineWidth',1.5)
plot(1:nsteps, xx_d(2,:), '--','LineWidth', 2)
legend("Position of the vehicle","Trap. Reference","Location","SouthEast")
grid on, zoom on
xlabel("steps")
ylabel("Y(m)")

figure
hold on
title("Steering Angle")
plot(1:nsteps, uu(1,:),"LineWidth",1.5)
zoom on, grid on
hold off
% Variables we'll use to initialize the DDP algorithm

init.xx = xx(:,:);
init.uu = uu(:,:);
save("init.mat")

        