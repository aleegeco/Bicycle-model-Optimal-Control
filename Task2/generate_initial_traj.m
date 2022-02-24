%PCcontrol

clear, close all
load("params.mat")
Ts = 61;
dt = params.model.dt; 
nsteps = Ts/dt;
 
xx_d = zeros(3,nsteps);
xx_s = zeros(6,nsteps);

xx_s(:,1) = [0;0;pi/2;2;0;0];

               
Kp = 55000;      
Ki = 0.45*Kp;       

a = Kp + Ki*dt/2;
b = -Kp + Ki*dt/2;


[xx_d(1,:), xx_d(2,:), xx_d(3,:)] = trajectory(nsteps,9.125);

% %% PI Controller
error = zeros(2,nsteps);
uu_s = zeros(2,nsteps);

% first iteration outside the cycle
steer = 15.5;
ks = 10;

error(1,1) = (norm(xx_s([1,2],1)-xx_d([1, 2],1)));

v(1) = 0;
uu_s(1,1) = - deg2rad(steer);
uu_s(2,1) = a*error(1,1);
[x_plus,~,~] = dynamics_ale(xx_s(:,1),uu_s(:,1),params,zeros(6,1));
xx_s(:,2) = x_plus;
second_circle = 0;
for i=2:nsteps
    
    xx_s(:,i) = x_plus;
    
    error(1,i) = (norm(xx_s([1,2],i)-xx_d([1, 2],i)));    
    
    
    v(i) = sqrt(x_plus(1)^2 + x_plus(2)^2);
    
    uu_s(1,i) = -deg2rad(steer); 
    
%     Condition to change the constant steering angle once the first circle
%     is completed
    if xx_s(1,i) > -1 && xx_s(1,i) < 1 && xx_s(2,i) > -0.2 && xx_s(2,i) < 0.2 && i > nsteps/4 || second_circle
        uu_s(1,i) = -uu_s(1,i);
        second_circle = 1;
    end
    
    uu_s(2,i) = a*error(1,i) + b*error(1,i-1); % discrete PI 
    
    [x_plus,~,~] = dynamics_ale(xx_s(:,i),uu_s(:,i),params,zeros(6,1));
    
    
end



figure
hold on
plot(xx_s(1,:), xx_s(2,:), 'r','LineWidth',2);
grid on, zoom on
xlabel("X(m)")
ylabel("Y(m)")
skidpad_track(9.125,3)
hold off

init_skidpad_1.xx = xx_s;
init_skidpad_1.uu = uu_s;
save("init_skidpad_1.mat");
