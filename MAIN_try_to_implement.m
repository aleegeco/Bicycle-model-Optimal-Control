% % % % % Basic script to test the dynamics 
load("params.mat")
dt = params.model.dt;
rad = deg2rad(30);
Ts = 27;
steps = Ts/dt;
u = [-rad;500];
x_o = [0;0;pi/2;0.3;0;0];
store = zeros(6,steps);

x = x_o;
for i=1:steps
    if i == steps/2
        u = [rad,500];
    end
  [x_plus,dfx,dfu] = dynamics_ale(x,u,params,rand(6,1));
  x = x_plus;
  store(:,i) = x_plus;
  A = dfx';
  B = dfu';
 
end

figure
plot(1:steps,store(1,:))
xlabel("time")
ylabel("X(m)")
hold on
plot(1:steps,store(2,:))
xlabel("time")
ylabel("Y(m)")
hold off

figure
plot(store(1,:),store(2,:))
xlabel("X(m)")
ylabel("Y(m)")

figure
plot(store(3,:),store(6,:))
xlabel("yaw angle")
ylabel("angular velocity")

figure
plot(store(4,:),store(5,:))
xlabel("longitudinal velocity")
ylabel("lateral velocity")