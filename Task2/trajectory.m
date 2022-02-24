% Function which computes the reference trajectory (centerline) of the
% skidpad based on the simulation steps only. 
% Input: n_steps, radius of the track
% Output: reference x,y,phi

function [x_traj, y_traj, phi_traj] = trajectory(n_steps,r_track)

x_traj = zeros(1,n_steps);
y_traj = zeros(1,n_steps);
th = linspace(-pi,pi,n_steps/2);
th_1 = linspace(0,2*pi,n_steps/2);
phi_traj = zeros(1,n_steps);

for i=1:(n_steps/2)
        x_traj(i) = r_track*cos(-th(i)) + r_track;
        y_traj(i) = r_track*sin(-th(i));
        phi_traj(i) = -th(i) - pi/2;
end
for i=(n_steps/2):n_steps-1
        x_traj(i) = r_track*cos(th_1(i + 1 - n_steps/2)) - r_track;
        y_traj(i) = r_track*sin(th_1(i + 1 - n_steps/2));
        phi_traj(i) = th_1(i + 1 - n_steps/2) - 3*pi/2;
end
x_traj(end) = 0;
y_traj(end) = 0;
phi_traj(end) = pi/2;
end
