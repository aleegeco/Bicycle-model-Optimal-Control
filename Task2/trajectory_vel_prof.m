% Function which computes the center trajectory of the skidpad based on the
% space traveled "s"
% Input: s, r_track - space travelled, radius of the track
% Output: x,y,phi reference

function [x_traj, y_traj, phi_traj] = trajectory_vel_prof(s,r_track)

x_traj = zeros(1,length(s));
y_traj = zeros(1,length(s));
th = zeros(1,length(s));
phi_traj = zeros(1,length(s));

for i=1:length(s)
    if s(i)<=2*pi*r_track
        th(i) = pi - s(i)/r_track;
        x_traj(i) = r_track*cos(th(i)) + r_track;
        y_traj(i) = r_track*sin(th(i));
        phi_traj(i) = th(i) - pi/2;
     else
        th(i) = (s(i) - 2*pi*r_track)/r_track;
        x_traj(i) = r_track*cos(th(i)) - r_track;
        y_traj(i) = r_track*sin(th(i));
        phi_traj(i) = th(i) + pi/2;
    end
    
end

end
