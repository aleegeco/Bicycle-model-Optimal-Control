% Function which computes the skidpad circuit and Plot it. 
% This is a slightly difference version for the animation, since it gives
% also back the points that can be used to plot the circuit outside the
% function
% Input: Radius of the track, width of the road
% Output: [] - Plot of the track

function [x_int_1, y_int_1, x_int_2, y_int_2, x_traj_1, y_traj_1,x_traj_2, y_traj_2, x_ext, y_ext] = skidpad_track_anim(r_track,width_road)

% 21.25 external radius =>  9.125 r_track 
% 18.25 internal radius     3 width_road


th = 0:pi/120:2*pi;

% cut the theta extension of external circles to avoid overlapping on the
% track
al = acos(r_track/(r_track+width_road/2));
th_ext_1 = al:pi/120:(2*pi-al+pi/360);
th_ext_2 = -(pi-al):pi/120:(pi-al+pi/360);

% generation of external circles
x_ext_1 = (r_track+width_road/2)*cos(th_ext_1) - r_track;
x_ext_2 = (r_track+width_road/2)*cos(th_ext_2) + r_track;

y_ext_1 = (r_track+width_road/2)*sin(th_ext_1);
y_ext_2 = (r_track+width_road/2)*sin(th_ext_2);

x_ext = [x_ext_1, x_ext_2];
y_ext = [y_ext_1, y_ext_2];
% generation of internal circles
x_int_1 = (r_track-width_road/2)*cos(th) - r_track;
x_int_2 = (r_track-width_road/2)*cos(th) + r_track;

y_int_1 = (r_track-width_road/2)*sin(th);
y_int_2 = (r_track-width_road/2)*sin(th);

% generation of internal trajectories
x_traj_1 = r_track*cos(th) - r_track;
x_traj_2 = r_track*cos(th) + r_track;

y_traj_1 = r_track*sin(th);
y_traj_2 = r_track*sin(th);

end