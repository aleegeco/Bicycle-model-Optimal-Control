%% animation

%% load the trajectory to be plotted.
load("noisy_trajectory.mat")
load('optimal_trajectory.mat')

xx_noisy = xx_n;
uu_noisy = uu_feed;

xx_opt = xx_opt(:,:);
uu_opt = uu_opt(:,:);

%% load parameters of the car and the simulation
load("params.mat")
a = params.model.a;
b = params.model.b;


%% initialize the function which gives the points to plot the track
r_track = 9.125;
road_width = 3;
[x_int_1, y_int_1, x_int_2, y_int_2, x_traj, y_traj, x_ext, y_ext] =skidpad_track_anim(r_track, road_width);

%% graphical representation of the car in its reference frame:
car_shape_rel = [3.5,0; 3.5,0.5; 3.45,0.7; 1.5,1; -1.8,1; -1.9,0.6; -1.9,-0.6; -1.8,-1; 1.5,-1; 3.45,-0.7; 3.5,-0.5; 3.5,0]*0.8;
car_ref_rel = [0,0; 1.5+a,0; 1.5+a,0.2; 2.2+a,0; 1.5+a,-0.2; 1.5+a,0];

steer_point_rel = [a, 0];
steer_vector_rel = [0,0; 3,0; 3,0.2; 4,0; 3,-0.2; 3,0];

wheel_distance = 1.2;
wheel_radius = 0.4;
wheel_width = 0.3;
wheel_center_LF_rel = [a, wheel_distance/2];
wheel_center_RF_rel = [a, -wheel_distance/2];
wheel_center_LB_rel = [-b, wheel_distance/2];
wheel_center_RB_rel = [-b, -wheel_distance/2];
wheel_axis_rel = [0,0.7; 0,-0.7];
wheels= [-wheel_radius,wheel_width/2; wheel_radius,wheel_width/2; wheel_radius,-wheel_width/2; -wheel_radius,-wheel_width/2; -wheel_radius,wheel_width/2];

fig = figure;
fig.WindowState = 'maximized';    
hold on

for step = 1:50:length(xx_opt) % to speed up the animation one over a certain
                               % number of steps will be plotted
    % traduce the position in lighter notation:
    x_opt = xx_opt(1, step);
    y_opt = xx_opt(2, step);
    th_opt = xx_opt(3, step);
    delta_opt= uu_opt(1, step);

    x_noisy = xx_noisy(1, step);
    y_noisy = xx_noisy(2, step);
    th_noisy = xx_noisy(3, step);
    delta_noisy= uu_noisy(1, step);

    % intialize the rotation matrices:
    R_main_opt = [cos(th_opt), -sin(th_opt); sin(th_opt), cos(th_opt)];
    R_steer_opt = [cos(delta_opt), -sin(delta_opt); sin(delta_opt), cos(delta_opt)];
    
    R_main_noisy = [cos(th_noisy), -sin(th_noisy); sin(th_noisy), cos(th_noisy)];
    R_steer_noisy = [cos(delta_noisy), -sin(delta_noisy); sin(delta_noisy), cos(delta_noisy)];

    % apply the rotation matrices to give the correct abslute orientation
    % to the various component of the car:
    % opt car
    steer_point_opt = R_main_opt*steer_point_rel';
    wheel_center_LF_opt = R_main_opt*wheel_center_RF_rel';
    wheel_center_RF_opt = R_main_opt*wheel_center_LF_rel';
    wheel_center_LB_opt = R_main_opt*wheel_center_LB_rel';
    wheel_center_RB_opt = R_main_opt*wheel_center_RB_rel';
    % noisy car
    steer_point_noisy = R_main_opt*steer_point_rel';
    wheel_center_LF_noisy = R_main_noisy*wheel_center_RF_rel';
    wheel_center_RF_noisy = R_main_noisy*wheel_center_LF_rel';
    wheel_center_LB_noisy = R_main_noisy*wheel_center_LB_rel';
    wheel_center_RB_noisy = R_main_noisy*wheel_center_RB_rel';

    for i=1:size(car_shape_rel,1)
        car_shape_opt(i,:) = [x_opt,y_opt]' + R_main_opt*car_shape_rel(i,:)';
        car_shape_noisy(i,:) = [x_noisy,y_noisy]' + R_main_noisy*car_shape_rel(i,:)';
    end

    for i=1:size(car_ref_rel,1)
        car_ref_opt(i,:) = [x_opt,y_opt]' + R_main_opt*car_ref_rel(i,:)';
        car_ref_noisy(i,:) = [x_noisy,y_noisy]' + R_main_noisy*car_ref_rel(i,:)';
    end

    for i=1:5
        %back wheels opt:
        LB_wheel_opt(i,:) = [x_opt,y_opt]' + wheel_center_LB_opt + R_main_opt*wheels(i,:)';
        RB_wheel_opt(i,:) = [x_opt,y_opt]' + wheel_center_RB_opt + R_main_opt*wheels(i,:)';
        %front wheels opt:
        rotated_front_wheel_opt(i,:) = R_main_opt*R_steer_opt*wheels(i,:)';
        L_wheel_opt(i,:) = rotated_front_wheel_opt(i,:) + [x_opt,y_opt] + wheel_center_LF_opt';
        R_wheel_opt(i,:) = rotated_front_wheel_opt(i,:) + [x_opt,y_opt] + wheel_center_RF_opt';
        %back wheels noisy:
        LB_wheel_noisy(i,:) = [x_noisy,y_noisy]' + wheel_center_LB_noisy + R_main_noisy*wheels(i,:)';
        RB_wheel_noisy(i,:) = [x_noisy,y_noisy]' + wheel_center_RB_noisy + R_main_noisy*wheels(i,:)';
        %front wheels noisy:
        rotated_front_wheel_noisy(i,:) = R_main_noisy*R_steer_noisy*wheels(i,:)';
        L_wheel_noisy(i,:) = rotated_front_wheel_noisy(i,:) + [x_noisy,y_noisy] + wheel_center_LF_noisy';
        R_wheel_noisy(i,:) = rotated_front_wheel_noisy(i,:) + [x_noisy,y_noisy] + wheel_center_RF_noisy';
    end

    for i=1:2
        %front wheels axis opt:
        rotated_wheel_axis_opt(i,:) = R_main_opt*R_steer_opt*wheel_axis_rel(i,:)';
        L_wheel_axis_opt(i,:) = rotated_wheel_axis_opt(i,:) + [x_opt,y_opt] + wheel_center_LF_opt';
        R_wheel_axis_opt(i,:) = rotated_wheel_axis_opt(i,:) + [x_opt,y_opt] + wheel_center_RF_opt';
        %front wheels axis noisy:
        rotated_wheel_axis_noisy(i,:) = R_main_noisy*R_steer_noisy*wheel_axis_rel(i,:)';
        L_wheel_axis_noisy(i,:) = rotated_wheel_axis_noisy(i,:) + [x_noisy,y_noisy] + wheel_center_LF_noisy';
        R_wheel_axis_noisy(i,:) = rotated_wheel_axis_noisy(i,:) + [x_noisy,y_noisy] + wheel_center_RF_noisy';
    end


    hold on
    axis equal, zoom on, grid on
    %axis ([x-2,x+7,-1,3]);
    axis([-20, 20, -15, 15]);
    
    % plot the track
    plot(x_int_1, y_int_1, 'b');
    plot(x_int_2, y_int_2, 'b');
    plot(x_ext, y_ext, 'b');
    plot(x_traj, y_traj, 'r--');
    
    %plot the opt car moving
    plot(car_shape_opt(:,1), car_shape_opt(:,2),'b');
%     plot(car_ref_opt(:,1),car_ref_opt(:,2),'b');
    plot(L_wheel_opt(:,1), L_wheel_opt(:,2), 'b');
    plot(R_wheel_opt(:,1), R_wheel_opt(:,2), 'b');
    plot(LB_wheel_opt(:,1), LB_wheel_opt(:,2), 'b');
    plot(RB_wheel_opt(:,1), RB_wheel_opt(:,2), 'b');
    plot(L_wheel_axis_opt(:,1), L_wheel_axis_opt(:,2), 'b-.');
    plot(R_wheel_axis_opt(:,1), R_wheel_axis_opt(:,2), 'b-.');

    %plot the noisy car moving
    plot(car_shape_noisy(:,1), car_shape_noisy(:,2),'r');
%     plot(car_ref_noisy(:,1),car_ref_noisy(:,2),'r');
    plot(L_wheel_noisy(:,1), L_wheel_noisy(:,2), 'r');
    plot(R_wheel_noisy(:,1), R_wheel_noisy(:,2), 'r');
    plot(LB_wheel_noisy(:,1), LB_wheel_noisy(:,2), 'r');
    plot(RB_wheel_noisy(:,1), RB_wheel_noisy(:,2), 'r');
    plot(L_wheel_axis_noisy(:,1), L_wheel_axis_noisy(:,2), 'r-.');
    plot(R_wheel_axis_noisy(:,1), R_wheel_axis_noisy(:,2), 'r-.');

    pause(1e-10);
    clf;

end