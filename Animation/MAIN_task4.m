%% animation

% load the trajectory to be plotted.
clc, clear, close all;

load('noisy_trajectory.mat')
load('optimal_trajectory.mat')

xx_noisy = xx_n;
uu_noisy = uu_feed;

xx_opt = xx_opt(:,:);
uu_opt = uu_opt(:,:);

% load parameters of the car and the simulation
load("params.mat")
a = params.model.a;
b = params.model.b;


% initialize the function which gives the points to plot the track
r_track = 9.125;
road_width = 3;
[x_int_1, y_int_1, x_int_2, y_int_2, x_traj_1, y_traj_1,x_traj_2, y_traj_2, x_ext, y_ext] = skidpad_track_anim(r_track, road_width);

% graphical representation of the car in its reference frame:
car_shape_rel = [a,0; -b,0];
car_ref_rel = [0,0; 1.5+a,0; 1.5+a,0.2; 2.2+a,0; 1.5+a,-0.2; 1.5+a,0];

steer_vector_rel = [0,0; 3,0; 3,0.1; 4,0; 3,-0.1; 3,0];

wheel_distance = 1.2;
wheel_radius = 0.4;
wheel_width = 0.3;

wheel_center_F_rel = [a, 0];
wheel_center_B_rel = [-b, 0];

wheel_axis_rel = [0,0.7; 0,-0.7];
wheels= [-wheel_radius,wheel_width/2; wheel_radius,wheel_width/2; wheel_radius,-wheel_width/2; -wheel_radius,-wheel_width/2; -wheel_radius,wheel_width/2];


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
    wheel_center_F_opt = R_main_opt*wheel_center_F_rel';
    wheel_center_B_opt = R_main_opt*wheel_center_B_rel';
    % noisy car
    wheel_center_F_noisy = R_main_noisy*wheel_center_F_rel';
    wheel_center_B_noisy = R_main_noisy*wheel_center_B_rel';
    
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
        B_wheel_opt(i,:) = [x_opt,y_opt]' + wheel_center_B_opt + R_main_opt*wheels(i,:)';
        %front wheels opt:
        rotated_front_wheel_opt(i,:) = R_main_opt*R_steer_opt*wheels(i,:)';
        F_wheel_opt(i,:) = rotated_front_wheel_opt(i,:) + [x_opt,y_opt] + wheel_center_F_opt';
        %back wheels noisy:
        B_wheel_noisy(i,:) = [x_noisy,y_noisy]' + wheel_center_B_noisy + R_main_noisy*wheels(i,:)';
        %front wheels noisy:
        rotated_front_wheel_noisy(i,:) = R_main_noisy*R_steer_noisy*wheels(i,:)';
        F_wheel_noisy(i,:) = rotated_front_wheel_noisy(i,:) + [x_noisy,y_noisy] + wheel_center_F_noisy';
    end

    for i=1:2
        %front wheels axis opt:
        rotated_wheel_axis_opt(i,:) = R_main_opt*R_steer_opt*wheel_axis_rel(i,:)';
        F_wheel_axis_opt(i,:) = rotated_wheel_axis_opt(i,:) + [x_opt,y_opt] + wheel_center_F_opt';
        %front wheels axis noisy:
        rotated_wheel_axis_noisy(i,:) = R_main_noisy*R_steer_noisy*wheel_axis_rel(i,:)';
        F_wheel_axis_noisy(i,:) = rotated_wheel_axis_noisy(i,:) + [x_noisy,y_noisy] + wheel_center_F_noisy';
    end


    hold on
    title("'\fontsize{18}{\color{blue}Noisy Car} vs {\color{red}Optimal Car}")
    axis equal
    %axis ([x-2,x+7,-1,3]);
    axis([-20, 20, -15, 15]);
    
    % plot the track
    plot(x_int_1, y_int_1, 'k');
    plot(x_int_2, y_int_2, 'k');
    plot(x_ext, y_ext, 'k');
    plot(x_traj_1, y_traj_1, 'c--','LineWidth',0.75);
    plot(x_traj_2, y_traj_2, 'c--','LineWidth',0.75);
    
    %plot the opt car moving
    plot(xx_opt(1,[1:step]),xx_opt(2,[1:step]),'r','LineWidth',1);
    plot(car_shape_opt(:,1), car_shape_opt(:,2),'r','LineWidth',1.5);
    plot(F_wheel_opt(:,1), F_wheel_opt(:,2), 'r','LineWidth',1.5);
    plot(B_wheel_opt(:,1), B_wheel_opt(:,2), 'r','LineWidth',1.5);
    plot(F_wheel_axis_opt(:,1), F_wheel_axis_opt(:,2), 'r-.');

    %plot the noisy car moving
    plot(xx_noisy(1,[1:step]),xx_noisy(2,[1:step]),'b','LineWidth',1);
    plot(car_shape_noisy(:,1), car_shape_noisy(:,2),'b','LineWidth',1.5);
    plot(F_wheel_noisy(:,1), F_wheel_noisy(:,2), 'b','LineWidth',1.5);
    plot(B_wheel_noisy(:,1), B_wheel_noisy(:,2), 'b','LineWidth',1.5);
    plot(F_wheel_axis_noisy(:,1), F_wheel_axis_noisy(:,2), 'b-.');
    pause(1e-10);
    clf;

end