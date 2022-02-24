% Script which apply the LQ control to the linearized system
% dynamics around the Optimal State-Input trajectory obtained using the DDP
% algorithm in task 2.

close all;
load("init_LQR.mat")
load("params.mat")
% load("params_1.mat")
%importing the optimal trajectory computed in the task 3 and saved
xx_opt = init_LQR.xx;
uu_opt = init_LQR.uu;

% parameters that decides if there ia a noise on initial condition or on
% the  position signal while the car is running

plots = 1;
noise_x_0 = 1;
noise_x_t = 1;

delta_0 = 0.35;
delta_t = 0.027;

[nx,steps] = size(xx_opt);
[nu,~] = size(uu_opt);

%inizializing the necessary signals
uu_feed = zeros(nu,steps);
xx_n = zeros(nx,steps);
xx_n_noisy = zeros(nx,steps); 


if noise_x_0
        %a random vector is generated and is summed to the optimal initial
        %conditions
    noise_0 = rand(nx,1)*delta_0;
    xx_n(:,1) = noise_0 + xx_opt(:,1);
else 
    xx_n(:,1) = xx_opt(:,1);
end


A_opt = zeros(nx,nx,steps);
B_opt = zeros(nx,nu,steps);


Q_reg_t = [1e5 0 0 0 0 0;
            0 1e5 0 0 0 0;
            0 0 1e4 0 0 0;
            0 0 0 1e3 0 0;
            0 0 0 0 1e2 0;
            0 0 0 0 0 1e2];

Q_reg_T = [1e6 0 0 0 0 0;
            0 1e6 0 0 0 0;
            0 0 1e3 0 0 0;
            0 0 0 1e3 0 0;
            0 0 0 0 1e3 0;
            0 0 0 0 0 1e3];

R_reg_t = [1e5 0; 0 1e2];

% computation of linearized matrices
for t=1:steps
    [~,df_x,df_u,~] = dynamics_ale(xx_opt(:,t),uu_opt(:,t),params,zeros(6,1));
    A_opt(:,:,t) = df_x';
    B_opt(:,:,t) = df_u';
    
end
PP = zeros(nx,nx,steps);
KK = zeros(nu,nx,steps);

PP(:,:,steps) = Q_reg_T;

% backward iteration for the Riccati equation
for t=(steps-1):-1:1
    PP(:,:,t) = Q_reg_t + A_opt(:,:,t)'*PP(:,:,t+1)*A_opt(:,:,t) ...
        - (A_opt(:,:,t)'*PP(:,:,t+1)*B_opt(:,:,t))*inv((R_reg_t + B_opt(:,:,t)'*PP(:,:,t+1)*B_opt(:,:,t)))*(B_opt(:,:,t)'*PP(:,:,t+1)*A_opt(:,:,t));
    
end

% Feedback gain cycle
for t=1:steps-1
    KK(:,:,t) = -(R_reg_t + B_opt(:,:,t)'*PP(:,:,t+1)*B_opt(:,:,t))\(B_opt(:,:,t)'*PP(:,:,t+1)*A_opt(:,:,t));
end




% input feedback calculation
for t=1:(steps-1)
    
    if noise_x_t 
     %at runtime the noise is introduced on the position xx(1) and xx(2) 
        xx_n_noisy(:,t) = xx_n(:,t);
        xx_n_noisy([1,2],t) = xx_n_noisy([1,2],t) + randn(2,1)*delta_t;
        uu_feed(:,t) = uu_opt(:,t) + KK(:,:,t)*(xx_n_noisy(:,t) - xx_opt(:,t));    
    else 
        uu_feed(:,t) = uu_opt(:,t) + KK(:,:,t)*(xx_n(:,t) - xx_opt(:,t));   
    end
    
    xx_n(:,t+1) = dynamics_ale(xx_n(:,t),uu_feed(:,t),params,zeros(6,1));   
          
end


figure
hold on
title("{\color{green}Optimal} vs {\color{red}Noisy} - Skidpad")
plot(xx_n(1,:),xx_n(2,:),"r","LineWidth",2.5)
plot(xx_opt(1,:),xx_opt(2,:),"g","LineWidth",1.5)
skidpad_track(9.125,3)
zoom on, grid on

if plots
    if noise_x_t || noise_x_0
        figure
        hold on
        title("X(m) - Noisy LQR vs Optimal")
        plot(1:steps, xx_opt(1,:),"--","LineWidth",2)
        plot(1:steps, xx_n(1,:),"LineWidth",1.5)
        xlabel("steps")
        ylabel("$X(m)$","interpreter","latex")
        grid on, zoom on
        legend("$x^{opt}_1$","$x^{lqr}_1$","interpreter","latex","FontSize",14)
        hold off
        
        figure
        hold on
        title("Y(m) - Noisy LQR vs Optimal")
        plot(1:steps, xx_opt(2,:),"--","LineWidth",2)
        plot(1:steps, xx_n(2,:),"LineWidth",1.5)
        grid on, zoom on
        legend("$x^{opt}_2$","$x^{lqr}_2$","interpreter","latex","FontSize",14)
        hold off
    else
        figure
        title("LQR vs Optimal")
        hold on
        subfigure(2,1,1)
        plot(1:steps,xx_n(1,:),"LineWidth",1.5)
        plot(1:steps,xx_opt(1,:),"--","LineWidth",2)
        xlabel("steps")
        ylabel("$X(m)$","interpreter","latex")
        hold off
        
        subfigure(2,1,1)
        plot(1:steps,xx_n(2,:),"LineWidth",1.5)
        plot(1:steps,xx_opt(2,:),"--","LineWidth",2)
        xlabel("steps")
        ylabel("$Y(m)$","interpreter","latex")
        hold off        
    end
end

save('noisy_trajectory','xx_n','uu_feed');
save('optimal_trajectory','xx_opt','uu_opt');

