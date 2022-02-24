% Script that compute the lane change meaneuver of the Project task 1 by
% using a DDP algorithm

% We want to define a trajectory moving our vehicle from an equilibrium
% to another one. These equilibriums are identified as two different
% position over the lane.

load("params.mat")
load("init.mat")


dt = params.model.dt;
max_iters = 10;
tol = 1e-6;
nx = 6;
nu = 2;
b = params.model.b;
a = params.model.a;


Ts = 30; % simulation time in Seconds

steps = Ts/dt; % Steps

% % calling various plots and parameters % %
plots = 1;
gamma_fixed = 1;
armijo = 0;

% initial condition
x0 = init.xx(:,1);

params.cost.Q = [1e-1 0 0 0 0 0;
    0 1e4 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

params.cost.R = [1e2 0;
    0 1];

params.cost.QT = [1e-1 0 0 0 0 0;
    0 1e5 0 0 0 0;
    0 0 1e3 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];


xx_ref = zeros(nx,steps);
uu_ref = zeros(nu,steps);

xx = zeros(nx, steps, max_iters); 
uu = zeros(nu, steps, max_iters); 

xx_ref(1,:) = linspace(0,400,steps);
xx_ref(4,:) = 15;
xx_ref(2,(steps/3):(steps/2)-1) = linspace(0,1,steps/6);
xx_ref(2,steps/2:end) = 1;
xx_ref(3,steps/2:end) = 0;
xx_ref(6,steps:2:end) = 0;


uu_ref(1,:) = 0;
uu_ref(2,:) = 150;

%(x_0,u_0) state-input trajectory to initialize the algorithm 
xx(:,:,1) = init.xx(:,1:steps); 
uu(:,:,1) = init.uu(:,1:steps);

% initialization of cost function and descent direction
JJ = zeros(max_iters, 1); 
descent = zeros(max_iters,1);

kk = 1;
for t=1:steps-1
    
    stage_cost = stage_cost_DDP_ale(xx(:,t,kk),uu(:,t,kk),xx_ref(:,t),uu_ref(:,t),params);
    JJ(kk) = JJ(kk) + stage_cost;
    
end

term_cost = term_cost_DDP_ale(xx(:,steps,kk),xx_ref(:,steps),params);
JJ(kk) = JJ(kk) + term_cost;

for kk=1:max_iters-1
  
    K = zeros(nu,nx,steps); % feedback gain
    sigma = zeros(nu,steps); % feedforward action
    
    pp = zeros(nx,steps); % costate vector initialization
    PP = zeros(nx,nx,steps); % costate matrix initialization
    
    [~, pp(:,steps), PP(:,:,steps)] = term_cost_DDP_ale(xx(:,steps),xx_ref(:,steps),params);
    
    for t=(steps-1):-1:1
        
%         call dynamics function to compute gradient and tensor contraction
        [~,df_x,df_u,~,~,~,pfxx,pfuu,pfux] = dynamics_ale(xx(:,t,kk), uu(:,t,kk),params,pp(:,t+1));
        
%         call stage cost function to compute it and its gradients
        [~,dl_x,dl_xx,dl_u,dl_uu,dl_ux] = stage_cost_DDP_ale(xx(:,t,kk),uu(:,t,kk), xx_ref(:,t), uu_ref(:,t), params);
        
% %         update Gain and Feedforward term
        KK(:,:,t) = -(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)\(dl_ux + df_u*PP(:,:,t+1)*df_x' + pfux);
        sigma(:,t) = -(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)\(dl_u + df_u*pp(:,t+1));

        
%         update PP Matrix and P vector
        PP(:,:,t) = (dl_xx + df_x*PP(:,:,t+1)*df_x' + pfxx) - KK(:,:,t)'*(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)*KK(:,:,t);
        pp(:,t) = (dl_x + df_x*pp(:,t+1)) - KK(:,:,t)'*(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)*sigma(:,t);
% %         
        descent(kk) = descent(kk) - sigma(:,t)'*sigma(:,t);
        
    end
    
    if armijo
        cc = 0.5;
        beta = 0.5;
        gammas = 1;

        cost_arm = [];

        xx_temp = zeros(nx,steps);
        uu_temp = zeros(nu,steps);

        xx_temp(:,1) = x0;
        JJ_temp = 0;

        for t=1:steps-1
        
            uu_temp(:,t) = uu(:,t,kk) + gammas(end)*sigma(:,t) + KK(:,:,t)*(xx_temp(:,t) - xx(:,t,kk));
        
            xx_temp(:,t+1) = dynamics_ale(xx_temp(:,t), uu_temp(:,t),params, pp(:,t+1));
        
            cost_temp = stage_cost_DDP_ale(xx_temp(:,t), uu_temp(:,t), xx_ref(:,t), uu_ref(:,t), params);
        
            JJ_temp = JJ_temp + cost_temp;
        end
    
        cost_temp = term_cost_DDP_ale(xx_temp(:,steps), xx_ref(:,steps), params);
        JJ_temp = JJ_temp + cost_temp;
            
        cost_arm = [cost_arm; JJ_temp];
        
    
        while cost_arm(end) > JJ(kk) + cc*gammas(end)*descent(kk)
        
            gammas = [gammas; gammas(end)*beta];
            xx_temp(:,1) = x0;
            JJ_temp = 0;
        
            for t = 1:steps-1
            
                uu_temp(:,t) = uu(:,t,kk) + gammas(end)*sigma(:,t) + KK(:,:,t)*(xx_temp(:,t) - xx(:,t,kk));
            
                xx_temp(:,t+1) = dynamics_ale(xx_temp(:,t), uu_temp(:,t), params, pp(:,t+1));
                
            
                cost_temp = stage_cost_DDP_ale(xx_temp(:,t), uu_temp(:,t), xx_ref(:,t), uu_ref(:,t), params);
            
                JJ_temp = JJ_temp + cost_temp;
            end
            
            cost_temp = term_cost_DDP_ale(xx_temp(:,steps), xx_ref(:,steps), params);
            JJ_temp = JJ_temp + cost_temp;
            
            cost_arm = [cost_arm; JJ_temp];
            
        end
    
    gamma_steps = gammas;
    gamma = gammas(end);
    
    else
        gamma = gamma_fixed;
    end
    
    if armijo
        points = 20;
        
        gamma_stepsiz = linspace(1,0,points);
        
        cost_temp = zeros(points,1);
        xx_temp = zeros(nx, steps);
        uu_temp = zeros(nu, steps);
   
    for ii = 1:length(gamma_stepsiz)
        
        gamma_i = gamma_stepsiz(ii);
        xx_temp(:,1) = x0;
        for t = 1:steps-1
            uu_temp(:,t) = uu(:,t,kk) + gamma_i*sigma(:,t) + KK(:,:,t)*(xx_temp(:,t) - xx(:,t,kk));
            
            xx_temp(:,t+1) = dynamics_ale(xx_temp(:,t), uu_temp(:,t), params, pp(:,t+1));
            cost_dummy = stage_cost_DDP_ale(xx_temp(:,t), uu_temp(:,t), xx_ref(:,t), uu_ref(:,t), params);
            JJ_temp = JJ_temp + cost_dummy;
            cost_temp(ii) = cost_temp(ii) + cost_dummy;
        end
        [cost_dummy, ~] = term_cost_DDP_ale(xx_temp(:,steps), xx_ref(:,steps), params);
        cost_temp(ii) = cost_temp(ii) + cost_dummy;
    end
    
    figure(1900);
    
    plot(gamma_stepsiz, min(cost_temp, 5*JJ(kk)), 'LineWidth',2, ...
      'DisplayName','$J(x^k - \gamma\nabla J^k)$');
    hold on;
    grid on;
    zoom on;
    
    plot(gamma_stepsiz, JJ(kk) + gamma_stepsiz*descent(kk), ...
      'r--', 'LineWidth',2,'DisplayName','$J(x^k) + \gamma descent$');
    
    plot(gamma_stepsiz, JJ(kk) + cc*gamma_stepsiz*descent(kk), ...
      'g--', 'LineWidth',2,'DisplayName','$J(x^k) + c \gamma descent$');
    plot(gammas, cost_arm, '*', 'DisplayName', 'Armijo steps');
    zoom on
    
    tit = sprintf('Iter: %d', kk);
    title(tit);
    
    legend('Interpreter','latex', 'FontSize', 12, 'Location','southeast');
    
    hold off
    
    drawnow
    
    end
        
    
%     trajectory update
    xx(:,1,kk+1) = x0;
    for t=1:(steps-1)
        
        uu(:,t,kk+1) = uu(:,t,kk) + gamma*sigma(:,t) + KK(:,:,t)*(xx(:,t,kk+1) - xx(:,t,kk));
        
        [xx(:,t+1,kk+1), ~] = dynamics_ale(xx(:,t,kk+1), uu(:,t,kk+1), params, pp(:,t+1));
        
        [cost_temp, ~] = stage_cost_DDP_ale(xx(:,t,kk+1), uu(:,t,kk+1), xx_ref(:,t), uu_ref(:,t), params);
        
        JJ(kk+1) = JJ(kk+1) + cost_temp;
    end
    
    [cost_temp, ~] = term_cost_DDP_ale(xx(:,steps,kk+1), xx_ref(:,steps), params);
    JJ(kk+1) = JJ(kk+1) + cost_temp;
    
    
    fprintf("Iteration; %d\n", kk);
    fprintf("Descent: %.4e\n", descent(kk));
    fprintf("Cost: %.4e\n", JJ(kk));
    
    
    if abs(descent(kk))<tol
        fprintf("Tolerance reached! \n")
        break;
    end
end

uu(:,steps,kk) = uu(:,steps-1,kk);


% Cost Function Plot
figure(1000);
hold on
title("Cost Function")
plot(1:kk,log(JJ(1:kk)),"LineWidth",2)
xlabel("iterations")
ylabel("$\log(J)$","interpreter","latex")
grid on, zoom on
hold off

% X.Y plot
figure(1020);
hold on;
stairs(1:steps, xx(2,:,kk),'LineWidth',2);
stairs(1:steps, xx_ref(2,:),'--','LineWidth',2);
ylabel('$Y(m)$',"interpreter","latex",'FontSize',16)
xlabel('steps','FontSize',16)
grid on, zoom on
hold off
    
%    
figure(1010);
hold on;
stairs(1:steps, xx(1,:,kk),'LineWidth',2);
stairs(1:steps, xx_ref(1,:),'--','LineWidth',2);
ylabel('$X(m)$',"interpreter","latex",'FontSize',16)
xlabel('steps','FontSize',12)
grid on, zoom on
hold off

if plots
%     Yaw Rate and Yaw Angle plots
    figure(1030)
    subplot(2,1,1)
    hold on
    stairs(1:steps, xx(3,:,kk),"LineWidth",2);
    stairs(1:steps, xx_ref(3,:), "--","LineWidth",2);
    ylabel("$\psi$","interpreter","latex",'FontSize',16)
    xlabel("steps",'FontSize',12)
    title("Yaw Angle")
    legend("$x_{3,t}$","$x_{3,ref}$","interpreter","latex")
    grid on, zoom on
    hold off
    
    subplot(2,1,2)
    hold on
    stairs(1:steps, xx(6,:,kk),"LineWidth",2);
    stairs(1:steps, xx_ref(6,:), "--","LineWidth",2);
    ylabel("$\frac{d}{dt}{\psi}$","interpreter","latex",'FontSize',12)
    xlabel("steps",'FontSize',12)
    title("Yaw rate")
     legend("$x_{6,t}$","$x_{6,ref}$","interpreter","latex")
    grid on, zoom on
    hold off

%   Velocities Plots
    figure(1040)
    subplot(2,1,1)
    hold on
    stairs(1:steps, xx(4,:,kk),"LineWidth",2);
    stairs(1:steps, xx_ref(4,:), "--","LineWidth",2);
    ylabel("$v_x$","interpreter","latex",'FontSize',12)
    xlabel("steps",'FontSize',12)
    title("Longitudinal Velocity")
    legend("$x_{4,t}$","$x_{4,ref}$","interpreter","latex")
    grid on, zoom on
    hold off
    
    subplot(2,1,2)
    hold on
    stairs(1:steps, xx(5,:,kk),"LineWidth",2);
    stairs(1:steps, xx_ref(5,:), "--","LineWidth",2);
    ylabel("$v_y$","interpreter","latex",'FontSize',12)
    xlabel("steps",'FontSize',12)
    title("Lateral Velocity")
    legend("$x_{5,t}$","$x_{5,ref}$","interpreter","latex")
    grid on, zoom on
    hold off
    
%     Inputs Plots
    figure
    subplot(2,1,1)
    hold on
    stairs(1:steps,uu(2,:,kk),'LineWidth',2);
    stairs(1:steps,uu_ref(2,:),'--','LineWidth',2);
    grid on, zoom on
    xlabel('steps','FontSize',12)
    ylabel('$F_x$','interpreter','latex','FontSize',12)
    legend("$u_{2,t}$","$u_{2,ref}$",'interpreter','latex','FontSize',12)
    title("$F_x$","interpreter","latex")
    hold off

    subplot(2,1,2)
    hold on
    stairs(1:steps, uu(1,:,kk),'LineWidth',2)
    stairs(1:steps,uu_ref(1,:),'--','LineWidth',2)
    grid on, zoom on
    xlabel('steps','FontSize',12)
    ylabel('$\delta$','interpreter','latex','FontSize',12)
    legend("$u_{1,t}$", "$u_{1,ref}$",'interpreter','latex','FontSize',12)
    title("Steering Angle")
    hold off
end




