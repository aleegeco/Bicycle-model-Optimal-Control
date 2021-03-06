%  In this script we will compute the optimal trajectory for the Skipad run
%  by using the DDP algorithm, and we will use it as optimal reference trajectory
%  in the last task.


load("params.mat")
load("init_skidpad.mat")

dt = params.model.dt;
tol = 1e-6;

% call for Armijo's and all the Plots
plots = 1;
armijo = 0;
gamma = 1;

Ts = 55;
steps = Ts/dt;

iter =10;

nx = 6;
nu = 2;
r_track = 9.125;

% Initial Conditions - generated by generate_initial_script;
x_0 = init_skidpad.xx(:,1);

xx = zeros(nx,steps,iter);
uu = zeros(nu,steps,iter);

xx_ref = zeros(nx,steps);
uu_ref = zeros(nu,steps);

% Generation of the reference by using the trajectory script
[xx_ref(1,:), xx_ref(2,:), xx_ref(3,:)] = trajectory(steps,9.125);
xx_ref(3,:) = pi/2;

uu_ref(2,:) = 33.5;
% Initialization of Cost Function and Descent direction
J = zeros(iter,1);
descent = zeros(iter,1);

% (x0,u0) - state-input trajectory pair to initialize the algorithm 
 xx(:,:,1) = init_skidpad.xx(:,1:steps);
 uu(:,:,1) = init_skidpad.uu(:,1:steps);

params.cost.Q = [1e4 0 0 0 0 0;
                 0 1e4 0 0 0 0;
                 0 0 1e3 0 0 0;
                 0 0 0 1e0 0 0;
                 0 0 0 0 1e0 0;
                 0 0 0 0 0 1e0];

params.cost.QT = [1e5 0 0 0 0 0;
                  0 1e5 0 0 0 0;
                  0 0 1e2 0 0 0;
                  0 0 0 1e0 0 0;
                  0 0 0 0 1e0 0;
                  0 0 0 0 0 1e0];

params.cost.R = [1e2 0; 
                 0 1e1];


% DDP algorithm

% Computation of the cost at k = 1 for each t
for t=1:steps-1
    stage_cost = stage_cost_DDP_ale(xx(:,t,1),uu(:,t,1),xx_ref(:,steps),uu_ref(:,steps),params);
    J(1) = J(1) + stage_cost;
end

term_cost = term_cost_DDP_ale(xx(:,steps,1),xx_ref(:,steps),params);
J(1) = J(1) + term_cost;

for kk=1:iter-1
    
%     Initialize gain K and Feedforward term Sigma
    KK = zeros(nu,nx,steps);
    sigma = zeros(nu,steps);
    
    pp = zeros(nx,steps);
    PP = zeros(nx,nx,steps);
    
%   Set p_T and P_T as gradient and hessian of the cost
    [~,dlT_x, dlT_xx] = term_cost_DDP_ale(xx(:,steps,kk),xx_ref(:,steps),params);
    pp(:,steps) = dlT_x;
    PP(:,:,steps) = dlT_xx;
    
%     Backward Iteration
    for t=(steps-1):-1:1
        
%       call of stage cost function to get gradients and hessians
        [~, dl_x, dl_xx, dl_u, dl_uu, dl_ux] = stage_cost_DDP_ale(xx(:,t,kk),uu(:,t,kk),xx_ref(:,t),uu_ref(:,t),params);
%         call of dynamics function to get linearization of contracted
%         tensors
        [~, df_x, df_u, ~, ~, ~, pfxx, pfuu, pfux] = dynamics_ale(xx(:,t,kk),uu(:,t,kk), params, pp(:,t+1));
        
%         Computation of Gain and Feedforward term

        KK(:,:,t) = -(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)\(dl_ux + df_u*PP(:,:,t+1)*df_x' + pfux);
        sigma(:,t) = -(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)\(dl_u + df_u*pp(:,t+1));
        
        PP(:,:,t) = (dl_xx + df_x*PP(:,:,t+1)*df_x' + pfxx) - KK(:,:,t)'*(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)*KK(:,:,t);
        pp(:,t) = (dl_x + df_x*pp(:,t+1)) - KK(:,:,t)'*(dl_uu + df_u*PP(:,:,t+1)*df_u' + pfuu)*sigma(:,t);
        
%         Update of descent direction
        descent(kk) = descent(kk) - sigma(:,t)'*sigma(:,t);
    end
    
% % % % % % % %         ARMIJO STEPSIZE 
        if armijo
            cc = 0.5;
            beta = 0.5;
            gamma = 1;
            
            cost_arm = [];
            
            xx_arm = zeros(nx,steps);
            uu_arm = zeros(nu,steps);
            
            xx_arm(:,1) = x_0;
            JJ_arm = 0;
            
            for t=1:(steps-1)
                
                uu_arm(:,t) = uu(:,t,kk) + gamma(end)*sigma(:,t) + KK(:,:,t)*(xx_arm(:,t) - xx(:,t,kk));
                xx_arm(:,t+1) = dynamics_ale(xx_arm(:,t),uu_arm(:,t),params,pp(:,t+1));
                
                stage_cost_arm = stage_cost_DDP_ale(xx_arm(:,t),uu_arm(:,t), xx_ref(:,t), uu_ref(:,t),params);
                
                JJ_arm = JJ_arm + stage_cost_arm;
            end
            
            term_cost_arm = term_cost_DDP_ale(xx_arm(:,steps), xx_ref(:,steps), params);
            JJ_arm = JJ_arm + term_cost_arm;
            
            cost_arm = [cost_arm; JJ_arm];
            
%             Starting point of the Armijo Loop - keep going until the
%             condition is satisfied 

            while cost_arm(end) > J(kk) + cc*gamma(end)*descent(kk)
                gamma = [gamma; gamma(end)*beta];
                xx_arm(:,1) = x_0;
                JJ_arm = 0;
                
                for t=1:(steps-1)
                    
                    uu_arm(:,t) = uu(:,t,kk) + gamma(end)*sigma(:,t) + KK(:,:,t)*(xx_arm(:,t) - xx(:,t,kk));
                    xx_arm(:,t+1) = dynamics_ale(xx_arm(:,t),uu_arm(:,t), params, pp(:,t+1));
                    
                    stage_cost_arm = stage_cost_DDP_ale(xx_arm(:,t), uu_arm(:,t), xx_ref(:,t), uu_ref(:,t), params);
                    JJ_arm = JJ_arm + stage_cost_arm;
                end
                
                term_cost_arm = term_cost_DDP_ale(xx_arm(:,steps), xx_ref(:,steps), params);
                JJ_arm = JJ_arm + term_cost_arm;
            end
            
            gamma_steps = gamma;
            gamma = gamma(end);
        end
        
%         ARMIJO PLOTS
        if armijo
            points = 15;
            
            gamma_stepsize = linspace(0,1,points);
            
            cost_temp = zeros(points,1);
            xx_temp = zeros(nx,steps);
            uu_temp = zeros(nu,steps);
            
            for i = 1:length(gamma_stepsize)
                gamma_i = gamma_stepsize(i);
                xx_temp(:,1) = x_0;
                
                for t = 1:(steps-1)
                    uu_temp(:,t) = uu(:,t,kk) + gamma_i*sigma(:,t) + KK(:,:,t)*(xx_temp(:,t) - xx(:,t));
                    xx_temp(:,t+1) = dynamics_ale(xx_temp(:,t),uu_temp(:,t), params, pp(:,t+1));
                    
                    stage_cost_temp = stage_cost_DDP_ale(xx_temp(:,t), uu_temp(:,t), xx_ref(:,t), uu_ref(:,t), params);
                    cost_temp(i) = cost_temp(i) + stage_cost_temp;
                end
                term_cost_temp = term_cost_DDP_ale(xx_temp(:,steps), xx_ref(:,steps), params);
                cost_temp(i) = cost_temp(i) + term_cost_temp;
            end
            
            figure("Name","Armijo")
            
            plot(gamma_stepsize, min(cost_temp, 5*J(kk)),"LineWidth",2,"DisplayName","$J(x^k - \gamma\nabla J^k$");
            hold on, grid on, zoom on
            
            plot(gamma_stepsize, J(kk) + gamma_stepsize*descent(kk),"r--","LineWidth",2,"DisplayName","$J(x^k) + \gamma descent$");
            
            plot(gamma_stepsize, J(kk) + cc*gamma_stepsize*descent(kk),"g--","LineWidth",2,"DisplayName","$J(x^k) + c \gamma descent$");
            
            plot(gamma, cost_arm,"*","DisplayName","Armijo step");
            
            tit = sprintf("Iter: %d",kk);
            title(tit);
            
            legend("Interpreter","latex","FontSize",12,"location","southeast");
            
            hold off
            drawnow
            
        end
        
        
%     Forward iteration to update the Trajectories for the next k
    xx(:,1,kk+1) = x_0;
    for t=1:steps-1
        
%         Computation of the control input
        uu(:,t,kk+1) = uu(:,t,kk) + gamma*sigma(:,t) + gamma*KK(:,:,t)*(xx(:,t,kk+1) - xx(:,t,kk));
%         Computation of x_plus
        xx(:,t+1,kk+1) = dynamics_ale(xx(:,t,kk+1),uu(:,t,kk+1),params,pp(:,t+1));
        
        stage_cost_temp = stage_cost_DDP_ale(xx(:,t,kk+1),uu(:,t,kk+1),xx_ref(:,t),uu_ref(:,t),params);
        
%         Update of the cost by the stage cost
        J(kk+1) = J(kk+1) + stage_cost_temp;
        
    end
    
%     Update of the cost by the final cost
    term_cost_temp = term_cost_DDP_ale(xx(:,steps,kk+1),xx_ref(:,steps),params);
    J(kk+1) = J(kk+1) + term_cost_temp;
    
    fprintf("Iteration %d \n", kk)
    fprintf("Descent: %d, Cost: %d \n", descent(kk), J(kk))
    
    if abs(descent(kk)) < tol
        fprintf("Tolerance reached \n")
        break
    end
end


uu(:,steps,kk) = uu(:,steps-1,kk);

figure
hold on
title("Skidpad")
plot(xx(1,:,kk),xx(2,:,kk),"r","LineWidth",2)
skidpad_track(9.125,3)
xlabel("X(m)")
ylabel("Y(m)")
hold off
zoom on, grid on

figure
hold on
title("Cost Function")
plot(1:kk,log(J(1:kk)),"LineWidth",2)
xlabel("Iteration")
ylabel("$\log(J)$","interpreter","latex")
hold off

figure
subplot(2,1,1)
hold on
stairs(1:steps, xx(1,:,kk),"LineWidth",2);
stairs(1:steps, xx_ref(1,:), "--","LineWidth",2);
ylabel("$X(m)$","interpreter","latex",'FontSize',16)
xlabel("steps",'FontSize',12)
title("X(m)")
legend("$x_{1,t}$","$x_{1,ref}$","interpreter","latex")
grid on, zoom on
hold off

subplot(2,1,2)
hold on
stairs(1:steps, xx(2,:,kk),"LineWidth",2);
stairs(1:steps, xx_ref(2,:), "--","LineWidth",2);
ylabel("$Y(m)$","interpreter","latex",'FontSize',12)
xlabel("steps",'FontSize',12)
title("Y(m)")
legend("$x_{2,t}$","$x_{2,ref}$","interpreter","latex")
grid on, zoom on
hold off




if plots
%     Yaw Rate and Yaw Angle plots
    figure(1030)
    subplot(2,1,1)
    hold on
    stairs(1:steps, xx(3,:,kk),"LineWidth",2);
    stairs(1:steps, xx_ref(3,:), "--","LineWidth",2);
    ylabel("$\psi$","interpreter","latex",'FontSize',12)
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



% initialization for the LQR 
final_skidpad.xx = xx;
final_skidpad.uu = uu;
save("final_skidpad.mat")
