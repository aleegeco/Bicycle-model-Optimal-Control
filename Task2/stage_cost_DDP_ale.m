% Function which computes the stage cost from 0 up to T-1
% function inputs: x_t, u_t, x_ref, u_ref, params (= structure with plant
% parameters)
% function return = lt, dlx, dlxx, dlu, dluu, dlux
% (stage cost and its gradients and Hessians)
% Dimensions: lt = (1,1) dl_x = (nx,1), dl_xx = (nx,nx), dl_u = (nu,1),
% dl_uu = (nu,nu); dl_ux = (nu,nx)

function [lt,dl_x,dl_xx,dl_u,dl_uu,dl_ux] = stage_cost_DDP_ale(x_t,u_t,x_ref,u_ref,params)

% state and input dimensions
nx = length(x_t);
nu = length(u_t);

% weigth matrices for states and inputs
Q = params.cost.Q;
R = params.cost.R;

% stage cost definition
lt = (x_t - x_ref)'*Q*(x_t - x_ref) + (u_t - u_ref)'*R*(u_t - u_ref);

% gradients of the stage cost w.r.t. x and u
dl_x = 2*Q*x_t - 2*Q*x_ref;
dl_u = 2*R*u_t - 2*R*u_ref;

% Hessians
dl_xx = 2*Q;
dl_uu = 2*R; 
dl_ux = zeros(nu,nx);

end