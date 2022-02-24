% % Function which computes the discretized dynamics of our dynamic bicycle model
% Function Inputs: x_t, u_t, params, pp ( = costate vector)
% Returns: x_t+1, df_x, df_u, df_xx, df_uu, df_ux, pfxx, pfuu, pfux

function [x_plus,df_x,df_u,df_xx,df_uu,df_ux,pfxx,pfuu,pfux]= dynamics_ale(xx,uu,params,pp)

m = params.model.m; % [kg]
Iz = params.model.Iz; % [kgm^2]
a = params.model.a; % [m]
b = params.model.b; % [m]
mu = params.model.mu; % [dimensionless]
g = params.model.g; % [m/s^2]
dt = params.model.dt; % sampling time

if length(xx) == 6 && length(uu) == 2
    nx = length(xx); % state dimension
    nu = length(uu); % input dimension
else
    fprintf("Invalid vectors dimensions \n")
    fprintf("The state dimension must be %3.f \nThe input dimension must be %3.f \n",6,2)
    return 
end
    

x = xx; % at time t the value of x are set equal to the input ones 
u = uu; % same for the input

% definition of slip angles and forces

beta_f = u(1) - (x(5) + a*x(6))/(x(4)); % front slip angle
beta_r =  -(x(5) - b*x(6))/(x(4)); % rear slip angle

Fz_f = (m*g*b)/(a+b); % Vertical force at front 
Fz_r = (m*g*a)/(a+b); % Vertical force at rear

Fy_f = mu*Fz_f*beta_f; %lateral tire force front wheel
Fy_r = mu*Fz_r*beta_r; %lateral tire force rear wheel

x_plus = zeros(nx,1); %initialization of x_(t+1) vector

% discretized model 

x_plus(1) = x(1) + dt*( x(4)*cos(x(3)) - x(5)*sin(x(3)) );
x_plus(2) = x(2) + dt*( x(4)*sin(x(3)) + x(5)*cos(x(3)) );
x_plus(3) = x(3) + dt*( x(6) );
x_plus(4) = x(4) + dt*(1/m)*( u(2)*cos(u(1)) - Fy_f*sin(u(1)) + m*x(6)*x(5) );
x_plus(5) = x(5) + dt*(1/m)*( u(2)*sin(u(1)) + Fy_f*cos(u(1)) + Fy_r - m*x(6)*x(4) );
x_plus(6) = x(6) + dt*(1/Iz)*( (u(2)*sin(u(1)) + Fy_f*cos(u(1)) )*a - Fy_r*b );

% linearization of the model - compute the Jacobians/Gradients
% we should first compute the linearization of the Lateral Forces because
% they are functions of the states because of the slip angles

% derivatives of Fy_f w.r.t. x4,x5,x6,u1; others are 0

dFy_f_x4 = Fz_f*(x(5)+a*x(6))/(x(4)^2);
dFy_f_x5 = (-Fz_f)/(x(4));
dFy_f_x6 = (-Fz_f*a)/(x(4));
dFy_f_u1 = Fz_f;

dFy_f_x4_x4 = -2*Fz_f*(x(5)+a*x(6))/(x(4)^3); %  second order derivative of Fy_f w.r.t. x4
dFy_f_x4_x5 = (Fz_f)/(x(4)^2);
dFy_f_x4_x6 = (a*Fz_f)/(x(4)^2);

dFy_f_x5_x4 = Fz_f/(x(4)^2);

dFy_f_x6_x4 = (a*Fz_f)/(x(4)^2);

% derivatives of Fy_r w.r.t. x4,x5,x6; others are 0

dFy_r_x4 = Fz_r*(x(5)-b*x(6))/(x(4)^2);
dFy_r_x5 = (-Fz_r)/(x(4));
dFy_r_x6 = (b*Fz_r)/(x(4));

dFy_r_x4_x4 = -2*Fz_r*(x(5)-b*x(6))/(x(4)^3); % second order derivative of Fy_r w.r.t x4
dFy_r_x4_x5 = Fz_r/(x(4)^2);
dFy_r_x4_x6 = -b*Fz_r/(x(4)^2);

dFy_r_x5_x4 = Fz_r/(x(4)^2);

dFy_r_x6_x4 = -b*Fz_r/(x(4)^2);


% f1 = x_1 + delta*(x_4 cosx_3 - x_5 sin x3)

df1_x3 = -dt*(x(4)*sin(x(3)) + x(5)*cos(x(3)));
df1_x4 = dt*cos(x(3));
df1_x5 = -dt*sin(x(3));

df1_x3_x3 = -dt*(x(4)*cos(x(3)) - x(5)*sin(x(3))); 
df1_x3_x4 = -dt*(sin(x(3)));
df1_x3_x5 = -dt*(cos(x(3)));

df1_x4_x3 = -dt*sin(x(3));

df1_x5_x3 = -dt*cos(x(3));

%  f2 = x_2 + delta*(x4 sin x3 + x5 cos x3)
df2_x3 = dt*(x(4)*cos(x(3)) - x(5)*sin(x(3)));
df2_x4 = dt*sin(x(3));
df2_x5 = dt*cos(x(3));

df2_x3_x3 = -dt*(x(4)*sin(x(3)) + x(5)*cos(x(3)));
df2_x3_x4 = dt*cos(x(3));
df2_x3_x5 = -dt*sin(x(3));

df2_x4_x3 = dt*cos(x(3));

df2_x5_x3 = -dt*sin(x(3));


% f4 = x4 + delta/m(u2 cos u1 - Fyf sin u1 + mx6x5 )
df4_x4 = 1 - (dt/m)*(dFy_f_x4*sin(u(1)));
df4_x5 = -(dt/m)*(dFy_f_x5*sin(u(1)) - m*x(6));
df4_x6 = -(dt/m)*(dFy_f_x6*sin(u(1)) - m*x(5));

df4_x4_x4 = -(dt/m)*(dFy_f_x4_x4*sin(u(1)));
df4_x4_x5 = -(dt/m)*(dFy_f_x4_x5*sin(u(1)));
df4_x4_x6 = -(dt/m)*(dFy_f_x4_x6*sin(u(1)));

df4_x5_x4 = -(dt/m)*(dFy_f_x5_x4*sin(u(1)));

df4_x6_x4 = -(dt/m)*(dFy_f_x6_x4*sin(u(1)));

df4_u1 = -(dt/m)*(u(2)*sin(u(1)) + dFy_f_u1*sin(u(1)) + Fy_f*cos(u(1)));
df4_u2 = (dt/m)*(cos(u(1)));

df4_u1_u1 = -(dt/m)*(u(2)*cos(u(1)) + dFy_f_u1*cos(u(1)) + dFy_f_u1*cos(u(1)) - Fy_f*sin(u(1)));
df4_u1_u2 = -(dt/m)*(sin(u(1)));

df4_u2_u1 = -(dt/m)*(sin(u(1)));

df4_u1_x4 = -(dt/m)*(dFy_f_x4*cos(u(1)));
df4_u1_x5 = -(dt/m)*(dFy_f_x5*cos(u(1)));
df4_u1_x6 = -(dt/m)*(dFy_f_x6*cos(u(1)));


% f5 = x5 + delta/m(u2 sin u1 + Fyf cos u1 + Fyr - mx6x4 )
df5_x4 = (dt/m)*(dFy_f_x4*cos(u(1)) + dFy_r_x4 - m*x(6));
df5_x5 = 1 + (dt/m)*(dFy_f_x5*cos(u(1)) + dFy_r_x5);
df5_x6 = (dt/m)*(dFy_f_x6*cos(u(1)) + dFy_r_x6 - m*x(4));

df5_x4_x4 = (dt/m)*(dFy_f_x4_x4*cos(u(1)) + dFy_r_x4_x4);
df5_x4_x5 = (dt/m)*(dFy_f_x4_x5*cos(u(1)) + dFy_r_x4_x5);
df5_x4_x6 = (dt/m)*(dFy_f_x4_x6*cos(u(1)) + dFy_r_x4_x6 - m);

df5_x5_x4 = (dt/m)*(dFy_f_x5_x4*cos(u(1)) + dFy_r_x5_x4);

df5_x6_x4 = (dt/m)*(dFy_f_x6_x4*cos(u(1)) + dFy_r_x6_x4 - m);

df5_u1 = (dt/m)*(u(2)*cos(u(1)) + dFy_f_u1*cos(u(1)) - Fy_f*sin(u(1)));
df5_u2 = (dt/m)*(sin(u(1)));

df5_u1_u1 = - (dt/m)*(u(2)*sin(u(1)) + dFy_f_u1*sin(u(1)) + dFy_f_u1*sin(u(1)) + Fy_f*cos(u(1)));
df5_u1_u2 = (dt/m)*(cos(u(1)));

df5_u2_u1 = (dt/m)*(cos(u(1)));

df5_u1_x4 = -(dt/m)*(dFy_f_x4*sin(u(1)));
df5_u1_x5 = -(dt/m)*(dFy_f_x5*sin(u(1)));
df5_u1_x6 = -(dt/m)*(dFy_f_x6*sin(u(1)));

% f6 = x6 + delta/Iz(u2sin u1 + Fyf cos u1)a - delta/Iz*Fyrb
df6_x4 = (dt/Iz)*(dFy_f_x4*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x4;
df6_x5 = (dt/Iz)*(dFy_f_x5*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x5;
df6_x6 = 1 + (dt/Iz)*(dFy_f_x6*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x6;

df6_x4_x4 = (dt/Iz)*(dFy_f_x4_x4*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x4_x4;
df6_x4_x5 = (dt/Iz)*(dFy_f_x4_x5*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x4_x5;
df6_x4_x6 = (dt/Iz)*(dFy_f_x4_x6*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x4_x6;

df6_x5_x4 = (dt/Iz)*(dFy_f_x5_x4*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x5_x4;

df6_x6_x4 = (dt/Iz)*(dFy_f_x6_x4*cos(u(1)))*a - (dt/Iz)*b*dFy_r_x6_x4;

df6_u1 = (dt/Iz)*a*(u(2)*cos(u(1)) + dFy_f_u1*cos(u(1)) - Fy_f*sin(u(1)));
df6_u2 = (dt/Iz)*a*(sin(u(1)));

df6_u1_u1 = -(dt/Iz)*a*(u(2)*sin(u(1)) + dFy_f_u1*sin(u(1)) + dFy_f_u1*sin(u(1)) + Fy_f*cos(u(1)));
df6_u1_u2 = (dt/Iz)*a*(cos(u(1)));

df6_u2_u1 = (dt/Iz)*a*(cos(u(1)));

df6_u1_x4 = -(dt/Iz)*a*(dFy_f_x4*sin(u(1)));
df6_u1_x5 = -(dt/Iz)*a*(dFy_f_x5*sin(u(1)));
df6_u1_x6 = -(dt/Iz)*a*(dFy_f_x6*sin(u(1)));

% gradient of f w.r.t. x = linearized matrix A transpose

df_x = [1, 0, df1_x3, df1_x4, df1_x5, 0;
    0, 1, df2_x3, df2_x4, df2_x5, 0;
    0, 0, 1, 0, 0, dt;
    0, 0, 0, df4_x4, df4_x5, df4_x6;
    0, 0, 0, df5_x4, df5_x5, df5_x6;
    0, 0, 0, df6_x4, df6_x5, df6_x6]';

% gradient of f w.r.t. u = linearized matrix B transpose

df_u = [0, 0;
    0, 0;
    0, 0;
    df4_u1, df4_u2;
    df5_u1, df5_u2;
    df6_u1, df6_u2]';

% Initialization for the tensor product
pfxx = zeros(nx,nx);
pfuu = zeros(nu,nu);
pfux = zeros(nu,nx);

% cell array (matrix of matrices) of the Hessian of f(x,u) w.r.t. x

df_xx = {[0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, df1_x3_x3, df1_x3_x4, df1_x3_x5, 0;
    0, 0, df1_x4_x3, 0, 0, 0;
    0, 0, df1_x5_x3, 0, 0, 0;
    0, 0, 0, 0, 0, 0]
    [0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, df2_x3_x3, df2_x3_x4, df2_x3_x5, 0;
    0, 0, df2_x4_x3, 0, 0, 0;
    0, 0, df2_x5_x3, 0, 0, 0;
    0, 0, 0, 0, 0, 0]
    zeros(6,6)
    [0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, df4_x4_x4, df4_x4_x5, df4_x4_x6;
    0, 0, 0, df4_x5_x4, 0, dt;
    0, 0, 0, df4_x6_x4, dt, 0]
    [0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, df5_x4_x4, df5_x4_x5, df5_x4_x6;
    0, 0, 0, df5_x5_x4, 0, 0;
    0, 0, 0, df5_x6_x4, 0, 0]
    [0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0;
    0, 0, 0,df6_x4_x4, df6_x4_x5, df6_x4_x6;
    0, 0, 0, df6_x5_x4, 0, 0;
    0, 0, 0, df6_x6_x4, 0, 0]
    };

% cell array (matrix of Matrices) of the Hessian of f(x,u) w.r.t. u
df_uu = {zeros(2,2)
    zeros(2,2)
    zeros(2,2)
    [df4_u1_u1, df4_u1_u2;
    df4_u2_u1, 0]
    [df5_u1_u1, df5_u1_u2;
    df5_u2_u1, 0]
    [df6_u1_u1, df6_u1_u2;
    df6_u2_u1, 0]
    };

df_ux = {zeros(2,6)
    zeros(2,6)
    zeros(2,6)
    [0, 0, 0, df4_u1_x4, df4_u1_x5, df4_u1_x6;
    0, 0, 0, 0, 0, 0]
    [0, 0, 0, df5_u1_x4, df5_u1_x5, df5_u1_x6;
    0, 0, 0, 0, 0, 0]
    [0, 0, 0, df6_u1_x4, df6_u1_x5, df6_u1_x6;
    0, 0, 0, 0, 0, 0]
    };



for ii=1:nx
    pfxx = pfxx + (df_xx{ii}*pp(ii));
    
    pfuu = pfuu + (df_uu{ii}*pp(ii));
    
    pfux = pfux + (df_ux{ii}*pp(ii));
end

end

