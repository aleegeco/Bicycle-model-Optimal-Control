% Function which calculates the final cost. 
% Function inputs: x_t, x_ref, params (= struct with plant parameters)
% Function returns: lT, dlT_x, dlT_xx
% dimensions: lT = (1,1) dlT_x = (nx,1), dlT_xx = (nx,nx)

function[lT,dlT_x,dlT_xx]=term_cost_DDP_ale(x_t,x_ref,params)

QT = params.cost.QT;

lT = (x_t-x_ref)'*QT*(x_t-x_ref);

dlT_x = 2*QT*(x_t-x_ref);
dlT_xx = 2*QT;

end