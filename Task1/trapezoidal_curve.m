% Function which computes the trapezoidal reference curve we use to define
% the reference of the lane change maneuver using the PI controller. 
% Input: x 
% Output: y, dy  

function [y, dy] = trapezoidal_curve(x)

x1=75; % where the trapezoidal start to increase (on the x-axis)
x3=115; % where it will arrive at the maximum value (on the x-axis)
y_ref=1; % reference (maximum) value on the y-axis

a = (x3-x1)/2;
x2 = x1 + a;
h = y_ref/a;
m = h/a;
C1 = m/2*x1^2; % C1 and C2 values calculated to improve continuity of y(x)
C2 = y_ref - x3*(h+m*x2-0.5*m*x3);

% computation of y(x) and dy(x)/dx in the significant intervals on the
% x-axis
    if x <= x1 
        y = 0;
        dy = 0;
    elseif (x1 < x) && (x <= x2)
        y = m*(0.5*x^2 - x1*x)+ C1;
        dy = m*(x-x1);
    elseif (x2 < x) && (x <= x3)
        y = h*x + m*(x*x2 - 0.5*x^2) + C2;
        dy = h - m*(x-x2);
    elseif x > x3
        y = y_ref;
        dy = 0;
    end
    
end

