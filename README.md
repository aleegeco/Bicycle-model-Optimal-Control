# Bicycle Model Optimal Control - Lane change manuevering and Skidpad

This project was delivered as part of  the **Optimal Control** course held by professor Notarstefano at University of Bologna.

The related task description can be read [here](http://github.com/aleegeco/Bicycle-model-Optimal-Control/blob/main/main_vehicle.pdf)

The report where we discuss in detail all the results can be read [here](http://github.com/aleegeco/Bicycle-model-Optimal-Control/blob/main/Report_Optimal_Control_Bicycle_Model_CecconiBugoFrangiamone.pdf)

## Model Definition ##

<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\begin{equation}\begin{array}{cc}&space;&space;&space;&space;&space;\Dot{x}&space;=&space;v_x\cos{\psi}-v_y\sin{\psi}&space;\\&space;&space;&space;&space;&space;\Dot{y}&space;=&space;v_x\sin{\psi}&plus;v_y\cos{\psi}&space;\\&space;&space;&space;&space;&space;m(\Dot{v}_x-\Dot{\psi}v_y)&space;=&space;F_x\cos{\delta}-F_{y,f}\sin{\delta}&plus;F_{y,r}&space;\\&space;&space;&space;&space;&space;m(\Dot{v}_y&plus;\Dot{\psi}v_x)&space;=&space;F_x\sin{\delta}&plus;F_{y,f}\cos{\delta}&plus;&space;F_{y,r}&space;\\&space;&space;&space;&space;&space;I_z\Ddot{\psi}&space;=&space;(F_x\sin{\delta}&space;&plus;&space;F_{y,f}\cos{\delta})a&space;-&space;F_{y,r}b\end{array}\end{array}" title="\begin{equation}\begin{array}{cc} \Dot{x} = v_x\cos{\psi}-v_y\sin{\psi} \\ \Dot{y} = v_x\sin{\psi}+v_y\cos{\psi} \\ m(\Dot{v}_x-\Dot{\psi}v_y) = F_x\cos{\delta}-F_{y,f}\sin{\delta}+F_{y,r} \\ m(\Dot{v}_y+\Dot{\psi}v_x) = F_x\sin{\delta}+F_{y,f}\cos{\delta}+ F_{y,r} \\ I_z\Ddot{\psi} = (F_x\sin{\delta} + F_{y,f}\cos{\delta})a - F_{y,r}b\end{array}\end{array}" />
