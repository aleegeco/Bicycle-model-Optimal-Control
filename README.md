# Bicycle Model Optimal Control - Lane change manuevering and Skidpad

This project was delivered as part of  the **Optimal Control** course held by professor Notarstefano at University of Bologna.

The related task description can be read [here](http://github.com/aleegeco/Bicycle-model-Optimal-Control/blob/main/main_vehicle.pdf)

The report where we discuss in detail all the results can be read [here](http://github.com/aleegeco/Bicycle-model-Optimal-Control/blob/main/Report_Optimal_Control_Bicycle_Model_CecconiBugoFrangiamone.pdf)

The dynamic bicycle model used for this project is ill-defined for low velocities, because of the way in which the slip angles are defined (*see the task description*). In particular the longitudinal velocity can never be zero.
As a consequence, all the tasks are initiliazed assuming that our car is moving. It is not a strong assumption even in the case of the skidpad if we are not considering particular racing cases. 

## Lane Change Maneuver ##
In this task, we defined a linear transition between two supposed equilibria simulating an ideal change of lane in an highway. In order to do that the implemented algorithm is the Differential Dynamic Programming.
