#Discrete Parameters
float32 paso_v
float32 paso_w
#Maximum Velocities Space
float32 Vs_x_max
float32 Vs_x_min
float32 Vs_z_max
float32 Vs_z_min
float32 Vs_w_max
float32 Vs_w_min
#Dynamic Windows Space
float32 Vd_x_max
float32 Vd_x_min
float32 Vd_z_max
float32 Vd_z_min
float32 Vd_w_max
float32 Vd_w_min
#Current velocity
geometry_msgs/Twist Vc 
#Evaluation Terms
uint16 filas_tot
uint16 filas_eval
float32[] headingYawTerm
float32[] headingZTerm
float32[] velocityTerm
float32[] minDistTerm
float32[] vx
float32[] vz 
float32[] w
float32[] G
#Selected Velocity 
float32 G_selected
float32 vx_selected
float32 vz_selected
float32 w_selected

