import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.colors as mcolors
from hector_moveit_actions.msg import DynamicWindowMsg
from geometry_msgs.msg import Twist

import numpy as np


class NumericInfo:
    def __init__(self):
        self.recieved_first_msg = False
        self.iter = 0

    def show_info(self):
        #####Plot voxels
        velocities = np.zeros([self.last_msg.filas_eval,3])
        
        #Selected Speed
        idx_w_selected = round((self.last_msg.w_selected - self.last_msg.Vs_w_min) / self.last_msg.paso_w) 
        idx_vx_selected = round((self.last_msg.vx_selected - self.last_msg.Vs_x_min) / self.last_msg.paso_v) 
        idx_vz_selected = round((self.last_msg.vz_selected - self.last_msg.Vs_z_min) / self.last_msg.paso_v) 
        #voxels_np[idx_w_selected, idx_vx_selected, idx_vz_selected] = True
        
        #Fill the np_arrays
        self.iter += 1
        print("************************************* \n")
        print("Velocities iter: {} \n".format(self.iter))
        for i in range(self.last_msg.filas_eval):
            idx_w = round((self.last_msg.w[i] - self.last_msg.Vs_w_min) / self.last_msg.paso_w) 
            idx_vx = round((self.last_msg.vx[i] - self.last_msg.Vs_x_min) / self.last_msg.paso_v) 
            idx_vz = round((self.last_msg.vz[i] - self.last_msg.Vs_z_min) / self.last_msg.paso_v)
            #voxels_np[idx_w, idx_vx, idx_vz] = True

            velocities[i] = [self.last_msg.w[i], self.last_msg.vx[i], self.last_msg.vz[i]]
            print("Vx: {}, Wz: {}, Vz: {} \n".format(self.last_msg.vx[i], self.last_msg.w[i], self.last_msg.vz[i]))
            print("\t G: {} \n".format(self.last_msg.G[i]))
            print("\t headingYawTerm: {} \n".format(self.last_msg.headingYawTerm[i]))
            print("\t headingZTerm: {} \n".format(self.last_msg.headingZTerm[i]))
            print("\t velocityTerm: {} \n".format(self.last_msg.velocityTerm[i]))
            print("\t minDistTerm: {} \n".format(self.last_msg.minDistTerm[i]))
        print('\033[42m' + 'Selected Velocity: \n')
        print('Vx: {}, Wz: {}, Vz: {} \n'.format(self.last_msg.vx_selected, self.last_msg.w_selected, self.last_msg.vz_selected))
        print("\t G: {} \n".format(self.last_msg.G_selected) + '\033[0m')
        print("************************************* \n")

    def infoCallback(self, msg):
        recieved_first_msg = True
        self.last_msg = msg


if __name__ == '__main__':
    try:
        rospy.init_node("Dwa_Numeric_Info")
        numericInfo = NumericInfo()
        info_subscriber = rospy.Subscriber("/DWA_visual_msg", DynamicWindowMsg, numericInfo.infoCallback)
        while True:
            numericInfo.last_msg = rospy.wait_for_message("/DWA_visual_msg", DynamicWindowMsg, timeout=None)
            numericInfo.show_info()
    except rospy.ROSInterruptException:
        pass
