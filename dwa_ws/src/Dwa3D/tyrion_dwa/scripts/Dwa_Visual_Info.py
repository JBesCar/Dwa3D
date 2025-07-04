import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.colors as mcolors
from hector_moveit_actions.msg import DynamicWindowMsg
from geometry_msgs.msg import Twist

import numpy as np


class Visualizer:
    def __init__(self):
        self.recieved_first_msg = False
        #Create a 3D figure
        self.fig= plt.figure()
        self.ax  = self.fig.add_subplot(projection="3d")
        #Create lines
        self.lines = self.ax.plot([], [], [], 'ro')
    def plot_init(self):
        #Set limits to plot
        self.ax.set_ylim(self.last_msg.Vs_x_min - self.last_msg.paso_v, self.last_msg.Vs_x_max +  self.last_msg.paso_v)
        self.ax.set_xlim(self.last_msg.Vs_w_max +  self.last_msg.paso_w, self.last_msg.Vs_w_min -  self.last_msg.paso_w)
        self.ax.set_zlim(self.last_msg.Vs_z_min -  self.last_msg.paso_v, self.last_msg.Vs_z_max +  self.last_msg.paso_v)
        #Set legend to axes
        self.ax.set_xlabel("w_z (rad/s)")
        self.ax.set_ylabel("v_x (m/s)")
        self.ax.set_zlabel("v_z (m/s)")

        #Create the np arrays to store the info
        self.vx_np = np.arange(self.last_msg.Vs_x_min, self.last_msg.Vs_x_max, self.last_msg.paso_v)
        self.w_np = np.arange(self.last_msg.Vs_w_min, self.last_msg.Vs_w_max, self.last_msg.paso_w)
        self.vz_np = np.arange(self.last_msg.Vs_z_min, self.last_msg.Vs_z_max, self.last_msg.paso_v)
        self.voxels_np = np.zeros((self.w_np.size, self.vx_np.size, self.vz_np.size), dtype=bool)

        #Set ticks
        # self.ax.set_xticks(self.w_np)
        # self.ax.set_yticks(self.vx_np)
        # self.ax.set_zticks(self.vz_np)
        return self.lines

    def update_plot(self,frames):
        #Scatter data
        #self.ax.scatter(self.last_msg.w_selected, self.last_msg.vx_selected, self.last_msg.vz_selected, color='g')
        #self.ax.scatter(self.last_msg.w, self.last_msg.vx, self.last_msg.vz, color ='r', alpha = 0.5, marker = 'x')
        #self.ax.scatter(self.last_msg.Vc.angular.z, self.last_msg.Vc.linear.x, self.last_msg.Vc.linear.z, color='k')
        
        #####Plot voxels
        voxels_np = np.zeros((self.w_np.size, self.vx_np.size, self.vz_np.size), dtype=bool)
        sizes = np.tile([self.last_msg.paso_w, self.last_msg.paso_v, self.last_msg.paso_v], (self.last_msg.filas_eval+1,1))
        positions = np.zeros([self.last_msg.filas_eval,3])
        colors = np.zeros([self.last_msg.filas_eval,4])
        
        #Selected Speed
        idx_w_selected = round((self.last_msg.w_selected - self.last_msg.Vs_w_min) / self.last_msg.paso_w) 
        idx_vx_selected = round((self.last_msg.vx_selected - self.last_msg.Vs_x_min) / self.last_msg.paso_v) 
        idx_vz_selected = round((self.last_msg.vz_selected - self.last_msg.Vs_z_min) / self.last_msg.paso_v) 
        #voxels_np[idx_w_selected, idx_vx_selected, idx_vz_selected] = True
        
        #Fill the np_arrays
        for i in range(self.last_msg.filas_eval):
            idx_w = round((self.last_msg.w[i] - self.last_msg.Vs_w_min) / self.last_msg.paso_w) 
            idx_vx = round((self.last_msg.vx[i] - self.last_msg.Vs_x_min) / self.last_msg.paso_v) 
            idx_vz = round((self.last_msg.vz[i] - self.last_msg.Vs_z_min) / self.last_msg.paso_v)
            #voxels_np[idx_w, idx_vx, idx_vz] = True

            if(idx_w == idx_w_selected and idx_vx == idx_vx_selected and idx_vz == idx_vz_selected):
                colors[i] = [0.03, 0.945, 0.659, 0.75] #'#C8EAD399'
                positions[i] = [self.last_msg.w_selected, self.last_msg.vx_selected, self.last_msg.vz_selected]
            else:
                #colors[i] = [0.659, 0.06, 0.06, 0.05] #'#e4676759'
                colors[i] = [1-self.last_msg.minDistTerm[i], self.last_msg.minDistTerm[i], 0, 0.2]
                positions[i] = [self.last_msg.w[i], self.last_msg.vx[i], self.last_msg.vz[i]]
        
        colors = np.vstack([colors,[0.03, 0.945, 0.659, 0.75]]) #'#C8EAD399'
        positions_np = np.vstack([positions,[self.last_msg.w_selected, self.last_msg.vx_selected, self.last_msg.vz_selected]])
        positions_np -= sizes/2
        #Add current speed
        positions_np = np.vstack([positions_np,
                                [self.last_msg.Vc.angular.z - self.last_msg.paso_w/2, 
                                self.last_msg.Vc.linear.x - self.last_msg.paso_v/2, 
                                self.last_msg.Vc.linear.z - self.last_msg.paso_v/2]])
        colors = np.vstack([colors, [0, 0, 0, 1]])
        sizes = np.vstack([sizes, [self.last_msg.paso_w, self.last_msg.paso_v, self.last_msg.paso_v]])
        #Add dynamic limits
        ldy = self.last_msg.Vd_x_max - self.last_msg.Vd_x_min + self.last_msg.paso_v
        ldx = self.last_msg.Vd_w_max - self.last_msg.Vd_w_min + self.last_msg.paso_w
        ldz = self.last_msg.Vd_z_max - self.last_msg.Vd_z_min + self.last_msg.paso_v

        lsy = self.last_msg.Vs_x_max - self.last_msg.Vs_x_min + self.last_msg.paso_v
        lsx = self.last_msg.Vs_w_max - self.last_msg.Vs_w_min + self.last_msg.paso_w
        lsz = self.last_msg.Vs_z_max - self.last_msg.Vs_z_min + self.last_msg.paso_v

        positions_np = np.vstack([positions_np, 
                                [self.last_msg.Vd_w_min - self.last_msg.paso_w/2, 
                                self.last_msg.Vd_x_min - self.last_msg.paso_v/2, 
                                self.last_msg.Vd_z_min - self.last_msg.paso_v/2],
                                [self.last_msg.Vs_w_min - self.last_msg.paso_w/2, 
                                self.last_msg.Vs_x_min - self.last_msg.paso_v/2, 
                                self.last_msg.Vs_z_min - self.last_msg.paso_v/2]])
        colors = np.vstack([colors, [0.03, 0.659, 0.945, 0.2], [0.63, 0.645, 0.0, 0.1]])
        sizes = np.vstack([sizes, [ldx, ldy, ldz], [lsx, lsy, lsz]])
        #Plot
        pc = self.plotCubeAt(positions=positions_np, sizes=sizes,
                             colors=colors, edgecolor=[0,0,0,0])
        while len(self.ax.collections) > 0:
            self.ax.collections.pop()
        self.ax.add_collection3d(pc)
        
        return self.lines


    def visualInfoCallback(self, msg):
        recieved_first_msg = True
        self.last_msg = msg


    def cuboid_data(self, o, size=(1,1,1)):
        X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
            [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
            [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
            [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
            [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
            [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
        X = np.array(X).astype(float)
        for i in range(3):
            X[:,:,i] *= size[i]
        X += np.array(o)
        return X

    def plotCubeAt(self, positions,sizes=None,colors=None, **kwargs):
        if not isinstance(colors,(list,np.ndarray)): colors=["C0"]*len(positions)
        if not isinstance(sizes,(list,np.ndarray)): sizes=[(1,1,1)]*len(positions)
        g = []
        for p,s,c in zip(positions,sizes,colors):
            g.append( self.cuboid_data(p, size=s) )
        return Poly3DCollection(np.concatenate(g),  
                                facecolors=np.repeat(colors,6, axis=0), **kwargs)

if __name__ == '__main__':
    try:
        rospy.init_node("Dwa_Visual_Info")
        visualizer = Visualizer()
        info_subscriber = rospy.Subscriber("/DWA_visual_msg", DynamicWindowMsg, visualizer.visualInfoCallback)
        visualizer.last_msg = rospy.wait_for_message("/DWA_visual_msg", DynamicWindowMsg, timeout=None)
        animation = FuncAnimation(visualizer.fig, visualizer.update_plot, init_func=visualizer.plot_init)
        plt.show(block=True)
    except rospy.ROSInterruptException:
        pass
