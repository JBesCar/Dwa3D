#!/usr/bin/python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton, QLabel, QLineEdit, QCheckBox
import PyQt5.QtCore as qtc
import rospy
from std_msgs.msg import String
from threading import Thread
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool


class CommanderGUIWidget(QWidget):

    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_ros()
        self.spin_thread = Thread(target=self.ros_spin)
        self.spin_thread.start()

    def ros_spin(self):
        rospy.spin()
    
    def init_ros(self):
        rospy.init_node("commander_gui")
        self.init_publishers()
    
    def init_publishers(self):
        self.order_pub = rospy.Publisher("order", String, queue_size=1)
        self.order_sub = rospy.Subscriber("order", String, self.orderCallback, queue_size=1)

    def orderCallback(self, msg):
        self.current_order = msg.data
        if(self.current_order == "TAKEOFF"):
            self.takeoff_button.setStyleSheet("background-color: green; color: white;")
            self.navigate_button.setStyleSheet("background-color: white; color: black;")
            self.land_button.setStyleSheet("background-color: white; color: black;")
            self.idle_button.setStyleSheet("background-color: white; color: black;")
            self.hover_button.setStyleSheet("background-color: white; color: black;")
        elif(self.current_order == "NAVIGATE"):
            self.takeoff_button.setStyleSheet("background-color: white; color: black;")
            self.navigate_button.setStyleSheet("background-color: green; color: white;")
            self.land_button.setStyleSheet("background-color: white; color: black;")
            self.idle_button.setStyleSheet("background-color: white; color: black;")
            self.hover_button.setStyleSheet("background-color: white; color: black;")
        elif(self.current_order == "LAND"):
            self.takeoff_button.setStyleSheet("background-color: white; color: black;")
            self.navigate_button.setStyleSheet("background-color: white; color: black;")
            self.land_button.setStyleSheet("background-color: green; color: white;")
            self.idle_button.setStyleSheet("background-color: white; color: black;")
            self.hover_button.setStyleSheet("background-color: white; color: black;")
        elif(self.current_order == "IDLE"):
            self.takeoff_button.setStyleSheet("background-color: white; color: black;")
            self.navigate_button.setStyleSheet("background-color: white; color: black;")
            self.land_button.setStyleSheet("background-color: white; color: black;")
            self.idle_button.setStyleSheet("background-color: green; color: white;")
            self.hover_button.setStyleSheet("background-color: white; color: black;")
        elif(self.current_order == "HOVER"):
            self.takeoff_button.setStyleSheet("background-color: white; color: black;")
            self.navigate_button.setStyleSheet("background-color: white; color: black;")
            self.land_button.setStyleSheet("background-color: white; color: black;")
            self.idle_button.setStyleSheet("background-color: white; color: black;")
            self.hover_button.setStyleSheet("background-color: green; color: white;")
        
    
    def init_ui(self):
        # Set up the UI.
        self.takeoff_button = QPushButton("TAKEOFF")
        self.navigate_button = QPushButton("NAVIGATE")
        self.land_button = QPushButton("LAND")
        self.idle_button = QPushButton("IDLE (OFFBOARD)")
        self.hover_button = QPushButton("HOVER (POSITION)")

        grid_layout = QGridLayout()
        grid_layout.addWidget(self.takeoff_button, 0, 0)
        grid_layout.addWidget(self.navigate_button, 1, 0)
        grid_layout.addWidget(self.land_button, 2, 0)
        grid_layout.addWidget(self.idle_button, 3, 0)
        grid_layout.addWidget(self.hover_button, 4, 0)
        self.setLayout(grid_layout)

        # ROS setup.
        self.takeoff_button.clicked.connect(self.send_takeoff)
        self.navigate_button.clicked.connect(self.send_navigate)
        self.land_button.clicked.connect(self.send_land)
        self.idle_button.clicked.connect(self.send_idle)
        self.hover_button.clicked.connect(self.send_hover)
        self.setObjectName('Tyrion Commander')

    def send_takeoff(self, state):
        msg = String(data=("TAKEOFF"))
        self.order_pub.publish(msg)
        rospy.loginfo("SENT TAKEOFF ORDER")

    def send_navigate(self, state):
        msg = String(data=("NAVIGATE"))
        self.order_pub.publish(msg)
        rospy.loginfo("SENT NAVIGATE ORDER")

    def send_land(self, state):
        msg = String(data=("LAND"))
        self.order_pub.publish(msg)
        rospy.loginfo("SENT LAND ORDER")

    def send_idle(self, state):
        msg = String(data=("IDLE"))
        self.order_pub.publish(msg)
        rospy.loginfo("SENT IDLE ORDER")

    def send_hover(self, state):
        msg = String(data=("HOVER"))
        self.order_pub.publish(msg)
        rospy.loginfo("SENT HOVER ORDER")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CommanderGUIWidget()
    window.show()
    sys.exit(app.exec_())
    window.spin_thread.join()