#!/usr/bin/env python

# from _typeshed import NoneType
import rospy
import numpy as np
from std_msgs.msg import String
from colorama import Fore, Back, Style
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
import tf2_ros
import tf2_geometry_msgs

class PlotAssemblyData():

    def __init__(self):
        
        self.average_wrench = None
        self.avg_wrench_sub = rospy.Subscriber("/assembly_tools/avg_wrench", Wrench, self.callback_update_wrench, queue_size=2)

        self.average_speed = None
        self.avg_speed_sub = rospy.Subscriber("/assembly_tools/avg_speed", Point, self.callback_update_speed, queue_size=2)

        self.pos = None
        self.rel_position_sub = rospy.Subscriber("/assembly_tools/rel_position", Point, self.callback_update_pos, queue_size=2)

        self.status = None
        self.status_sub = rospy.Subscriber("/assembly_tools/status", String, self.callback_update_status, queue_size=2)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # Config variables
        self.rate = rospy.Rate(50)
        self.recordInterval = rospy.Duration(.1)
        self.plotInterval = rospy.Duration(.5)
        self.lastPlotted = rospy.Time(0)
        self.lastRecorded = rospy.Time(0)
        self.recordLength = 100
        self.surface_height = None
        

        rospy.loginfo(Fore.GREEN+Back.MAGENTA+"Plotter node active."+Style.RESET_ALL)
    

    def callback_update_wrench(self, data: Wrench):
        self.average_wrench = data
    
    def callback_update_speed(self, data: Point):
        self.average_speed = self.point_to_array(data)
    
    def callback_update_pos(self, data: Point):
        self.pos = self.point_to_array(data)
    
    def callback_update_status(self, data: String):
        
        self.status = dict(eval(str(data.data)))
        # If surface has been found we add a line to the plot:
        if('surface_height' in self.status and self.surface_height == None):
            self.surface_height = self.status['surface_height']
        rospy.logwarn_throttle(1, "Status is " + str(self.status))

    def point_to_array(self, point):
        return np.array([point.x, point.y, point.z])

    def init_plot(self):
            #plt.axis([-50,50,0,10000])


        # Data containers
        self.speedHistory = np.array(self.average_speed)

        self.forceHistory = self.point_to_array(self.average_wrench.force)
        # 3xN array, vertically stacked positions
        self.posHistory = self.pos*1000
        self._start_time = rospy.get_rostime()
        self.plotTimes = [self._start_time.to_sec()]
        plt.ion()
        # plt.show()
        # plt.draw()

        self.fig, (self.planView, self.sideView) = plt.subplots(1, 2)
        self.fig.suptitle('Algorithm Path Plots')
        plt.subplots_adjust(left=.125, bottom=.2, right=.975, top=.8, wspace=.175, hspace=.1)

        self.min_plot_window_offset = 2
        self.min_plot_window_size = 15
        self.pointOffset = 1
        self.barb_interval = 5
    
    def update_plots(self):
        #Record data to the buffer
        if(rospy.Time.now() > self.lastRecorded + self.recordInterval):
            self.lastRecorded = rospy.Time.now()

            #log all interesting data
            self.speedHistory = np.vstack((self.speedHistory, np.array(self.average_speed)*1000))
            self.forceHistory = np.vstack((self.forceHistory, self.point_to_array(self.average_wrench.force)))
            self.posHistory = np.vstack((self.posHistory, self.pos*1000))
            self.plotTimes.append((rospy.get_rostime() - self._start_time).to_sec())
            
            #limit list lengths
            if(len(self.speedHistory)>self.recordLength):
                self.speedHistory = self.speedHistory[1:-1]
                self.forceHistory = self.forceHistory[1:-1]
                self.posHistory = self.posHistory[1:-1]
                self.plotTimes = self.plotTimes[1:-1]
                self.pointOffset += 1

        #Actually plot the data; this is computation-heavy so we minimize the hz
        if(rospy.Time.now() > self.lastPlotted + self.plotInterval):
            self.lastPlotted = rospy.Time.now()

            self.planView.clear()
            self.sideView.clear()

            self.planView.set(xlabel='X Position',ylabel='Y Position')
            self.planView.set_title('XY Position and Force')
            self.sideView.set(xlabel='Time (s)',ylabel='Position (mm) and Force (N)')
            self.sideView.set_title('Vertical Position and Force')

            

            self.planView.plot(self.posHistory[:,0], self.posHistory[:,1], 'r')

            #self.planView.quiver(self.posHistory[0:-1:10,0], self.posHistory[0:-1:10,1], self.forceHistory[0:-1:10,0], self.forceHistory[0:-1:10,1], angles='xy', scale_units='xy', scale=.1, color='b')
            barb_increments = {"flag" :5, "full" : 1, "half" : 0.5}

            offset = self.barb_interval+1-(self.pointOffset % self.barb_interval)
            self.planView.barbs(self.posHistory[offset:-1:self.barb_interval,0], self.posHistory[offset:-1:self.barb_interval,1], 
                self.forceHistory[offset:-1:self.barb_interval,0], self.forceHistory[offset:-1:self.barb_interval,1],
                barb_increments=barb_increments, length = 6, color=(0.2, 0.8, 0.8))
            #TODO - Fix barb directions. I think they're pointing the wrong way, just watching them in freedrive mode.

            self.sideView.plot(self.plotTimes[:], self.forceHistory[:,2], 'k', self.plotTimes[:], self.posHistory[:,2], 'b')

            xlims = [np.min(self.posHistory[:,0]) - self.min_plot_window_offset, np.max(self.posHistory[:,0]) + self.min_plot_window_offset]
            if xlims[1] - xlims[0] < self.min_plot_window_size:
                xlims[0] -= self.min_plot_window_size / 2
                xlims[1] += self.min_plot_window_size / 2

            ylims = [np.min(self.posHistory[:,1]) - self.min_plot_window_offset, np.max(self.posHistory[:,1]) + self.min_plot_window_offset]
            if ylims[1] - ylims[0] < self.min_plot_window_size:
                ylims[0] -= self.min_plot_window_size / 2
                ylims[1] += self.min_plot_window_size / 2
            
            self.planView.set_xlim(xlims)
            self.planView.set_ylim(ylims)
            
            self.planView.annotate("State is " + self.status['state'], xy=(.05, .03), xycoords='figure fraction',size=10, ha="left", va="bottom",
            bbox=dict(boxstyle="round4",
            ec=(1., 0.5, 0.5),
            fc=(1., 0.8, 0.8)
            )
            )

            self.planView.annotate("TCP: " + self.status['tcp_name'], xy=(.95, .03), xycoords='figure fraction',size=10, ha="right", va="bottom",
            bbox=dict(boxstyle="round4",
            ec=(1., 0.5, 0.5),
            fc=(1., 0.8, 0.8)
            )
            )

            force_message = "Force x: " + str(np.round(self.average_wrench.force.x,2)) + "\nForce y: " + str(np.round(self.average_wrench.force.y,2))

            self.planView.annotate(force_message, xy=(3, 3), xycoords='axes points',size=8, ha="left", va="bottom",
            )

            vert_message = "Force z: " + str(np.round(self.average_wrench.force.z,2)) + "\nPosition z :" + str(np.round(self.pos[2],2))

            self.sideView.annotate(vert_message, xy=(3, 3), xycoords='axes points',size=8, ha="left", va="bottom",
            )
            
            self.fig.canvas.draw()


    def main(self):

        # vital_data = [self.average_speed, self.average_wrench, self.status, self.pos]
        # #Wait till we have real values for everything
        x=True
        while (type(None) in [type(self.average_speed), type(self.average_wrench), type(self.status), type(self.pos)]):
            # vital_data = [self.average_speed, self.average_wrench, self.status, self.pos]
            # x = vital_data[:] == [2,2,2,2]

            self.rate.sleep()
        
        rospy.loginfo(Fore.GREEN+Back.MAGENTA+"Plotter node starting."+Style.RESET_ALL)

        self.init_plot()

        while (not rospy.is_shutdown()):
            self.update_plots()
            self.rate.sleep()