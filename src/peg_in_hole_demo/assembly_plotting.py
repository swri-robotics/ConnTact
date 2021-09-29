#!/usr/bin/env python

import rospy
import numpy as np
from colorama import Fore, Back, Style
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped, Wrench, TransformStamped, PoseStamped, Pose, Point, Quaternion, Vector3, Transform
import tf2_ros
import tf2_geometry_msgs

class PlotAssemblyData():

    def __init__(self):
        
        self.average_wrench = Wrench()
        self.avg_wrench_sub = rospy.Subscriber("/assembly_tools/avg_wrench", WrenchStamped, self.callback_update_wrench, queue_size=2)

        self.average_speed = np.arange(3)
        self.avg_speed_sub = rospy.Subscriber("/assembly_tools/avg_speed", Point, self.callback_update_speed, queue_size=2)

        self.pos = Point()
        self.rel_position_sub = rospy.Subscriber("/assembly_tools/rel_position", Point, self.callback_update_pos, queue_size=2)
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #plotting parameters
        # Need avg_speed, avg_wrench, position, state

        self.speedHistory = np.array(self.average_speed)
        self.forceHistory = self._as_array(self.average_wrench.force)
        self.posHistory = np.array([self.pos.x*1000, self.pos.y*1000, self.pos.z*1000])
        self.plotTimes = [0]
        self.recordInterval = rospy.Duration(.1)
        self.plotInterval = rospy.Duration(.5)
        self.lastPlotted = rospy.Time(0)
        self.lastRecorded = rospy.Time(0)
        self.recordLength = 100
        self.surface_height = None

        rospy.loginfo(Fore.GREEN+Back.MAGENTA+"Hello! I'm alive!"+Style.RESET_ALL)
        quit()
    
    def _init_plot(self):
            #plt.axis([-50,50,0,10000])
        plt.ion()
        # plt.show()
        # plt.draw()

        self.fig, (self.planView, self.sideView) = plt.subplots(1, 2)
        self.fig.suptitle('Horizontally stacked subplots')
        plt.subplots_adjust(left=.1, bottom=.2, right=.95, top=.8, wspace=.15, hspace=.1)

        self.min_plot_window_offset = 2
        self.min_plot_window_size = 10
        self.pointOffset = 1
        self.barb_interval = 5
    def callback_update_wrench(self):
        pass
    
    def callback_update_speed(self):
        pass
    
    def callback_update_pos(self):
        pass
    
    def _update_plots(self):
        if(rospy.Time.now() > self.lastRecorded + self.recordInterval):
            self.lastRecorded = rospy.Time.now()

            #log all interesting data
            self.speedHistory = np.vstack((self.speedHistory, np.array(self.average_speed)*1000))
            self.forceHistory = np.vstack((self.forceHistory, self._as_array(self.average_wrench.force)))
            self.posHistory = np.vstack((self.posHistory, self._as_array(self.current_pose.transform.translation)*1000))
            self.plotTimes.append((rospy.get_rostime() - self._start_time).to_sec())
            
            #limit list lengths
            if(len(self.speedHistory)>self.recordLength):
                self.speedHistory = self.speedHistory[1:-1]
                self.forceHistory = self.forceHistory[1:-1]
                self.posHistory = self.posHistory[1:-1]
                self.plotTimes = self.plotTimes[1:-1]
                self.pointOffset += 1

        if(rospy.Time.now() > self.lastPlotted + self.plotInterval):
            
            self.lastPlotted = rospy.Time.now()

            self.planView.clear()
            self.sideView.clear()

            self.planView.set(xlabel='X Position',ylabel='Y Position')
            self.planView.set_title('Sped and Poseation and Forke')
            self.sideView.set(xlabel='Time (s)',ylabel='Position (mm) and Force (N)')
            self.sideView.set_title('Vertical position and force')

            self.planView.plot(self.posHistory[:,0], self.posHistory[:,1], 'r')
            #self.planView.quiver(self.posHistory[0:-1:10,0], self.posHistory[0:-1:10,1], self.forceHistory[0:-1:10,0], self.forceHistory[0:-1:10,1], angles='xy', scale_units='xy', scale=.1, color='b')
            barb_increments = {"flag" :5, "full" : 1, "half" : 0.5}

            offset = self.barb_interval+1-(self.pointOffset % self.barb_interval)
            self.planView.barbs(self.posHistory[offset:-1:self.barb_interval,0], self.posHistory[offset:-1:self.barb_interval,1], 
                self.forceHistory[offset:-1:self.barb_interval,0], self.forceHistory[offset:-1:self.barb_interval,1],
                barb_increments=barb_increments, length = 6, color=(0.2, 0.8, 0.8))
            #TODO - Fix barb directions. I think they're pointing the wrong way, just watching them in freedrive mode.

            self.sideView.plot(self.plotTimes, self.forceHistory[:,2], 'k', self.plotTimes, self.posHistory[:,2], 'b')

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
                        
            if not self.surface_height is None:
                self.sideView.axhline(y=self.surface_height*1000, color='r', linestyle='-')

            self.fig.canvas.draw()
            # plt.pause(0.001)
            #plt.show()

            #x velocity
            #plt.scatter(self.speedHistory[:][0], )