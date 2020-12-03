#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import String

class discrete_bayes_filter:

    def __init__(self):

        # init subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.move_sub = rospy.Subscriber('/move', String, self.move_callback)

        # init publishers
        self.pos_pub = rospy.Publisher('/robot_pos', Point, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.grid_pub = rospy.Publisher('/robot_pos_map', OccupancyGrid, queue_size=10)

        #init parameters
        self.map_width = 20
        self.map_height = 20
        self.probability_grid_map = np.zeros((self.map_height, self.map_height))
        
        self.motion_probs = rospy.get_param('~/robot_move_probabilities', [0.9, 0.04, 0.04, 0.0, 0.02])
        
        self.prob_scan_correct = 0.8
        self.prob_scan_oneoff = 0.1

        #init the world map
        self.map_input = OccupancyGrid()

        #init output robot pos
        self.assumed_robot_pos = Point()

        #init marker
        self.pos_marker = Marker()
        self.pos_marker.header.frame_id = "map"
        self.pos_marker.ns = "navigation"
        self.pos_marker.id = 0
        self.pos_marker.type = Marker.CUBE
        self.pos_marker.action = Marker.ADD
        self.pos_marker.scale.x = 1
        self.pos_marker.scale.y = 1
        self.pos_marker.scale.z = 0.2
        self.pos_marker.color.a = 1.0
        self.pos_marker.color.r = 0.0
        self.pos_marker.color.g = 1.0
        self.pos_marker.color.b = 0.0
        self.pos_marker.pose.orientation = Quaternion(0, 0, 0, 1)

        #init the output OccupancyGrid
        self.pub_grid_map = OccupancyGrid()
        self.pub_grid_map.info.resolution = 1
        self.pub_grid_map.info.height = self.map_height
        self.pub_grid_map.info.width = self.map_width

    # fill the probability grid
    # and store the ground truth distances for each grid cell
    def precomputation(self):
        self.scan_groundtruth_grid = np.zeros((self.map_height, self.map_width, 4))
        free = 0 #counter of free cells

        for x in range(self.map_height):
            for y in range(self.map_width):
                if self.map[x, y] == 0:
                    free += 1
                    
                    # set the probability to 1 if grid cell is free, normalize later
                    self.probability_grid_map[x, y] = 1

                    dist_n = 0
                    dist_e = 0
                    dist_s = 0
                    dist_w = 0

                    # determine distance to northern wall
                    x_1 = x
                    y_1 = y
                    while x_1 < self.map_height and self.map[x_1, y_1] != 100:
                        dist_n += 1
                        x_1 += 1

                    # determine distance to eastern wall
                    x_1 = x
                    y_1 = y
                    while y_1 < self.map_width and self.map[x_1, y_1] != 100:
                        dist_e += 1
                        y_1 += 1

                    # determine distance to southern wall
                    x_1 = x
                    y_1 = y
                    while x_1 >= 0 and self.map[x_1, y_1] != 100:
                        dist_s += 1
                        x_1 -= 1

                    # determine distance to western wall
                    x_1 = x
                    y_1 = y
                    while y_1 >= 0 and self.map[x_1, y_1] != 100:
                        dist_w += 1
                        y_1 -= 1

                    # store distances
                    self.scan_groundtruth_grid[x][y] = np.array([dist_s, dist_w, dist_n, dist_e])
        
        #normalize the probability grid map
        for x in range(self.map_height):
            for y in range(self.map_width):
                self.probability_grid_map[x, y] /= free

    #perform a prediction step based on the intended move direction
    def prediction_step(self):
        #get probabilities for actual movement directions
        if self.move_input == 'N':
            self.prob_move_n = self.motion_probs[0] #forward
            self.prob_move_w = self.motion_probs[1] #left
            self.prob_move_e = self.motion_probs[2] #right
            self.prob_move_s = self.motion_probs[3] #back

        elif self.move_input == 'E':
            self.prob_move_e = self.motion_probs[0] #forward
            self.prob_move_n = self.motion_probs[1] #left
            self.prob_move_s = self.motion_probs[2] #right
            self.prob_move_w = self.motion_probs[3] #back

        elif self.move_input == 'S':
            self.prob_move_s = self.motion_probs[0] #forward
            self.prob_move_e = self.motion_probs[1] #left
            self.prob_move_w = self.motion_probs[2] # right
            self.prob_move_n = self.motion_probs[3] #back

        elif self.move_input == 'W':
            self.prob_move_w = self.motion_probs[0] #forward
            self.prob_move_s = self.motion_probs[1] #left
            self.prob_move_n = self.motion_probs[2] #right
            self.prob_move_e = self.motion_probs[3] #back

        self.prob_move_stay = self.motion_probs[4] #stay

        # update predictions
        probability_grid_map_old = np.copy(self.probability_grid_map)
        for x in range(self.map_height):
            for y in range(self.map_width):
                if probability_grid_map_old[x, y] != 0:
                    # movement north
                    if self.map[x+1, y] != 100:
                        self.probability_grid_map[x+1,y] += probability_grid_map_old[x, y] * self.prob_move_n
                    else:
                        self.probability_grid_map[x,y] += probability_grid_map_old[x, y] * self.prob_move_n

                    # movement south
                    if self.map[x-1, y] != 100:
                        self.probability_grid_map[x-1,y] += probability_grid_map_old[x, y] * self.prob_move_s
                    else:
                        self.probability_grid_map[x,y] += probability_grid_map_old[x, y] * self.prob_move_s

                    # movement east
                    if self.map[x, y+1] != 100:
                        self.probability_grid_map[x,y+1] += probability_grid_map_old[x, y] * self.prob_move_e
                    else:
                        self.probability_grid_map[x,y] += probability_grid_map_old[x, y] * self.prob_move_e

                    # movement west
                    if self.map[x, y-1] != 100:
                        self.probability_grid_map[x,y-1] += probability_grid_map_old[x, y] * self.prob_move_w
                    else:
                        self.probability_grid_map[x,y] += probability_grid_map_old[x, y] * self.prob_move_w

                    # stay in place
                    self.probability_grid_map[x,y] += probability_grid_map_old[x, y] * self.prob_move_stay
     
    #perform a correction step, based on sensor measurements
    def correction_step(self):
        for pos_x in range(self.map_height):
            for y in range(self.map_width):

                # only look at valid configurations
                if sum(self.scan_groundtruth_grid[pos_x, y]) != 0:

                    # determine the difference between measurement and ground truth for each cell
                    diff_pos = np.abs(self.scan_groundtruth_grid[pos_x, y] - self.scan_input)

                    #sensor can't be more than one off
                    # -> likelihood zero if difference greater 1
                    if max(diff_pos) > 1:
                        self.probability_grid_map[pos_x, y] = 0.0
                    else:
                        for diff in diff_pos:
                            if diff == 0:
                                self.probability_grid_map[pos_x, y] *= self.prob_scan_correct
                            else:
                                self.probability_grid_map[pos_x, y] *= self.prob_scan_oneoff
        
        #normalize
        self.probability_grid_map /= sum(sum(probability_grid_map))

        # publish probability grid map
        self.pub_grid_map.data = self.probability_grid_map.astype(int)
        self.pub_grid_map.data = np.array(self.pub_grid_map.data.flatten())
        self.grid_pub.publish(self.pub_grid_map)

        # determine cell with maximum prediction value and publish it
        prediction_max = self.probability_grid_map.argmax()

        self.assumed_robot_pos.x = prediction_max % self.map_width
        self.assumed_robot_pos.y = int(prediction_max / self.map_width)
        
        self.pos_pub.publish(self.assumed_robot_pos)

        # publish marker
        self.pos_marker.pose.position.x = self.assumed_robot_pos.x + 0.5
        self.pos_marker.pose.position.y = self.assumed_robot_pos.y + 0.5
        self.marker_pub.publish(self.pos_marker)

    def run(self, rate: float = 10):
        #rospy.sleep(1)
        rospy.wait_for_message('/map', String)
        self.precomputation()

        while not rospy.is_shutdown():
            rospy.wait_for_message('/move', String)
            rospy.wait_for_message('/scan', LaserScan)

            self.prediction_step()
            self.correction_step()

    #callback functions
    def map_callback(self, OccupancyGrid):
        self.map_input = np.array(OccupancyGrid.data)
        self.map = self.map_input.reshape(self.map_width, self.map_height)

    def scan_callback(self, LaserScan):
        self.scan_input = np.array(LaserScan.ranges)

    def move_callback(self, String):
        self.move_input= String.data
if __name__ == "__main__":
    rospy.init_node('discrete_bayes_filter')

    probability_location = discrete_bayes_filter()
    probability_location.run(rate = 10)