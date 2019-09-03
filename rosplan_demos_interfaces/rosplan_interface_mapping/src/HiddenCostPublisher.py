#!/usr/bin/env python
import rospy
import math  
from itertools import product
from nav_msgs.msg import OccupancyGrid

def talker():
    rospy.init_node('hidden_cost_publisher', anonymous=True)
    pub = rospy.Publisher('costmap', OccupancyGrid, queue_size=10)

    if rospy.has_param('~peaks'):
        peaks = rospy.get_param('~peaks')
    rate = rospy.Rate(0.05)
    while not rospy.is_shutdown():
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05
        grid.info.width = 556
        grid.info.height = 1029
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0

        for y in range(grid.info.height):
            for x in range(grid.info.width):
                cost = 1
                for (posx,posy,r) in peaks:
                    gridx = posx/grid.info.resolution
                    gridy = posy/grid.info.resolution
                    d = math.sqrt( float ( abs(gridx-x)*abs(gridx-x) + abs(gridy-y)*abs(gridy-y) ) )
                    if d <= r:
                        new_cost = 100.0 * (1 - float(d)/float(r))
                        if new_cost > cost:
                            cost = new_cost
                grid.data.append(int(cost))

        pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
