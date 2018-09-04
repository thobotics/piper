#!/usr/bin/env python

import rospy
import numpy as np
import imageio
import math
from scipy.ndimage.morphology import distance_transform_edt
from piper.msg import FieldGrid
from std_msgs.msg import Bool

shutdown_flag = False

def signedDistanceField2D_(ground_truth_map, cell_size):
    # regularize unknow area to open area
    map = (ground_truth_map > 0.75);
    # inverse map
    inv_map = 1 - map;

    # get signed distance from map and inverse map
    map_dist = distance_transform_edt(inv_map) # bwdist(map);
    inv_map_dist = distance_transform_edt(map) # bwdist(inv_map);

    field = map_dist - inv_map_dist;

    # metric
    field = field * cell_size;
    # field = float(field);

    # limit inf
    if math.isinf(field[0,0]):
        field = np.ones(field.shape) * 1000

    return field

def loadMap_(image_path="./maps/social_contexts.png"):
    cell_size = 0.05
    field_grid = FieldGrid()
    grid = imageio.imread(image_path)

    # Generate OccupancyGrid
    bgrid = np.array(grid[:,:,0]).T
    bgrid[bgrid == 0] = int(1.)
    bgrid[bgrid == 255] = int(0.)
    bgrid = np.rot90(bgrid, 1)
    field = signedDistanceField2D_(bgrid, cell_size)

    field_grid.info.resolution = cell_size
    field_grid.info.width = grid.shape[0]
    field_grid.info.height = grid.shape[1]
    field_grid.info.origin.position.x = 0.#-20.
    field_grid.info.origin.position.y = 0.#-20.
    field_grid.data = field.flatten()

    return field_grid

def fieldgrid_ack_cb(data):
    print("Sent map, Ack received ", data.data)
    global shutdown_flag
    shutdown_flag = data.data

def run():
    rospy.init_node('png2fieldgrid')

    # Create publisher
    fg_pub = rospy.Publisher('/map_grid', FieldGrid, queue_size=10)
    rospy.Subscriber("/ack_map_grid", Bool, fieldgrid_ack_cb)
    rate = rospy.Rate(0.5) # 2 secs

    grid_map = loadMap_()

    global shutdown_flag
    while not rospy.is_shutdown() and not shutdown_flag:
        fg_pub.publish(grid_map)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
