# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

# constants
occ_bins = [-1, 0, 100, 101]


class Occupy(Node):

    def __init__(self):
        super().__init__('occupy')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        # occdata = np.array([])

    def listener_callback(self, msg):
        # create numpy array
        occdata = np.array([msg.data])
        # compute histogram to identify percent of bins with -1
        occ_counts = np.histogram(occdata,occ_bins)
        # calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        # log the info
        self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))
        # make occdata go from 0 to 2 so we can use uint8, which won't be possible if we have
        # negative values
        # first make negative values 0
        occ2 = occdata + 1
        # now change all the values above 1 to 2
        occ3 = (occ2>1).choose(occ2,2)
        # convert into 2D array using column order
        odata = np.uint8(occ3.reshape(msg.info.height,msg.info.width))
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # show the image using grayscale map
        plt.imshow(img,cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)


def main(args=None):
    rclpy.init(args=args)

    occupy = Occupy()

    # create matplotlib figure
    plt.ion()
    plt.show()

    rclpy.spin(occupy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
