# Copyright 1996-2020 Soft_illusion.
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
# This script is used to process the incoming image and calculate cmd_vel.
import threading

from example_interfaces.action import Fibonacci

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,Vector3
from rock_rhino_msgs.msg import DetectedTag
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from queue import Queue

def search(list, n):
    for i in range(len(list)):
        if list[i] == n:
            return True
    return False

class ArucoController(Node):
    def __init__(self):
        super().__init__('image_processor')
        # Publish cmd vel
        self.pubs_detected_tag = self.create_publisher(DetectedTag, 'image_processor/detected_tag', 1)
        #action sever from process_controller
        self._goal_handle = None
        self.recent_tags = Queue(maxsize = 3)
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'image_processor/action_server_pro_ctr',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())


        # vehicle parameters
        self.speed = 0.1
        self.angle_correction = -10.0

        # Initialize parameters
        self.delta = 0
        self.cmd = Twist()
        self.detected_tag = DetectedTag()
        self.stop = False
        self.count = 0
        self.number_of_pixels = 512
        # value is in pixels 40% of image
        self.reach_threshold = self.number_of_pixels * 0.4
        self.reached = False

        self.camera_subscriber = self.create_subscription(
                                Image,
                                'camera/image_raw',
                                self.image_processing_callback,
                                qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.processedImage_publish = self.create_publisher(
                                      Image,
                                      'camera/processed_image',
                                      1)
        self.translation_x = 0
        self.get_logger().info('222222222222222222222222222-----Aruco Init Done.')

    def broadcast_detected_tag(self):
        # Constant velocity
        self.cmd.linear.x = self.speed

        # # Correction parameters
        self.cmd.angular.z = self.angle_correction * self.translation_x

        if self.stop:
            self.cmd.linear.x = 0.01
            self.cmd.angular.z = 1.2

        if self.reached:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        # Publish cmd vel
        #self.detected_tag.tag_id = 10

        self.pubs_detected_tag.publish(self.detected_tag)

    def image_processing_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        height = msg.height
        width = msg.width
        self.number_of_pixels = height
        matrix_coefficients = np.mat([[1.0, 0.0, height / 2.0],
                                     [0.0, 1.0, width / 2.0],
                                     [0.0, 0.0, 1.0]])
        distortion_coefficients = np.mat([0.0, 0.0, 0.0, 0.0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        # Use 5x5 dictionary to find markers
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        # Marker detection parameters
        parameters = aruco.DetectorParameters_create()
        # lists of ids and the corners beloning to each id
        corners, ids, _ = aruco.detectMarkers(
                            gray,
                            aruco_dict,
                            parameters=parameters,
                            cameraMatrix=matrix_coefficients,
                            distCoeff=distortion_coefficients)

        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                if self.recent_tags.full():
                    self.recent_tags.get() #kick off the front ele if the queue is full
                if search(list(self.recent_tags.queue), int(ids[i][0])):
                    None
                 #   self.get_logger().info('tag have been added:{0}'.format(list(self.recent_tags.queue)))
                else:
                    self.recent_tags.put(int(ids[i][0]))
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                                        corners[i],
                                        0.1,
                                        matrix_coefficients,
                                        distortion_coefficients)

                    self.detected_tag.tag_id = int(ids[i][0])
                    #m = Vector3(x = rvec[0][0][0], y = rvec[0][0][1], z = rvec[0][0][2])
                # m.x = rvec[0][0][0]
                # m.y = rvec[0][0][1]
                #  m.z = rvec[0][0][2]
                    self.detected_tag.position = Vector3(x = tvec[0][0][0], y = tvec[0][0][1], z = tvec[0][0][2]) #[1.1,2.1,3.1] #rvec[0][0][0]
                    self.detected_tag.posture = Vector3(x = rvec[0][0][0], y = rvec[0][0][1], z = rvec[0][0][2]) #[1.1,2.1,3.1] #rvec[0][0][0]
                    self.broadcast_detected_tag()
                    # Draw A square around the markers
                    aruco.drawDetectedMarkers(frame, corners)
                    aruco.drawAxis(frame, matrix_coefficients,
                                distortion_coefficients,
                                rvec,
                                tvec,
                                0.1)  # Draw Axis
                    top = corners[0][0][0][0]
                    bottom = corners[0][0][2][0]
                    if abs(top - bottom) > self.reach_threshold:
                        self.reached = True
                        self.get_logger().info('Reached the goal')
                    else:
                        self.reached = False
                    self.translation_x = tvec[0][0][0]
                    self.stop = False
        else:
            self.translation_x = 0
            self.stop = True
        self.processedImage_publish.publish(self.bridge.cv2_to_imgmsg(frame,
                                                                      "rgb8"))
########START: define request action from process_controller

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Append the seeds for the Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Start executing the action
        for i in range(1, goal_handle.request.order):
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return Fibonacci.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Update Fibonacci sequence
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        goal_handle.succeed()

        # Populate result message
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result


######## END: define request action from process controller


def main(args=None):

    rclpy.init(args=args)

    commander = ArucoController()
    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(commander,  executor=executor)

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
