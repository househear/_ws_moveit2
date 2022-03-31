# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient

import _thread

import sys

import geometry_msgs.msg
import rclpy
from moveit_msgs.srv import RespondProcessCtr
from rclpy.node import Node




class Action_Client_2_PKG:
         
    # The init method or constructor
    def __init__(self, _parent_node, _process_controller, _action_server_name):
         
        # Instance Variable
        self.action_client = ActionClient(self, Fibonacci, _action_server_name),        
        self.process_controller = _process_controller
        self.parent_node = _parent_node
        self.action_server_name = _action_server_name
 
    # Adds an instance variable

######## START: call_back for action client talking to image_processor

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.parent_node.get_logger().info('Goal rejected :(')
            return

        self.parent_node.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.parent_node.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.parent_node.get_logger().info('Goal succeeded! Result: {0}'.format(result.sequence))
            self.process_controller.runtask_completed_call_back(self.action_server_name + self.task_name)
        else:
            self.parent_node.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        #rclpy.shutdown()

    def send_task(self, _task_name):
        self.task_name = _task_name
        self.parent_node.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = 4

        self.parent_node.get_logger().info('Sending goal request...')

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

######## END: call_back for action client talking to image_processor



class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('rock_rhino_process_controller')
        self.cli = self.create_client(RespondProcessCtr, 'add_two_ints__')

        self.pub = self.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

        # define action client that reuests image_processor to detect the tag on the live image
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')


      #  while not self.cli.wait_for_service(timeout_sec=1.0):
       #     self.get_logger().info('service not available, waiting again...')
        self.req = RespondProcessCtr.Request()
    
    def set_action_client(self, _process_controller):
        self.action_client_image_processor = \
        Action_Client_2_PKG(self, _process_controller, 'fibonacci')

        self.action_client_data_base = \
        Action_Client_2_PKG(self, _process_controller, 'data_base')




if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
"""

moveBindings = {
    'c': (1, 0, 0, 0),
    'C': (1, 0, 0, -1),

}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),

}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

def process_status_controller(_node, n):

    settings = saveTerminalSettings()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    try:
        print(msg)
        #print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                command_string = input('Enter your command: ')
                print('Command sent:',command_string)
                _node.send_goal()

            
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                #print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            #pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        #pub.publish(twist)

        restoreTerminalSettings(settings)


class Process_Controller:
     
    # Class Variable
    stream = 'cse'     
     
    # The init method or constructor
    def __init__(self, _node):
         
        # Instance Variable
        self.node = _node       
        self.address = ''  
        self.process_status = 'idle' 
        self.run_mode = 'manual'
 
    # Adds an instance variable
    def setAddress(self, address):
        self.address = address

    def set_process_status(self, _process_status):
        self.process_status = _process_status
     
    # Retrieves instance variable   
    def getAddress(self):   
        return self.address  


    def send_runtask(self, task_name):
        if task_name == 'detect_tag':
            self.node.action_client_image_processor.send_task()
        if task_name == 'update_db':
            self.node.action_client_data_base.send_task()
        else:
            print('Unknown command....')


    def send_runjob(self, job_mode):
        if job_mode == 'conti':

            self.node.action_client_image_processor.send_task()
            self.set_process_status('image_processing')
            while self.process_status != 'idle':
                m = 1
                print('image_processing....')
            print('image_processing done')
            self.node.action_client_data_base.send_task()      
            self.set_process_status('updating_d')     
            while self.process_status != 'idle':
                m = 1
                print('data_base updating...')
            print('data_base_updating done')

        if job_mode == 'step':
            self.node.action_client_data_base.send_task()
        else:
            print('Unknown command....')


    def runtask_completed_call_back(self, task_name):
        print(task_name, ' has completed, process_status is set to idle')
        self.set_process_status('idle')
        

    def parse_entered_command(self, command_string):
        res = command_string.split()
        try:
            if res[0] == 'system_mode':
                if res[1] == 'check':
                    print('Current system_ mode: ', self.process_status)
                elif res[1] == 'change':
                    print('Current system_ mode is changed: ')
                elif res[1] == 'help':
                    print('msg_system_mode')
                else:
                    print('Unknown command: enter: system_mode help to get info about this command')

            elif res[0] == 'run_mode':
                if res[1] == 'check':
                    print('Current run_ mode: ')
                elif res[1] == 'change':
                    print('Current run_ mode is changed: ')
                else:
                    print('Unknown command: ')

            elif res[0] == 'process_status':
                if res[1] == 'check':
                    print('Current process_status: ')
                elif res[1] == 'change':
                    print('Current process_status is changed: ')
                elif res[1] == 'run':
                    print('run  ')
                    self.node.send_goal()
                else:
                    print('Unknown command: ')

            elif res[0] == 'runtask':
                self.send_runtask(res[1])

            elif res[0] == 'runjob':
                self.send_runjob(res[1])

            else:
                print('Unknown command: ')
        except Exception as e:
            #print(e)
            print('Unknown command')



def process_status_controller_test(process_controller, n):
    

    settings = saveTerminalSettings()
    build_mode = 'bui'

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0


    print(msg)
    #print(vels(speed, turn))
    while True:
        key = getKey(settings)
        if key == 'c':
            command_string = input('Enter your command: ')
            #print('Command sent:',command_string)
            process_controller.parse_entered_command(command_string)
            #process_controller.send_action_request_to_image_process()
        elif key in speedBindings.keys():

            #print(vels(speed, turn))
            if (status == 14):
                print(msg)
            status = (status + 1) % 15
        else:
            if (key == '\x03'):
                break


        restoreTerminalSettings(settings)





def main():

    rclpy.init()
    minimal_client = MinimalClientAsync() #config the node/service/action client....
    process_controller = Process_Controller(minimal_client) #handle the action client request, mainstain process status
    minimal_client.set_action_client(process_controller)
    _thread.start_new_thread(process_status_controller_test, (process_controller,1))

    rclpy.spin(minimal_client)



if __name__ == '__main__':
    main()













    def send_runjob(self, job_mode):
        action_sever_names= ["image_processor", 
        "data_base"]
        for action_sever_name in action_sever_names:
            self.node.action_clients[action_sever_name].send_task('a')
            self.set_process_status(action_sever_name)
            print(action_sever_name + 'is running')
            while self.process_status != 'idle':
                m = 1
            print(action_sever_name + 'is running')            
        
        if job_mode == 'conti':
            self.node.action_clients["image_processor"].send_task('a')
            self.set_process_status('image_processing')
            print('image_processing....')
            while self.process_status != 'idle':
                m = 1
            print('image_processing done')

            self.node.action_clients["data_base"].send_task("a")      
            self.set_process_status('updating_d')     
            print('data_base updating...')
            while self.process_status != 'idle':
                m = 1
            print('data_base_updating done')

        elif job_mode == 'step':
            self.node.action_clients["data_base"].send_task("a") 

        else:
            print('Unknown command in send_runjob')