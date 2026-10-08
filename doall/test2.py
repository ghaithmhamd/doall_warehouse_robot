#!/usr/bin/env python3

#407.7 incs per revolution
#ros2 run doall arduino_controller --ros-args -p mode:='rpm'
#ros2 param set /arduino_controller rpmR 0
import rclpy
from rclpy.node import Node
import serial
import struct
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import Int32
from interfaces.action import Gripper
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
import tf2_ros
from rclpy.executors import MultiThreadedExecutor

# Angles offset
ROLL_ZERO  = 84
PITCH_ZERO = 73
GRIPPER_ZERO = 103
WHEEL_BASE = 0.250

class ArduinoControllerNode(Node):
    def __init__(self):
        super().__init__('arduino_controller')
        # Declare parameters with default values
        self.declare_parameter('mode', 'cmd_vel') # options: 'cmd_vel' | 'rpm'
        self.declare_parameter('port', '/dev/arduino_nano')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('v', 0)
        self.declare_parameter('w', 0)
        self.declare_parameter('roll_angle', ROLL_ZERO)
        self.declare_parameter('pitch_angle', PITCH_ZERO)
        #self.declare_parameter('gripp_angle', GRIPPER_ZERO)
        self.declare_parameter('close_gripper', 0)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        mode = self.get_parameter('mode').value
        param_v = self.get_parameter('v').value
        param_w = self.get_parameter('w').value
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.roll_angle = self.get_parameter('roll_angle').value
        self.pitch_angle = self.get_parameter('pitch_angle').value
        #self.gripp_angle = self.get_parameter('gripp_angle').value
        self.close_gripper = self.get_parameter('close_gripper').value
        publish_rate = self.get_parameter('publish_rate').value
        self.gripper_angle = GRIPPER_ZERO
        self.gripper_closed = False
        self.cmd_vel_v = 0.0
        self.cmd_vel_w = 0.0
        self.msg_count = 0
        # odom and tf
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time_odom = self.get_clock().now()
        
        # Initialize serial connection
        try:
            self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=0.01)
            time.sleep(0.1)
            self.get_logger().info(f'Connected to Arduino on {port} at {baudrate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise
        
        # Timer for sending commands and reading joints
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Arduino Controller Node Started')
        self.get_logger().info(f'Default values: v={param_v}, w={param_w}, '
                              f'roll={self.roll_angle}, pitch={self.pitch_angle}, '
                              f'gripp_state={self.close_gripper}, '
                              f'MODE={mode} ')
        
        # cmd_vel subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Odometry publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # gripp_roll
        self.roll_sub = self.create_subscription(
            Int32, '/cmd_gripper_roll', self.roll_callback, 10)

        # gripp_pitch
        self.pitch_sub = self.create_subscription(
            Int32, '/cmd_gripper_pitch', self.pitch_callback, 10)
        
        # gripper_action
        self._action_server = ActionServer(
            self,
            Gripper,
            'state_gripper_action',
            execute_callback=self.gripper_execute_cb,
            goal_callback=self.gripper_goal_cb,
            cancel_callback=self.gripper_cancel_cb,
        )

        # joint_states publisher
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
            
        self.last_time = self.get_clock().now()


    # cmd_vel_callback
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_v = msg.linear.x
        self.cmd_vel_w = msg.angular.z
        #v_l = v - (w * WHEEL_BASE / 2.0)
        #v_r = v + (w * WHEEL_BASE / 2.0)
        #self.cmd_vel_rpmL = int((v_l / (2 * 3.1416 * self.wheel_radius)) * 60)
        #self.cmd_vel_rpmR = int((v_r / (2 * 3.1416 * self.wheel_radius)) * 60)


    # roll_callback
    def roll_callback(self, msg: Int32):
        self.roll_angle = max(0, min(180, msg.data))   
        self.get_logger().info(f'Roll angle set to {self.roll_angle}°')


    # pitch_callback
    def pitch_callback(self, msg: Int32):
        self.pitch_angle = max(0, min(180, msg.data))
        self.get_logger().info(f'Pitch angle set to {self.pitch_angle}°')


    # gripper_action callbacks
    def gripper_goal_cb(self, goal_request):
        self.get_logger().info(
            f'Gripper action goal received: close={goal_request.close}')
        return GoalResponse.ACCEPT


    def gripper_cancel_cb(self, goal_handle):
        self.get_logger().info('Gripper action cancel requested')
        return CancelResponse.ACCEPT
    

    # gripper execute callbacks
    def gripper_execute_cb(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")
        self.close_gripper = goal_handle.request.close
        self.get_logger().info(
            f"START EXEC → closed={self.gripper_closed}, "
            f"target={self.close_gripper}"
        )
        start_time = self.get_clock().now()
        while rclpy.ok():
            # Cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Gripper.Result()
                result.success = False
                result.final_angle = self.gripper_angle
                return result
            # Success 
            if self.close_gripper:
                if self.gripper_closed:
                    goal_handle.succeed()
                    result = Gripper.Result()
                    result.success = True
                    result.final_angle = self.gripper_angle
                    return result
            else:
                if not self.gripper_closed:
                    goal_handle.succeed()
                    result = Gripper.Result()
                    result.success = True
                    result.final_angle = self.gripper_angle
                    return result

            # Timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed > 10.0:
                self.get_logger().warn("Gripper timeout → aborting")
                goal_handle.abort()
                result = Gripper.Result()
                result.success = False
                result.final_angle = self.gripper_angle
                return result
            # Feedback
            feedback = Gripper.Feedback()
            feedback.current_angle = self.gripper_angle
            feedback.gripper_closed = self.gripper_closed
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
 
    
    #def send_command(self, v, w, roll, pitch, gripp, close_gripper):
    def send_command(self, v, w, roll, pitch, close_gripper):
        """Send command packet to Arduino"""
        try:
            # Pack data: S rpmL rpmR RollAngle PitchAngle GrippAngle CloseGripper E
            #packet = struct.pack('<BffBBBBB', ord('S'),float(v), float(w), roll, pitch, gripp, close_gripper, ord('E'))
            packet = struct.pack('<BffBBBB', ord('S'),float(v), float(w), roll, pitch, close_gripper, ord('E'))
            self.arduino.write(packet)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            return False
    

    '''
    def read_arduino_joint_state(self):
        try:
            if self.arduino.in_waiting >= 20:
                if self.arduino.read(1) == b'F':
                    payload = self.arduino.read(19)
                    if len(payload) != 19:
                        return None
                    posL, posR, velL, velR, gripper_angle, gripper_closed, end = struct.unpack('<ffffBBB', payload)
                    if end == ord('E'):
                        return posL, posR, velL, velR, gripper_angle, gripper_closed
        except Exception as e:
            self.get_logger().error(f'Failed to read joints: {e}')
        return None
    '''
    def read_arduino_joint_state(self):
        try:
            while self.arduino.in_waiting >= 20:
                byte = self.arduino.read(1)
                if byte != b'F':
                    continue  
                payload = self.arduino.read(18)
                if len(payload) != 18:
                    return None
                posL, posR, velL, velR, gripper_angle, gripper_closed = \
                    struct.unpack('<ffffBB', payload)
                end = self.arduino.read(1)
                if end != b'E':
                    continue 
                return posL, posR, velL, velR, gripper_angle, bool(gripper_closed)
        except Exception as e:
            self.get_logger().error(f'Failed to read joints: {e}')
        return None

    
    def timer_callback(self):
        """Main loop - send commands and read joints"""
        # Get current parameter values (allows dynamic reconfiguration)
        #self.roll_angle = self.get_parameter('roll_angle').value
        #self.pitch_angle = self.get_parameter('pitch_angle').value
        #self.gripp_angle = self.get_parameter('gripp_angle').value
        #self.close_gripper = self.get_parameter('close_gripper').value
        mode = self.get_parameter('mode').value
        if mode == 'cmd_vel':
            v = self.cmd_vel_v
            w = self.cmd_vel_w
        elif mode == 'rpm':
            v = self.get_parameter('v').value
            w = self.get_parameter('w').value
        else:
            rpmL = 0.0
            rpmR = 0.0
        
        # Send command to Arduino
        #self.send_command(v, w, self.roll_angle, self.pitch_angle, self.gripp_angle, self.close_gripper)
        self.send_command(v, w, self.roll_angle, self.pitch_angle, self.close_gripper)
        # Read joints feedback
        joint_data = self.read_arduino_joint_state()
        
        if joint_data is not None:
            posL, posR, velL, velR, gripper_angle, gripper_closed = joint_data
            self.gripper_angle = gripper_angle
            self.gripper_closed = gripper_closed

            # Publish joint states
            now = self.get_clock().now()
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = ['wheel2_joint', 
                        'wheel1_joint',
                        'gripper_roll_joint',
                        'gripper_pitch_joint',
                        'gripper_left_upper_joint',
            ]
            js.position = [
                posL,
                posR,
                (self.roll_angle - ROLL_ZERO) * 3.1416 / 180,  
                (self.pitch_angle - PITCH_ZERO) * 3.1416 / 180,
                (self.gripper_angle - GRIPPER_ZERO) * 3.1416 / 180 + 1.5708,
            ]
            js.velocity = [velL, velR, 0.0, 0.0, 0.0]
            js.effort = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.joint_pub.publish(js)

            # Update odometry
            now = self.get_clock().now()
            dt = (now - self.last_time_odom).nanoseconds * 1e-9
            self.last_time_odom = now
            v = (velL + velR) / 2.0 # m/s
            omega = (velR - velL) / WHEEL_BASE # rad/s
            self.x += v * dt * math.cos(self.theta)
            self.y += v * dt * math.sin(self.theta)
            self.theta += omega * dt

            # Publish TF
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

            # Publish Odometry message
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = omega
            self.odom_pub.publish(odom)
   
            # Log occasionally (every 10 messages = . second at .Hz)
            self.msg_count += 1
            if self.msg_count % 1 == 0:
                # Determine gripper status
                if gripper_closed and not self.close_gripper:
                    gripper_status = "OPENING"
                elif not gripper_closed and self.close_gripper:
                    gripper_status = "CLOSING"
                elif gripper_closed:
                    gripper_status = "CLOSED"
                else:
                    gripper_status = "OPEN"
                self.get_logger().info(
                    f'measures: - posL: {posL:.4f}, posR: {posR:.4f}, velL: {velL:.4f}, velR: {velR:.4f}, gripper: {gripper_status}, with angle: {gripper_angle}'
                )    


    def destroy_node(self):
        """Cleanup when node is destroyed"""
        # Stop motors before closing
        self.get_logger().info('Stopping motors...')
        #self.send_command(0, 0, ROLL_ZERO, PITCH_ZERO, GRIPPER_ZERO, 0)
        self.send_command(0, 0, ROLL_ZERO, PITCH_ZERO, 0)
        time.sleep(0.1)
        # Close serial connection
        if hasattr(self, 'arduino') and self.arduino.is_open:
            self.arduino.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoControllerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()