import rospy
from sauvc_msgs.msg import MotionData
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


class PIDController(object):
    """A class that represent the PID Controller."""

    def __init__(self, stabilised_speed_publisher):
        """Initialise the controller
        Args:
            stabilised_speed_publisher (rospy.Publisher): a ROS publisher for motor stabilised speed
        """
        rospy.Subscriber("/auv/motion", MotionData, self._update_motion_data)
        rospy.Subscriber("/filter/quaternion", QuaternionStamped, self._get_quaternion_data)
        self._stabilised_speed_publisher = stabilised_speed_publisher  # type: type(rospy.Publisher)
        self._auv_motion = "stop"  # type: str
        self._P = 25  # type: float
        self._thrusters_stabilised_speed = {
            "1": int,
            "2": int,
            "3": int,
            "4": int,
            "5": int,
            "6": int,
            "7": int,
            "8": int
        }  # type: dict
        self._thrusters_actual_speed = {
            "1": int,
            "2": int,
            "3": int,
            "4": int,
            "5": int,
            "6": int,
            "7": int,
            "8": int
        }  # type: dict
        self._actual_euler = {"pitch": float, "roll": float, "yaw": float}  # type: dict
        self._target_euler = {"pitch": float, "roll": float, "yaw": float}  # type: dict

    def _update_motion_data(self, msg):
        """Update the motion data."""
        if self._auv_motion != msg.motion:
            self._target_euler["pitch"] = self._actual_euler["pitch"]
            self._target_euler["roll"] = self._actual_euler["roll"]
            self._target_euler["yaw"] = self._actual_euler["yaw"]
            self._auv_motion = msg.motion
        self._thrusters_actual_speed["1"] = msg.thrusters_speed.thruster_id1_speed
        self._thrusters_actual_speed["2"] = msg.thrusters_speed.thruster_id2_speed
        self._thrusters_actual_speed["3"] = msg.thrusters_speed.thruster_id3_speed
        self._thrusters_actual_speed["4"] = msg.thrusters_speed.thruster_id4_speed
        self._thrusters_actual_speed["5"] = msg.thrusters_speed.thruster_id5_speed
        self._thrusters_actual_speed["6"] = msg.thrusters_speed.thruster_id6_speed
        self._thrusters_actual_speed["7"] = msg.thrusters_speed.thruster_id7_speed
        self._thrusters_actual_speed["8"] = msg.thrusters_speed.thruster_id8_speed

    def _get_quaternion_data(self, msg):
        """Get IMU quaternion data."""
        quat_list = [msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z]
        (self._actual_euler["pitch"], self._actual_euler["roll"], self._actual_euler["yaw"]) \
            = euler_from_quaternion(quat_list)

    def _compute_movement_error(self, motion):
        """Compute the forward movement error's magnitude and direction. ccw deviation from actual: +ve; cw: -ve"""
        error = {"pitch": 0, "roll": 0, "yaw": 0}
        if (motion == "forward" or motion == "backward"):
            error["yaw"] = self._actual_euler["yaw"] - self._target_euler["yaw"]
        if (motion == "submerge" or motion == "surface"):
            error["pitch"] = self._actual_euler["pitch"] - self._target_euler["pitch"]
            error["roll"] = self._actual_euler["roll"] - self._target_euler["roll"]
        for key in error:
            if (error[key] > math.pi):
                    error[key] -= 2 * math.pi
            if (error[key] < math.pi):
                    error[key] += 2 * math.pi
        return error

    def _compute_stabilised_speed(self, thruster_id, error):
        """Compute the stabilised speed from the controller."""
        actual_error = 0
        #yaw: thruster 1-4
        if error["yaw"] > 0:
            #Forward: ccw/left deviation. To correct, speed up 2&4 and speed down 1&3
            if (thruster_id == "1" or thruster_id == "3"):
                actual_error = -1*error["yaw"]
            else:
                actual_error = error["yaw"]
        else if error["yaw"] < 0:
            #Forward: cw/right deviation. To correct, speed up 1&3 and speed down 2&4
            if (thruster_id == "2" or thruster_id == "4"):
                actual_error = -1*error["yaw"]
            else:
                actual_error = error["yaw"]

        #pitch: thruster 5-8
        if error["pitch"] > 0:
            #Surface/submerge: forward deviation. To correct, speed up 5&6 and speed down 7&8
            if (thruster_id == "7" or thruster_id == "8"):
                actual_error += -1*error["pitch"]
            else:
                actual_error += error["pitch"]
        else if error["pitch"] < 0:
            #Surface/submerge: forward deviation. To correct, speed up 7&8 and speed down 5&6
            if (thruster_id == "5" or thruster_id == "6"):
                actual_error += -1*error["pitch"]
            else:
                actual_error += error["pitch"]

        #roll: thruster 5-8
        if error["roll"] > 0:
            #Surface/submerge: right roll deviation. To correct, speed up 5&7 and speed down 6&8
            if (thruster_id == "6" or thruster_id == "8"):
                actual_error += -1*error["roll"]
            else:
                actual_error += error["roll"]
        else if error["roll"] < 0:
            #Surface/submerge: left roll deviation. To correct, speed up 6&8 and speed down 5&7
            if (thruster_id == "5" or thruster_id == "7"):
                actual_error += -1*error["roll"]
            else:
                actual_error += error["roll"]


        return int(self._thrusters_actual_speed[thruster_id] + self._P * actual_error)

    def _update_stabilised_speed(self):
        """Update the stabilised speed."""
        for key in _thrusters_actual_speed:
            self._thrusters_stabilised_speed[key] = self._thrusters_actual_speed[key]

        error = self._compute_movement_error(self._auv_motion)
        for key in _thrusters_stabilised_speed:
            self._thrusters_stabilised_speed[key] = self._compute_stabilised_speed(key, error)
        if self._auv_motion == "forward":
            error = self._compute_movement_error(self._auv_motion)
            self._thrusters_stabilised_speed["1"] = self._compute_stabilised_speed("1", error)
            self._thrusters_stabilised_speed["2"] = self._compute_stabilised_speed("2", error)


    def publish_stabilised_speed(self):
        """Publish the stabilised motor speed."""
        try:
            self._update_stabilised_speed()
        except TypeError:
            pass
        else:
            try:
                self._stabilised_speed_publisher.publish(
                    self._thrusters_stabilised_speed["1"],
                    self._thrusters_stabilised_speed["2"],
                    self._thrusters_stabilised_speed["3"],
                    self._thrusters_stabilised_speed["4"],
                    self._thrusters_stabilised_speed["5"],
                    self._thrusters_stabilised_speed["6"],
                    self._thrusters_stabilised_speed["7"],
                    self._thrusters_stabilised_speed["8"]
                )
            except rospy.ROSSerializationException:
                pass

