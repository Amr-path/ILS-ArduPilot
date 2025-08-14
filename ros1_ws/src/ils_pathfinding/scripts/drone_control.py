#!/usr/bin/env python3
import math, time, os
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from ils_pathfinding.ils_core import GridMap, astar, ils_mask

class Controller:
    def __init__(self):
        self.state = State()
        self.pose = None
        self.res = rospy.get_param("~resolution", 1.0)
        self.map_path = rospy.get_param("~map_path", "")
        self.goal_x = int(rospy.get_param("~goal_x", 80))
        self.goal_y = int(rospy.get_param("~goal_y", 80))
        self.w0 = float(rospy.get_param("~w0", 0.03))
        self.wstep = float(rospy.get_param("~wstep", 0.02))
        self.wmax = float(rospy.get_param("~wmax", 0.3))

        rospy.Subscriber("/mavros/state", State, self._on_state, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._on_pose, queue_size=10)
        self.pub_sp = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=20)
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")
        self.arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def _on_state(self, msg): self.state = msg
    def _on_pose(self, msg): self.pose = msg

    def wait_fc(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.state.connected:
            rate.sleep()

    def set_guided_arm(self):
        self.mode_srv(0, "GUIDED")
        time.sleep(0.5)
        self.arm_srv(True)
        time.sleep(0.5)

    def current_cell(self):
        if self.pose is None: return None
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        return (int(round(x/self.res)), int(round(y/self.res)))

    def send_sp(self, x, y, z=5.0):
        m = PoseStamped()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        self.pub_sp.publish(m)

    def run(self):
        if not self.map_path or not os.path.exists(self.map_path):
            rospy.logerr("map_path is missing")
            return
        self.wait_fc()
        self.set_guided_arm()

        gm = GridMap.from_png(self.map_path)
        goal = (self.goal_x, self.goal_y)

        # send a few setpoints before mode switching loop
        for _ in range(50):
            self.send_sp(0.0, 0.0, 5.0)
            rospy.sleep(0.02)

        # compute start from current position
        rate = rospy.Rate(20)
        start = None
        while start is None and not rospy.is_shutdown():
            start = self.current_cell()
            rate.sleep()

        # ILS A*
        w = self.w0
        path = []
        while w <= self.wmax and not path and not rospy.is_shutdown():
            mask = ils_mask(gm, start, goal, w)
            path = astar(gm, start, goal, mask)
            w += self.wstep

        if not path:
            rospy.logerr("No path found")
            return

        # follow
        idx = 0
        while not rospy.is_shutdown() and idx < len(path):
            cx, cy = path[idx]
            tx, ty = cx*self.res, cy*self.res
            self.send_sp(tx, ty, 5.0)
            if self.pose is not None:
                px = self.pose.pose.position.x
                py = self.pose.pose.position.y
                if math.hypot(tx-px, ty-py) < max(0.5, self.res*0.8):
                    idx += 1
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ils_controller")
    Controller().run()
