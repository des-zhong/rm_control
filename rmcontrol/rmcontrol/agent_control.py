import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robomaster_msgs.action import MoveArm, GripperControl
from robomaster_msgs.msg import GripperState
from ros2_interfaces.msg import Ctrl, State
from std_msgs.msg import ColorRGBA
from ros2_interfaces.srv import AgentStatus
from geometry_msgs.msg import Twist, Vector3, Pose2D, Point, PointStamped
import time
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus

from .utils import *

# arm_pose: high for move, low for catch
arm_pose=[MoveArm.Goal(x=0.15, z=0.06, relative=False), MoveArm.Goal(x=0.17, z=-0.08, relative=False)] # up and down position
gripper=[GripperControl.Goal(target_state=1, power=1.), GripperControl.Goal(target_state=2, power=1.)] # open and close
attacker=0
team_a=[1,3,5]
team_b=[4,2,6]
kick_speed_ep=1.1
kick_speed_s1=0.9
kick_time=0.7
# action_gap=0.05

class AgentControl(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name=name
        self.id=int(self.get_name()[2])
        self.get_logger().info(f"节点已启动：{self.get_name()}!")

        self.goal_pub=self.create_publisher(Twist, f'/RM{self.id}/cmd_vel',1)
        self.state_pub=self.create_publisher(State, f'/RM{self.id}/state_id', 2)
        
        self.arm_sub=self.create_subscription(PointStamped, f'/RM{self.id}/arm_position', self.update_arm, 1)
        self.grip_sub=self.create_subscription(GripperState, f'/RM{self.id}/gripper', self.update_grip, 1)
        self.arm_action=ActionClient(self, MoveArm, f'/RM{self.id}/move_arm')
        self.gripper_action=ActionClient(self, GripperControl, f'/RM{self.id}/gripper')
        self.arm_goal=arm_pose[0]
        self.gripper_goal=gripper[0]
        self.grip_state=0
        self.grip_start=0
        self.manipulator_initialized=self.id not in [1,2,3]
        self.release_start=0
        # time.sleep(10)
        self.arm_done=0
        
        # fut.add_done_callback(self.arm_done_cb)
        self.arm_pos=Point()
        self.arm_position_time=0.0
        self.action_failed=False
        self.arm_tolerance=float(self.declare_parameter('arm_position_tolerance', 0.02).value)
        self.action_timeout=float(self.declare_parameter('manipulator_timeout_sec', 10.0).value)
        self.action_attempts=max(1, int(self.declare_parameter('manipulator_attempts', 5).value))
        self.action_retry_delay=max(0.1, float(self.declare_parameter('manipulator_retry_delay_sec', 1.0).value))
        self.arm_start=0
        self.diagnostic_time=0.0
        self.last_diagnostic_state=None
        self.fault_reason=''


        self.create_subscription(Ctrl, 'ctrl', self.ctrl, 10)
        self.strategy_cli = self.create_client(AgentStatus, 'agent_status')
        self.status=10 
        self.code=0 # 0: stop 1: move 2: kick 3: catch

        self.target=Pose2D()
        self.req_s=AgentStatus.Request()
        self.req_s.id=self.id
        self.req_s.status=self.status

        # set LED color
        if self.id in team_a:
            self.color=ColorRGBA(r=80., g=30., b=30., a=0.5)
        else:
            self.color=ColorRGBA(r=10., g=0., b=200., a=0.5)
        self.create_publisher(ColorRGBA, f'/RM{self.id}/leds/color',5).publish(self.color)
        rclpy.spin_once(self)

    def ctrl(self, msg):
        self.code=msg.code[self.id]
        self.target=msg.pose[self.id]

    def stop(self):
        # self.status=0
        self.goal_pub.publish(Twist(linear=Vector3(x=0., y=0.), angular=Vector3(z=0.)))

    def kick(self):
        global kick_speed
        if self.id in [4,5,6]:
            kick_speed=kick_speed_s1
        else:
            kick_speed=kick_speed_ep 
        self.status=25
        self.sync_status()
        self.get_logger().info(f'RM{self.id} starts to kick')
        # kick
        self.goal_pub.publish(Twist(linear=Vector3(x=kick_speed, y=0.), angular=Vector3(z=0.)))
        time.sleep(kick_time)
        # self.stop()
        # get back
        self.goal_pub.publish(Twist(linear=Vector3(x=-kick_speed, y=0.), angular=Vector3(z=0.)))
        time.sleep(kick_time)
        self.status=10
        self.sync_status()
        self.get_logger().info(f'RM{self.id} finishes kick')
        self.stop()

    def arm_done_cb(self, future):
        goal_handle=future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        """获取结果反馈"""
        result = future.result().result
        self.get_logger().info(f'Result: {result}')



    def gripper_fbcb(self, msg):# msg.feedback.current_state: 1: open 2: closed 0: pause
        cs=msg.feedback.current_state
        # self.get_logger().info(f'current state: {cs}')
        self.grip_state=cs


    def arm_fbcb(self, msg):
        self.get_logger().info(f'arm state: {msg.feedback}')
        # if msg.feedback.progress==1:
        #     self.arm_done=1


    def update_arm(self, msg):
        # if self.id==1:
        #     self.get_logger().info(f'arm state: x {msg.point.x} z {msg.point.z}')
        self.arm_pos=msg.point
        self.arm_position_time=time.monotonic()

    def update_grip(self, msg):
        # self.get_logger().info(f'grip state: {msg}')
        self.grip_state=msg.state


    def check_arm(self):
        # Compare both coordinates with the commanded poses (metres).
        for state, goal in enumerate(arm_pose, start=1):
            if (abs(self.arm_pos.x-goal.x) <= self.arm_tolerance and
                    abs(self.arm_pos.z-goal.z) <= self.arm_tolerance):
                return state # 1: up, 2: down

    def log_diagnostics(self):
        now=time.monotonic()
        state=(self.code, self.status, self.action_failed)
        if state!=self.last_diagnostic_state or now-self.diagnostic_time>=2.0:
            self.diagnostic_time=now
            self.last_diagnostic_state=state
            age=now-self.arm_position_time if self.arm_position_time else float('inf')
            self.get_logger().info(
                f'[CONTROL] code={self.code}, status={self.status}, fault={self.action_failed}, '
                f'arm=({self.arm_pos.x:.3f},{self.arm_pos.z:.3f}), arm_age={age:.2f}s, '
                f'grip={self.grip_state}, requested_velocity='
                f'({self.target.x:.3f},{self.target.y:.3f},{self.target.theta:.3f}), '
                f'reason={self.fault_reason or "none"}'
            )

    def initialize_manipulator(self):
        """Raise idle EP arms; a catcher prepares directly in code 30."""
        if self.manipulator_initialized:
            return True
        if self.code==30:
            # The catcher needs the lower pose, not a successful raise first.
            # Code 30 already waits for lowering and opening before approach.
            self.manipulator_initialized=True
            self.get_logger().info(
                '[INIT] catch requested (code=30): skip startup raise; '
                'prepare lower arm and open gripper directly'
            )
            return True
        self.get_logger().info('[INIT] raising arm and opening gripper before chassis motion')
        if not self.run_manipulator_action(
                self.arm_action, arm_pose[0], 'Startup raise arm', arm_state=1,
                allow_command_changes=True):
            return False
        if not self.run_manipulator_action(
                self.gripper_action, gripper[0], 'Startup open gripper',
                allow_command_changes=True):
            return False
        self.manipulator_initialized=True
        self.status=10
        self.sync_status()
        self.get_logger().info('[INIT] arm raised and gripper open; initialization complete')
        return True

    def run_manipulator_action(self, client, goal, label, arm_state=None,
                               allow_command_changes=False):
        """Retry only rejected or terminal ABORTED goals, without overlapping requests."""
        expected_code=self.code
        for attempt in range(1, self.action_attempts+1):
            self.get_logger().info(
                f'[MANIPULATOR] {label} attempt={attempt}/{self.action_attempts}'
            )
            if self._run_manipulator_once(
                    client, goal, label, arm_state, allow_command_changes):
                return True
            if not self.manipulator_failure_retryable or attempt==self.action_attempts:
                break
            self.get_logger().warning(
                f'[MANIPULATOR] {self.fault_reason}; retry in {self.action_retry_delay:.1f}s'
            )
            until=time.monotonic()+self.action_retry_delay
            while rclpy.ok() and time.monotonic()<until:
                interrupted=(self.code==0 if allow_command_changes else self.code!=expected_code)
                if interrupted:
                    self.fault_reason=f'{label}: command changed during retry delay'
                    break
                rclpy.spin_once(self, timeout_sec=0.05)
            if not rclpy.ok() or (self.code==0 if allow_command_changes else self.code!=expected_code):
                break
        self.stop()
        self.get_logger().error(
            f'[MANIPULATOR] {self.fault_reason}; chassis stopped after {attempt} attempt(s). '
            'Send code 0 to clear fault, or restart the control node.'
        )
        return False

    def arm_already_at_goal(self, goal, allow_command_changes=False):
        """Confirm a no-op using fresh position updates over 0.3 seconds."""
        expected_code=self.code
        started=time.monotonic()
        last_sample=self.arm_position_time
        samples=0
        while rclpy.ok():
            now=time.monotonic()
            interrupted=(self.code==0 if allow_command_changes else self.code!=expected_code)
            if interrupted or now-self.arm_position_time>0.5 or not self.arm_position_time:
                return False
            if (abs(self.arm_pos.x-goal.x)>self.arm_tolerance or
                    abs(self.arm_pos.z-goal.z)>self.arm_tolerance):
                return False
            if self.arm_position_time!=last_sample:
                samples+=1
                last_sample=self.arm_position_time
            if now-started>=0.3 and samples>=3:
                return True
            if now-started>=0.6:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _run_manipulator_once(self, client, goal, label, arm_state=None,
                              allow_command_changes=False):
        """Send once and await the terminal action result; position is diagnostic only."""
        self.stop()
        expected_code=self.code
        started=time.monotonic()
        deadline=started+self.action_timeout
        handle=None
        terminal_result=False
        abandoned=False
        self.manipulator_failure_retryable=False
        stage='server'
        last_log=0.0

        def feedback_cb(msg):
            feedback=msg.feedback
            if arm_state is not None:
                self.get_logger().info(
                    f'[MANIPULATOR] {label} progress={feedback.progress:.3f}'
                )
            else:
                self.get_logger().info(
                    f'[MANIPULATOR] {label} gripper_state={feedback.current_state}'
                )

        def cancel_late_goal(future):
            if not abandoned:
                return
            try:
                late_handle=future.result()
                if late_handle.accepted:
                    late_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'[MANIPULATOR] {label} late response: {exc}')

        def wait_until(predicate):
            nonlocal last_log
            while rclpy.ok() and time.monotonic()<deadline:
                now=time.monotonic()
                interrupted=(self.code==0 if allow_command_changes else self.code!=expected_code)
                if interrupted:
                    raise RuntimeError(f'command changed: {expected_code} -> {self.code}')
                if predicate():
                    return
                if now-last_log>=1.0:
                    last_log=now
                    age=now-self.arm_position_time if self.arm_position_time else float('inf')
                    self.get_logger().info(
                        f'[MANIPULATOR] {label} wait={stage}, elapsed={now-started:.1f}s, '
                        f'arm=({self.arm_pos.x:.3f},{self.arm_pos.z:.3f}), '
                        f'arm_age={age:.2f}s (diagnostic only)'
                    )
                # The chassis was stopped before the action. Only service ROS
                # here, matching a standalone action client while it waits.
                rclpy.spin_once(self, timeout_sec=0.05)
            raise RuntimeError(f'{stage} timeout or ROS shutdown')

        try:
            if arm_state is not None and not goal.relative and self.arm_already_at_goal(
                    goal, allow_command_changes):
                self.action_failed=False
                self.fault_reason=''
                self.get_logger().info(
                    f'[MANIPULATOR] {label} COMPLETE: already at goal, '
                    f'fresh position stable at ({self.arm_pos.x:.3f},{self.arm_pos.z:.3f}); no goal sent'
                )
                return True
            self.get_logger().info(f'[MANIPULATOR] {label} sending ONE goal: {goal}')
            wait_until(client.server_is_ready)
            stage='goal acceptance'
            future=client.send_goal_async(goal, feedback_callback=feedback_cb)
            future.add_done_callback(cancel_late_goal)
            wait_until(future.done)
            handle=future.result()
            if not handle.accepted:
                self.manipulator_failure_retryable=True
                raise RuntimeError('goal rejected')
            self.get_logger().info(f'[MANIPULATOR] {label} goal accepted')
            stage='action result'
            result=handle.get_result_async()
            wait_until(result.done)
            result_status=result.result().status
            terminal_result=True
            status_names={
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                GoalStatus.STATUS_ABORTED: 'ABORTED',
                GoalStatus.STATUS_CANCELED: 'CANCELED',
            }
            status_name=status_names.get(result_status, str(result_status))
            self.get_logger().info(
                f'[MANIPULATOR] {label} result={status_name} ({result_status})'
            )
            if result_status!=GoalStatus.STATUS_SUCCEEDED:
                self.manipulator_failure_retryable=(result_status==GoalStatus.STATUS_ABORTED)
                raise RuntimeError(f'action result={status_name} ({result_status})')
            # Match the CLI: success does not require separate position telemetry.
            if arm_state is None:
                self.grip_state=goal.target_state
            self.action_failed=False
            self.fault_reason=''
            self.get_logger().info(f'[MANIPULATOR] {label} COMPLETE')
            return True
        except Exception as exc:
            abandoned=True
            self.action_failed=True
            self.fault_reason=f'{label}: {exc}'
            if handle is not None and handle.accepted and not terminal_result:
                try:
                    handle.cancel_goal_async()
                except Exception as cancel_exc:
                    self.get_logger().warning(f'[MANIPULATOR] cancel failed: {cancel_exc}')
            self.stop()
            self.get_logger().warning(f'[MANIPULATOR] {self.fault_reason}')
            return False

    def sync_status(self):
        self.req_s.status=self.status
        self.strategy_cli.call_async(self.req_s)
        self.state_pub.publish(State(state=self.status))


    @staticmethod
    def pose2d_to_twist(pose: Pose2D):
        pose=clip_pose2d(pose, [-0.5,-0.5], [0.5,0.5])
        return Twist(linear=Vector3(x=pose.x, y=pose.y), angular=Vector3(z=pose.theta))



def main(args=None):
    rclpy.init(args=args)
    node = AgentControl(name='RM1')
    time.sleep(1)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        node.log_diagnostics()
        # node.req_s.status=node.status
        # node.strategy_cli.call_async(node.req_s)
        # node.state_pub.publish(State(state=node.status))
        # node.get_logger().info(f'----code:{node.code}, status: {node.status}')
        if node.code==0:
            node.stop()
            node.action_failed=False
            node.fault_reason=''
            node.status=10
        elif node.action_failed:
            node.stop()
            continue
        elif not node.manipulator_initialized:
            node.initialize_manipulator()
            continue
        elif node.code==10:
            # node.status=10
            node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==20:
            node.status=20
            node.get_logger().info(f'RM{node.id} kickign, status: {node.status}')
            node.kick()
        elif node.code==30:
            # node.get_logger().info(f'----code:{node.code}, status: {node.status}')
            if node.status!=30:
                node.release_start=0
                node.status=29
                node.sync_status()
                if not node.run_manipulator_action(node.arm_action, arm_pose[1], 'Lower arm', arm_state=2):
                    continue
                if not node.run_manipulator_action(node.gripper_action, gripper[0], 'Open gripper'):
                    continue
                # arm down and grip open
                node.status=30
                node.sync_status()
                node.get_logger().info('[CATCH] arm down and gripper open; approach enabled, status=30')
            else:
                node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==32: # ready to grip
            node.get_logger().info(f'----ready to grip: {node.code}')
            if not node.run_manipulator_action(node.gripper_action, gripper[1], 'Close gripper'):
                continue
            node.get_logger().info(f'RM{node.id} closed grip')
            node.status=32
            node.sync_status()
            node.code=33
        elif node.code==33: # closed grip
            node.stop()
        elif node.code==34: # got ball
            # node.status=32
            # node.get_logger().info(f'----: {node.code}')
            node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==35: # release ball
            if node.status!=35:
                if not node.run_manipulator_action(node.gripper_action, gripper[0], 'Release ball'):
                    continue
                node.get_logger().info(f'RM{node.id} ball released')
                node.status=35
                node.sync_status()
            node.stop()
        elif node.code in [36, 37]: # move to position
            # node.get_logger().info(f'code=36')
            if node.status not in [36,37]:
                node.get_logger().info(f'RM{node.id} last step of carrying')
                if not node.run_manipulator_action(node.arm_action, arm_pose[0], 'Raise arm', arm_state=1):
                    continue
                node.status=36
                node.sync_status()
                time.sleep(0.2)
                
            node.goal_pub.publish(node.pose2d_to_twist(node.target))
        elif node.code==39:
            node.get_logger().info(f'RM{node.id} finished carrying')
            node.status=10
            node.sync_status()
            

        


        
    node.destroy_node()
    rclpy.shutdown()

