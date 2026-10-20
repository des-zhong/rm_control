"""Exercise the real action workflow with simulated clients, without ROS hardware."""

from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus

from rmcontrol import agent_control as control


class Future:
    def __init__(self, value, done=True):
        self.value = value
        self.ready = done
        self.callbacks = []

    def done(self):
        return self.ready

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.ready:
            callback(self)

    def complete(self):
        self.ready = True
        for callback in self.callbacks:
            callback(self)


class GoalHandle:
    def __init__(self, status=GoalStatus.STATUS_SUCCEEDED, accepted=True,
                 result_pending=False):
        self.accepted = accepted
        self.cancelled = False
        self.result_future = Future(SimpleNamespace(status=status), not result_pending)

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancelled = True


class Client:
    def __init__(self, handles, ready=True, response_pending=False):
        self.handles = handles
        self.ready = ready
        self.response_pending = response_pending
        self.sent = []
        self.responses = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback):
        # A second request must follow a rejection or a terminal result.
        if self.sent:
            previous = self.handles[len(self.sent) - 1]
            assert not previous.accepted or previous.result_future.done()
        handle = self.handles[len(self.sent)]
        self.sent.append(goal)
        if hasattr(goal, 'target_state'):
            feedback = SimpleNamespace(current_state=goal.target_state)
        else:
            feedback = SimpleNamespace(progress=0.98)
        feedback_callback(SimpleNamespace(feedback=feedback))
        response = Future(handle, not self.response_pending)
        self.responses.append(response)
        return response


class Harness:
    run_manipulator_action = control.AgentControl.run_manipulator_action
    _run_manipulator_once = control.AgentControl._run_manipulator_once
    arm_already_at_goal = control.AgentControl.arm_already_at_goal
    initialize_manipulator = control.AgentControl.initialize_manipulator

    def __init__(self):
        self.code = 30
        self.status = 10
        self.action_timeout = 1.0
        self.action_attempts = 5
        self.action_retry_delay = 1.0
        self.action_failed = False
        self.fault_reason = ''
        self.arm_position_time = 0.0
        self.arm_tolerance = 0.02
        self.arm_pos = SimpleNamespace(x=0.0, z=0.0)
        self.grip_state = 0
        self.manipulator_initialized = False
        self.logs = []
        self.stops = 0
        self.status_updates = []

    def get_logger(self):
        return SimpleNamespace(info=self.logs.append, warning=self.logs.append,
                               error=self.logs.append)

    def stop(self):
        self.stops += 1

    def sync_status(self):
        self.status_updates.append(self.status)


@pytest.fixture
def simulation(monkeypatch):
    clock = SimpleNamespace(now=10.0, on_spin=lambda node: None)

    def spin(node, timeout_sec):
        clock.now += timeout_sec
        clock.on_spin(node)

    monkeypatch.setattr(control, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    monkeypatch.setattr(control.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(control.rclpy, 'spin_once', spin)
    return clock


def lower(node, client, **kwargs):
    return node.run_manipulator_action(client, control.arm_pose[1], 'Lower arm',
                                       arm_state=2, **kwargs)


def test_success_uses_action_result_without_position_gate(simulation):
    node = Harness()
    client = Client([GoalHandle()])
    assert lower(node, client)
    assert len(client.sent) == 1
    assert not node.action_failed
    assert any('progress=0.980' in line for line in node.logs)


@pytest.mark.parametrize('first', [GoalHandle(status=GoalStatus.STATUS_ABORTED),
                                 GoalHandle(accepted=False)])
def test_terminal_failure_then_success_retries_serially(simulation, first):
    node = Harness()
    client = Client([first, GoalHandle()])
    assert lower(node, client)
    assert len(client.sent) == 2
    assert simulation.now >= 11.0
    assert not first.cancelled
    assert not node.action_failed
    assert node.fault_reason == ''


def test_retries_stop_at_limit(simulation):
    node = Harness()
    client = Client([GoalHandle(status=GoalStatus.STATUS_ABORTED) for _ in range(5)])
    assert not lower(node, client)
    assert len(client.sent) == 5
    assert node.action_failed
    assert 'ABORTED' in node.fault_reason
    assert node.stops > 0


def test_canceled_action_is_not_retried(simulation):
    node = Harness()
    client = Client([GoalHandle(status=GoalStatus.STATUS_CANCELED)])
    assert not lower(node, client)
    assert len(client.sent) == 1


def test_result_timeout_cancels_without_retry(simulation):
    node = Harness()
    handle = GoalHandle(result_pending=True)
    client = Client([handle])
    assert not lower(node, client)
    assert len(client.sent) == 1
    assert handle.cancelled


def test_late_acceptance_is_canceled_without_retry(simulation):
    node = Harness()
    handle = GoalHandle()
    client = Client([handle], response_pending=True)
    assert not lower(node, client)
    assert len(client.sent) == 1
    client.responses[0].complete()
    assert handle.cancelled


def test_missing_server_sends_no_goal(simulation):
    node = Harness()
    client = Client([], ready=False)
    assert not lower(node, client)
    assert not client.sent


def test_stop_during_retry_delay_prevents_next_request(simulation):
    node = Harness()
    client = Client([GoalHandle(status=GoalStatus.STATUS_ABORTED)])
    simulation.on_spin = lambda node: setattr(node, 'code', 0)
    assert not lower(node, client)
    assert len(client.sent) == 1


def test_stop_during_action_cancels_without_retry(simulation):
    node = Harness()
    handle = GoalHandle(result_pending=True)
    client = Client([handle])
    simulation.on_spin = lambda node: setattr(node, 'code', 0)
    assert not lower(node, client)
    assert handle.cancelled
    assert len(client.sent) == 1


def test_startup_raises_then_opens_before_marking_initialized(simulation):
    node = Harness()
    node.code = 10
    node.arm_action = Client([GoalHandle(status=GoalStatus.STATUS_ABORTED), GoalHandle()])
    node.gripper_action = Client([GoalHandle()])
    # Strategy updates during initialization must not interrupt a startup goal.
    simulation.on_spin = lambda node: setattr(node, 'code', 10)
    assert node.initialize_manipulator()
    assert node.manipulator_initialized
    assert node.status_updates == [10]
    assert all(goal is control.arm_pose[0] for goal in node.arm_action.sent)
    assert node.gripper_action.sent == [control.gripper[0]]
    # Later cycles must not initialize again.
    assert node.initialize_manipulator()
    assert len(node.arm_action.sent) == 2


def test_failed_startup_does_not_mark_initialized_or_open_gripper(simulation):
    node = Harness()
    node.code = 10
    node.arm_action = Client([GoalHandle(status=GoalStatus.STATUS_ABORTED) for _ in range(5)])
    node.gripper_action = Client([])
    assert not node.initialize_manipulator()
    assert not node.manipulator_initialized
    assert node.action_failed
    assert not node.gripper_action.sent
    assert not node.status_updates


def test_startup_still_honors_stop_command(simulation):
    node = Harness()
    node.code = 10
    node.arm_action = Client([GoalHandle(result_pending=True)])
    node.gripper_action = Client([])
    simulation.on_spin = lambda node: setattr(node, 'code', 0)
    assert not node.initialize_manipulator()
    assert node.arm_action.handles[0].cancelled
    assert not node.manipulator_initialized
    assert not node.gripper_action.sent


def test_robots_without_manipulator_skip_initialization(simulation):
    node = Harness()
    node.manipulator_initialized = True
    assert node.initialize_manipulator()
    assert not node.logs


def test_catcher_goes_directly_to_verified_lower_action(simulation):
    node = Harness()
    # Under the README order strategy has already assigned code 30 when
    # agent_control starts. A failed/unavailable raise must not gate lowering.
    node.arm_action = Client([GoalHandle()])
    node.gripper_action = Client([GoalHandle()])
    assert node.initialize_manipulator()
    assert node.manipulator_initialized
    assert not node.arm_action.sent
    assert not node.gripper_action.sent
    assert not node.status_updates  # Do not advertise catch-ready prematurely.
    assert lower(node, node.arm_action)
    assert node.run_manipulator_action(node.gripper_action, control.gripper[0], 'Open gripper')
    assert node.arm_action.sent == [control.arm_pose[1]]
    assert node.arm_action.sent[0].x == pytest.approx(0.17)
    assert node.arm_action.sent[0].z == pytest.approx(-0.08)
    assert not node.arm_action.sent[0].relative
    assert node.grip_state == 1


def test_waiting_client_stops_chassis_once_not_on_every_spin(simulation):
    node = Harness()
    handle = GoalHandle(result_pending=True)
    client = Client([handle])

    def complete_after_delay(node):
        if simulation.now >= 10.5:
            handle.result_future.complete()

    simulation.on_spin = complete_after_delay
    assert lower(node, client)
    assert node.stops == 1
    assert len(client.sent) == 1


@pytest.mark.parametrize('x,z', [(0.151, 0.058), (0.155, 0.068)])
def test_already_raised_arm_uses_fresh_stable_position(simulation, x, z):
    node = Harness()
    node.arm_pos = SimpleNamespace(x=x, z=z)
    node.arm_position_time = simulation.now
    simulation.on_spin = lambda node: setattr(node, 'arm_position_time', simulation.now)
    client = Client([])
    assert node.run_manipulator_action(client, control.arm_pose[0], 'Raise arm', arm_state=1)
    assert not client.sent
    assert simulation.now >= 10.3
    assert any('already at goal' in line for line in node.logs)


def test_stale_target_position_does_not_skip_action(simulation):
    node = Harness()
    node.arm_pos = SimpleNamespace(x=0.151, z=0.058)
    node.arm_position_time = simulation.now - 30.0
    client = Client([GoalHandle()])
    assert node.run_manipulator_action(client, control.arm_pose[0], 'Raise arm', arm_state=1)
    assert len(client.sent) == 1


def test_arm_away_from_goal_still_requires_action(simulation):
    node = Harness()
    node.arm_pos = SimpleNamespace(x=0.070, z=0.052)
    node.arm_position_time = simulation.now
    client = Client([GoalHandle(status=GoalStatus.STATUS_CANCELED)])
    assert not node.run_manipulator_action(client, control.arm_pose[0], 'Raise arm', arm_state=1)
    assert len(client.sent) == 1


def test_one_fresh_sample_is_not_enough_to_skip_action(simulation):
    node = Harness()
    node.arm_pos = SimpleNamespace(x=0.151, z=0.058)
    node.arm_position_time = simulation.now
    client = Client([GoalHandle()])
    assert node.run_manipulator_action(client, control.arm_pose[0], 'Raise arm', arm_state=1)
    assert len(client.sent) == 1


def test_position_drifting_away_does_not_skip_action(simulation):
    node = Harness()
    node.arm_pos = SimpleNamespace(x=0.151, z=0.058)
    node.arm_position_time = simulation.now

    def move_away(node):
        node.arm_position_time = simulation.now
        node.arm_pos.x = 0.070

    simulation.on_spin = move_away
    client = Client([GoalHandle()])
    assert node.run_manipulator_action(client, control.arm_pose[0], 'Raise arm', arm_state=1)
    assert len(client.sent) == 1
