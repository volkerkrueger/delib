#!/usr/bin/env python

# Copyright 2024 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""FlexBE State to command PyRoboSim robot to plan a path to target location."""

from action_msgs.msg import GoalStatus

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from pyrobosim_msgs.action import FollowPath
from pyrobosim_msgs.msg import ExecutionResult, Path


class FollowPathState(EventState):
    """
    FlexBE state to request following a path for PyRoboSim robot.

    Elements defined here for UI
    Parameters
    -- action_topic        Action topic name (default= 'robot/follow_path')
    -- server_timeout      Wait for action server timeout in seconds (default = 5s)

    Outputs
    <= done                Successfully reached the goal
    <= failed              Failed to reach goal

    User data
    ># path       Path     Desired path to follow
    #> msg        string   Result message
    """

    def __init__(self, action_topic='robot/follow_path',
                 server_timeout=2.0):
        # See example_state.py for basic explanations.
        super().__init__(outcomes=['done', 'failed'],
                         input_keys=['path'],
                         output_keys=['msg'])

        self._goal = None

        self._topic = action_topic
        self._server_timeout_sec = server_timeout

        self._client = ProxyActionClient({self._topic: FollowPath},
                                         wait_duration=0.0)  # no need to wait here, we'll check on_enter

        self._return = None  # Retain return value in case the outcome is blocked by operator

    def execute(self, userdata):
        """
        Call periodically while this state is active.

        Check if the action has been finished and evaluate the result.
        """
        if self._return is not None:
            # Return prior outcome in case transition is blocked by autonomy level
            return self._return

        status = self._client.get_status(self._topic)  # get status before clearing result
        if self._client.has_result(self._topic):
            # Check if the action has been finished
            result = self._client.get_result(self._topic, clear=True)
            Logger.localinfo(f"  '{self}' : '{self._topic}' returned {status} "
                             f'result.status={result.execution_result.status}'
                             f" '{result.execution_result.message}'")
            if status == GoalStatus.STATUS_SUCCEEDED:
                result_status = result.execution_result.status
                userdata.msg = result.execution_result.message  # Output message
                if result_status == ExecutionResult.SUCCESS:
                    self._return = 'done'
                else:
                    Logger.localwarn(f"{self} : '{self._topic}' -"
                                     f" failed during follow ({result_status}) '{userdata.msg}'")
                    self._return = 'failed'
                return self._return
        # Note: Not checking a timeout here it is up to follow capability and/or operator to enforce
        #       given varying path lengths

        # Otherwise check for action status change
        if status == GoalStatus.STATUS_CANCELED:
            Logger.loginfo(f" '{self}' : '{self._topic}' - goal was canceled! ")
            self._return = 'failed'
        elif status == GoalStatus.STATUS_ABORTED:
            Logger.loginfo(f" '{self}' : '{self._topic}' -  goal was aborted! ")
            self._return = 'failed'

        # If the action has not yet finished, None outcome will be returned and the state stays active.
        return self._return

    def on_enter(self, userdata):
        """Call when state becomes active."""
        # make sure to reset the data from prior executions
        Logger.localinfo(f"on_enter '{self}' - '{self.path}' ...")
        self._return = None
        userdata.msg = ''

        if 'path' not in userdata:
            self._return = 'failed'
            userdata.msg = f"FollowPathState '{self}' requires userdata.path key!"
            Logger.localwarn(userdata.msg)
            return

        # Send the goal.
        try:
            path = userdata.path
            if isinstance(path, Path):
                self._goal = FollowPath.Goal(path=path)
            else:
                userdata.msg = f"Invalid goal type '{type(path)}' - must be pyrobosim_msgs/Path!"
                Logger.localwarn(userdata.msg)
                self._return = 'failed'
                return
            Logger.localinfo(f'Send follow path goal with {len(path.poses)} waypoints ...')
            # Send goal clears prior results
            self._client.send_goal(self._topic, self._goal, wait_duration=self._server_timeout_sec)
        except Exception as exc:  # pylint: disable=W0703
            # Since a state failure does not necessarily cause a behavior failure,
            # it is recommended to only print warnings, not errors.
            # Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.logwarn(f"Failed to send the '{self}' command:\n  {type(exc)} - {exc}")
            self._client.remove_result(self._topic)  # clear any prior result from action server
            self._return = 'failed'

    def on_exit(self, userdata):
        """Call when state is deactivated."""
        # Make sure that the action is not running when leaving this state.
        # A situation where the action would still be active is for example when the operator manually triggers an outcome.

        if self._client.is_active(self._topic):
            self._client.cancel(self._topic)

            # Check for action status change (blocking call!)
            is_terminal, status = self._client.verify_action_status(self._topic, 0.1)
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._topic}' - request to follow was canceled! ")
            else:
                status_string = self._client.get_status_string(self._topic)
                Logger.loginfo(f" '{self}' : '{self._topic}' - Requested to cancel an active follow request"
                               f" ({is_terminal}, '{status_string}').")
            self._return = 'failed'

        # Local message are shown in terminal but not the UI
        if self._return == 'done':
            Logger.localinfo('Successfully followed path.')
        else:
            Logger.localwarn('Failed to follow path.')
        Logger.localinfo(f"on_exit '{self}' - '{self.path}' ...")

        # Choosing to remove in on_enter and retain in proxy for now
        # Either choice can be valid.
        # if self._client.has_result(self._topic):
        #     # remove the old result so we are ready for the next time
        #     # and don't prematurely return
        #     self._client.remove_result(self._topic)

    def on_pause(self):
        """Execute each time this state is paused."""
        Logger.localinfo(f"on_pause '{self}' - '{self.path}' ...")
        if self._client.is_active(self._topic):
            self._client.cancel(self._topic)
            Logger.localinfo(f"Cancelling active follow action '{self}' when paused ...")
            # Check for action status change (blocking call!)
            is_terminal, status = self._client.verify_action_status(self._topic, 0.1)
            if status == GoalStatus.STATUS_CANCELED:
                Logger.loginfo(f" '{self}' : '{self._topic}' - request to follow was canceled! ")
            else:
                status_string = self._client.get_status_string(self._topic)
                Logger.loginfo(f" '{self}' : '{self._topic}' - Requested to cancel an active follow request"
                               f" ({is_terminal}, '{status_string}').")
            self._return = 'failed'

    def on_resume(self, userdata):
        """Execute each time this state is resumed."""
        Logger.localinfo(f"on_resume '{self}' - '{self.path}' ...")
        if self._return is None:
            Logger.localinfo(f"Cannot resume follow action '{self}' - require new plan ...")
            self._return = 'failed'

    def on_start(self):
        """Call when behavior starts."""
        Logger.localinfo(f" on_start  '{self}' - '{self.path}' ")

    def on_stop(self):
        """Call when behavior stops."""
        Logger.localinfo(f" on_stop  '{self}' - '{self.path}'")
