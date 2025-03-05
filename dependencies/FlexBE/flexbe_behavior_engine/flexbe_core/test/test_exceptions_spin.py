#!/usr/bin/env python3

# Copyright 2024 Philipp Schillinger, Team ViGIR, Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Philipp Schillinger, Team ViGIR, Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Test FlexBE Exception handling with sm.spin()."""
import time
import unittest

from flexbe_core import EventState, OperatableStateMachine, initialize_flexbe_core
from flexbe_core.core.exceptions import StateError, StateMachineError, UserDataError
from flexbe_core.proxy import shutdown_proxies

import rclpy
from rclpy.executors import MultiThreadedExecutor


class TestExceptionsSpin(unittest.TestCase):
    """Test FlexBE Exception handling."""

    test = 0
    __EXECUTE_TIMEOUT_SEC = 0.2  # 0.025  # Timeout in executor loops for spin once
    __TIME_SLEEP = 0.2  # 0.025  # Sleep time for loops

    def __init__(self, *args, **kwargs):
        """Initialize TestExceptionsSpin instance."""
        super().__init__(*args, **kwargs)

    def setUp(self):
        """Set up the TestExceptionsSpin test."""
        TestExceptionsSpin.test += 1
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context)
        if not rclpy.ok(context=self.context):
            raise RuntimeError('rclpy failed to initialize properly with context')

        self.executor = MultiThreadedExecutor(context=self.context)
        self.node = rclpy.create_node('exception_test_' + str(self.test), context=self.context)
        self.node.get_logger().info(' set up exceptions test %d (%d) ... ' % (self.test, self.context.ok()))
        self.executor.add_node(self.node)
        initialize_flexbe_core(self.node)

    def tearDown(self):
        """Tear down the TestExceptionsSpin test."""
        self.node.get_logger().info(' shutting down exceptions test %d (%d) ... ' % (self.test, self.context.ok()))
        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=2 * TestExceptionsSpin.__EXECUTE_TIMEOUT_SEC)

        self.node.get_logger().info('    shutting down proxies in core test %d ... ' % (self.test))
        shutdown_proxies()
        time.sleep(TestExceptionsSpin.__TIME_SLEEP)

        self.node.get_logger().info('    destroy node in core test %d ... ' % (self.test))
        self.node.destroy_node()

        time.sleep(TestExceptionsSpin.__TIME_SLEEP)
        self.executor.shutdown()
        time.sleep(TestExceptionsSpin.__TIME_SLEEP)

        # Kill it with fire to make sure not stray published topics are available
        rclpy.shutdown(context=self.context)
        time.sleep(TestExceptionsSpin.__TIME_SLEEP * 2)

    def test_invalid_outcome(self):
        """Test invalid outcome."""
        self.node.get_logger().info('test_invalid_outcome ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestExceptionsSpin.__EXECUTE_TIMEOUT_SEC)

        class ReturnInvalidOutcomeState(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'])
                self._state_id = 256

            def execute(self, userdata):
                self._node.get_logger().info('test_invalid_outcome  - execute with invalid outcome ...')
                return 'invalid'

            def on_enter(self, userdata):
                self._node.get_logger().info('test_invalid_outcome  - on_enter ...')
                super().on_enter(userdata)

        sm = OperatableStateMachine(name='test_invalid_outcome', outcomes=['done'])
        sm._state_id = 1024

        with sm:
            OperatableStateMachine.add('state', ReturnInvalidOutcomeState(), transitions={'done': 'done'})
        outcome = sm.spin(None, rclpy_context=self.context)

        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateError)
        self.node.get_logger().info('test_invalid_outcome  - OK! ')

    def test_invalid_transition(self):
        """Test invalid transition."""
        self.node.get_logger().info('test_invalid_transition ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestExceptionsSpin.__EXECUTE_TIMEOUT_SEC)

        class ReturnDoneState(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'])
                self._state_id = 256

            def execute(self, userdata):
                return 'done'

        inner_sm = OperatableStateMachine(outcomes=['done'])
        inner_sm._state_id = 512
        with inner_sm:
            OperatableStateMachine.add('state', ReturnDoneState(), transitions={'done': 'invalid'})
        sm = OperatableStateMachine(name='test_invalid_transition', outcomes=['done'])
        sm._state_id = 1024
        with sm:
            OperatableStateMachine.add('inner', inner_sm, transitions={'done': 'done'})

        outcome = sm.spin(None, rclpy_context=self.context)

        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, StateMachineError)
        self.node.get_logger().info('test_invalid_transition - OK! ')

    def test_invalid_userdata_input(self):
        """Test invalid user data."""
        self.node.get_logger().info('test_invalid_userdata ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestExceptionsSpin.__EXECUTE_TIMEOUT_SEC)

        class AccessInvalidInputState(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'], input_keys=['input'])
                self._state_id = 256

            def execute(self, userdata):
                print(userdata.invalid)
                return 'done'

        sm = OperatableStateMachine(name='test_invalid_userdata_input', outcomes=['done'])
        sm._state_id = 1023
        with sm:
            OperatableStateMachine.add('state', AccessInvalidInputState(), transitions={'done': 'done'})

        outcome = sm.spin(None, rclpy_context=self.context)

        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info('test_invalid_userdata - OK! ')

    def test_invalid_userdata_output(self):
        """Test invalid userdata output."""
        self.node.get_logger().info('test_invalid_userdata_output ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestExceptionsSpin.__EXECUTE_TIMEOUT_SEC)

        class SetInvalidOutputState(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'], output_keys=['output'])
                self._state_id = 256

            def execute(self, userdata):
                userdata.invalid = False
                return 'done'

        sm = OperatableStateMachine(name='test_invalid_userdata_output', outcomes=['done'])
        sm._state_id = 1024
        with sm:
            OperatableStateMachine.add('state', SetInvalidOutputState(), transitions={'done': 'done'})

        outcome = sm.spin(None, rclpy_context=self.context)

        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info('test_invalid_userdata_output - OK! ')

    def test_missing_userdata(self):
        """Test missing userdata."""
        self.node.get_logger().info('test_missing_userdata ...')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestExceptionsSpin.__EXECUTE_TIMEOUT_SEC)

        class AccessValidInputState(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'], input_keys=['missing'])
                self._state_id = 256

            def execute(self, userdata):
                print(userdata.missing)
                return 'done'

        sm = OperatableStateMachine(name='test_missing_userdata', outcomes=['done'])
        sm._state_id = 1024
        with sm:
            OperatableStateMachine.add('state', AccessValidInputState(), transitions={'done': 'done'})

        outcome = sm.spin(None, rclpy_context=self.context)

        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info('test_missing_userdata - OK! ')

    def test_modify_input_key(self):
        """Test modify input key."""
        self.node.get_logger().info('test_modify_input_key ...! ')

        rclpy.spin_once(self.node, executor=self.executor, timeout_sec=TestExceptionsSpin.__EXECUTE_TIMEOUT_SEC)

        class ModifyInputKeyState(EventState):
            """Local Test state definition."""

            def __init__(self):
                super().__init__(outcomes=['done'], input_keys=['only_input'])
                self._state_id = 256

            def execute(self, userdata):
                userdata.only_input['new'] = 'not_allowed'
                return 'done'

        sm = OperatableStateMachine(name='test_modify_input_key', outcomes=['done'])
        sm._state_id = 1024
        sm.userdata.only_input = {'existing': 'is_allowed'}
        with sm:
            OperatableStateMachine.add('state', ModifyInputKeyState(), transitions={'done': 'done'})

        outcome = sm.spin(None, rclpy_context=self.context)

        self.assertIsNone(outcome)
        self.assertIsInstance(sm._last_exception, UserDataError)
        self.node.get_logger().info('test_modify_input_key - OK! ')


if __name__ == '__main__':
    unittest.main()
