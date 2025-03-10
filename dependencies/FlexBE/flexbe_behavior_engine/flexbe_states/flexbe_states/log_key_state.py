#!/usr/bin/env python

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

"""LogKeyState."""
from flexbe_core import EventState, Logger


class LogKeyState(EventState):
    """
    A state that can log a predefined message including an input key.

    The text should be a Python format string (e.g. 'Counter value:  {}'),
    where {} is a placeholder replaced by the value of the userdata.data using the
    text.format(userdata.data) command.

    This can be used to precisely inform the operator about what happened to the behavior.

    -- text         string    Format string of message to be logged to the terminal; e.g., 'Counter value:  {}'.
    -- severity     uint8     Type of logging (Logger.REPORT_INFO / WARN / HINT / ERROR)

    #> data         object    The data provided to be printed in the message. The exact type depends on the request.

    <= done                   Indicates that the message has been logged.
    """

    def __init__(self, text, severity=Logger.REPORT_HINT):
        """Initialize LogKeyState."""
        super(LogKeyState, self).__init__(outcomes=['done'],
                                          input_keys=['data'])
        self._text = text
        self._severity = severity

    def execute(self, userdata):
        """Return done on first execution."""
        # Already logged. No need to wait for anything.
        return 'done'

    def on_enter(self, userdata):
        """Log upon entering the state."""
        Logger.log(self._text.format(userdata.data), self._severity)
