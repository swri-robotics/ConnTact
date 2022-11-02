# Copyright 2021 Southwest Research Institute
# Licensed under the Apache License, Version 2.0

from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import input
__metaclass__ = type

import robotiq_2f_gripper_control.robotiq_2f_gripper_ctrl as gr

class Gripper:
    """
    Provide functionality of the Robotiq gripper class but allow simulation
    """

    # The velocity to move the gripper at (corresponds to rSP = 255)
    VELOCITY = 0.1
    # The force to move the gripper at (corresponds to rFR = 100)
    FORCE = 71.18
    # The "open" position of the gripper (corresponds to fPR = 0)
    OPEN_POSITION = 0.09221
    # The "closed" position of the gripper (corresponds to fPR = 210)
    CLOSED_POSITION = 0.008018
    # The timeout to use for the gripper
    TIMEOUT = 5

    def __init__(self, sim=False):
        if not sim:
            self._gripper = gr.RobotiqCGripper()
        else:
            self._gripper = None

    def setup(self):
        if self.gripper is not None:
            self.gripper.wait_for_connection()
            if self.gripper.is_reset():
                self.gripper.reset()
                self.gripper.activate()

    def reset(self):
        if self.gripper is not None:
            self.gripper.reset()
        else:
            print('Gripper reset')

    def activate(self):
        if self.gripper is not None:
            if not self.gripper.activate(Gripper.TIMEOUT):
                raise Exception('Timeout activating gripper')
        else:
            print('Gripper activate')

    def open(self, message):
        if self.gripper is not None:
            if not self.gripper.goto(
                    Gripper.OPEN_POSITION,
                    Gripper.VELOCITY,
                    Gripper.FORCE,
                    True, Gripper.TIMEOUT):
                raise Exception('Timeout waiting on gripper')
        input('{} - press enter to continue...'.format(message))

    def close(self, message):
        if self.gripper is not None:
            if not self.gripper.goto(
                    Gripper.CLOSED_POSITION,
                    Gripper.VELOCITY,
                    Gripper.FORCE,
                    True, Gripper.TIMEOUT):
                raise Exception('Timeout waiting on gripper')
        input('{} - press enter to continue...'.format(message))

    @property
    def gripper(self):
        return self._gripper

