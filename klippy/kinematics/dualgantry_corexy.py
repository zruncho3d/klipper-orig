# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2022 Zruncho3D <zruncho3d@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper

class DualGantryCoreXYKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzuv']
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[4].get_steppers():
            self.rails[3].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[3].get_steppers():
            self.rails[4].get_endstops()[0][0].add_stepper(s)
        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
        self.rails[3].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[4].setup_itersolve('corexy_stepper_alloc', b'-')
        for n, r in enumerate(self.rails):
            for s in r.get_steppers():
                if n < 3:
                    s.set_trapq(toolhead.get_trapq())
                toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord( min(ranges[0][0], ranges[3][0]),
                    min(ranges[1][0], ranges[4][0]), ranges[2][0])
        self.axes_max = toolhead.Coord( max(ranges[0][1], ranges[3][1]),
                    max(ranges[1][1], ranges[4][1]), ranges[2][1])
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        self.active_carriage = self.last_inactive_position = None
        self.printer.lookup_object('gcode').register_command(
                'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
                desc=self.cmd_SET_DUAL_CARRIAGE_help)
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        pos = [stepper_positions[rail.get_name()] for rail in self.rails[:3]]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails[:3]):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
    def _home_axis(self, homing_state, axis, rail):
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
    def home(self, homing_state):
        for axis in homing_state.get_axes():
            if axis in (0, 1):
                altc = self.active_carriage
                for i in [0, 1]:
                    self._activate_gantry(0)
                    self._home_axis(homing_state, axis, self.rails[axis])
                self._activate_gantry(altc)
            else:
                self._home_axis(homing_state, axis, self.rails[axis])
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self, eventtime):
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
            'dual_carriage': { 
                'mode': 'FULL_CONTROL',
                'actuator_carriage': self.active_carriage,
                'active_carriages': [self.active_carriage]
            }
        }
    def _activate_gantry(self, carriage):
        if self.active_carriage == carriage:
            return
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        _restore_position = self.last_inactive_position or pos
        self.last_inactive_position = pos
        self.rails[0], self.rails[3] = self.rails[3], self.rails[0]
        self.rails[1], self.rails[4] = self.rails[4], self.rails[1]
        for r in [self.rails[3], self.rails[4]]:
            r.set_trapq(None)
        for i, r in enumerate([self.rails[0], self.rails[1]]):
            r.set_trapq(toolhead.get_trapq())
            pos[i] = _restore_position[i]
            if self.limits[i][0] <= self.limits[i][1]:
                self.limits[i] = r.get_range()[i]
        toolhead.set_position(pos)
        self.active_carriage = carriage
    cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"
    def cmd_SET_DUAL_CARRIAGE(self, gcmd):
        carriage = gcmd.get_int('CARRIAGE', minval=0, maxval=1)
        self._activate_gantry(carriage)

def load_kinematics(toolhead, config):
    return DualGantryCoreXYKinematics(toolhead, config)
