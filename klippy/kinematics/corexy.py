# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import configparser, logging, math
import stepper
from . import dual_gantry_modes

error = configparser.Error

class CoreXYKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        printer_config = config.getsection('printer')

        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyz']
        for s in self.rails[1].get_steppers():
            self.rails[0].get_endstops()[0][0].add_stepper(s)
        for s in self.rails[0].get_steppers():
            self.rails[1].get_endstops()[0][0].add_stepper(s)
        self.rails[0].setup_itersolve('corexy_stepper_alloc', b'+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', b'-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', b'z')
        ranges = [r.get_range() for r in self.rails]
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        self.dg_module = None
        if config.has_section('dual_gantry_x'):
            if not config.has_section('dual_gantry_y'):
                raise error("If using dual gantry, must specify X & Y section")
            dg_config_x = config.getsection('dual_gantry_x')
            dg_config_y = config.getsection('dual_gantry_y')
            # dummy for cartesian config users
            # XXX: commenting out; not relevant
            #dg_config_x.getchoice('axis', {'x': 'x'}, default='x')
            #dg_config_y.getchoice('axis', {'x': 'x'}, default='x')
            # # setup second gantry with both rails
            self.rails.append(stepper.PrinterRail(dg_config_x)) # rail[3] right side x
            self.rails.append(stepper.PrinterRail(dg_config_y)) # rail[4] right side y
            # XXX restore below!
            for s in self.rails[3].get_steppers():
                self.rails[4].get_endstops()[0][0].add_stepper(s)
            for s in self.rails[4].get_steppers():
                self.rails[3].get_endstops()[0][0].add_stepper(s)
            # JUST A HACK for now to understand what's going on.
            #self.rails[3].setup_itersolve('cartesian_stepper_alloc', b'x')
            #self.rails[4].setup_itersolve('cartesian_stepper_alloc', b'x')
            self.rails[3].setup_itersolve('cartesian_stepper_alloc', b'x')
            self.rails[4].setup_itersolve('cartesian_stepper_alloc', b'x')
            # Left
            dg_rail_pair_0 = dual_gantry_modes.DualGantriesRailPair(
                self.printer,
                (self.rails[0], self.rails[1]), 
                active=True,
                stepper_alloc_active_x=('corexy_stepper_alloc', b'+'),
                stepper_alloc_active_y=('corexy_stepper_alloc', b'-'),
                stepper_alloc_inactive_x=None,
                stepper_alloc_inactive_y=None)
            # Right
            dg_rail_pair_1 = dual_gantry_modes.DualGantriesRailPair(
                self.printer,
                (self.rails[3], self.rails[4]),
                active=False,
                stepper_alloc_active_x=('corexy_stepper_alloc', b'+'),
                stepper_alloc_active_y=('corexy_stepper_alloc', b'-'),
                stepper_alloc_inactive_x=None,
                stepper_alloc_inactive_y=None)
            self.dg_module = dual_gantry_modes.DualGantries(self.printer,
                        dg_rail_pair_0, dg_rail_pair_1)

        for i, s in enumerate(self.get_steppers()):
            if i < 3:
                s.set_trapq(toolhead.get_trapq())
                toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
       
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3

        for i, r in enumerate(self.rails):
            self.printer.lookup_object('gcode').respond_info(
                "setting rail_pair: with %s" % repr(r))


        # !!!  NOT TESTED BELOW. PROBABLY IN WRONG LOCATION
        
        self.printer.register_event_handler("klippy:ready",
                                            self.after_connect)

    def after_connect(self):
        self.printer.lookup_object('gcode').respond_info(
                "in after_connect")
        toolhead = self.printer.lookup_object('toolhead')
        self.dg_module.dg[1].inactivate(toolhead.get_position())

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self, stepper_positions):
        # XXX TODO: fix this.  Almost certainly wrong.
        pos = [stepper_positions[rail.get_name()] for rail in self.rails]
        if (self.dg_module is not None and 'GANTRY_1' == \
                    self.dg_module.get_status()['active_gantry']):
            return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
        else:
            return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
    # XXX Added similarly to Hybrid-CoreXY kins
    def override_rail(self, i, rail):
        self.printer.lookup_object('gcode').respond_info(
                    "overriding rail %i with rail: %s " % (i, repr(rail)))
        self.rails[i] = rail
    def update_limits(self, i, range):
        self.limits[i] = range

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)

    # # # Original-ish CoreXY homing behavior (slightly refactored)
    # def home(self, homing_state):
    #     # Each axis is homed independently and in order
    #     for axis in homing_state.get_axes():
    #         rail = self.rails[axis]
    #         self._home_axis(homing_state, axis, rail)

    # Helper, taken from hybrid_corexy.py
    def _home_axis(self, homing_state, axis, rail):
        position_min, position_max = rail.get_range()
        self.printer.lookup_object('gcode').respond_info("min, max: %s, %s" %
            (position_min, position_max))
        hi = rail.get_homing_info()
        self.printer.lookup_object('gcode').respond_info("hi: %s" % repr(hi))
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        self.printer.lookup_object('gcode').respond_info("homepos: %s" % homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        self.printer.lookup_object('gcode').respond_info("performing homing")
        self.printer.lookup_object('gcode').respond_info(
            "homing with rail: %s, forcepos: %s, homepos: %s" % (rail, forcepos, homepos)
        )
        homing_state.home_rails([rail], forcepos, homepos)

    # Mod from hybrid_corexy.py
    def home(self, homing_state):
        for axis in homing_state.get_axes():
            # Home X?
            if (self.dg_module is not None and axis == 0):
                #raise error("hello! here x")
                #self.dg_module.save_dg_state()
                #self._home_axis(homing_state, axis, self.rails[0]
                # Start with left X
                self.dg_module.toggle_active_dg_rails(0)
                self._home_axis(homing_state, axis, self.rails[0]) # ?
                # Continue on right X
                self.printer.lookup_object('gcode').respond_info(
                    "About to home right-side X")
                self.dg_module.toggle_active_dg_rails(1)
                self.printer.lookup_object('gcode').respond_info("about to home axis")
                self._home_axis(homing_state, axis, self.rails[0]) # ?
                self.printer.lookup_object('gcode').respond_info("finished homing x")
                #self.dg_module.restore_dg_state()
                # Tircown says: may have issue with dg.state(): will end with wrong carriage active!
                # T0, then no save/restore, can have T1 active at end
            # Home Y?
            elif (self.dg_module is not None and axis == 1):
                #raise error("hello! here x")
                #self.dg_module.save_dg_state()
                #self._home_axis(homing_state, axis, self.rails[1])
                for i in [1, 4]:
                    self.dg_module.toggle_active_dg_rail(i)
                    self._home_axis(homing_state, axis, self.rails[i])
                #self.dg_module.restore_dg_state()
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
        }

def load_kinematics(toolhead, config):
    return CoreXYKinematics(toolhead, config)
