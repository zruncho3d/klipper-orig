# Support for duplication and mirroring modes for Dual Gantry printers
#
# Copyright (C) 2022  Zruncho3D <zruncho3d@gmail.com>
#
# Derived from idex_modes.py.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

class DualGantries:
    def __init__(self, printer, rail_pair_0, rail_pair_1):
        self.printer = printer
        self.printer.lookup_object('gcode').respond_info(
                "setting rail_pair_0: with %s" % repr(rail_pair_0))
        self.printer.lookup_object('gcode').respond_info(
                "setting rail_pair_1: with %s" % repr(rail_pair_1))
        self.dg = (rail_pair_0, rail_pair_1)  # tuple of DualGantriesRailPairs
        self.saved_state = None  # dict with 'mode', 'active_gantry', and 'axis_positions' values
        self.printer.add_object('dual_gantry', self)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
                   'SET_DUAL_GANTRY', self.cmd_SET_DUAL_GANTRY,
                   desc=self.cmd_SET_DUAL_GANTRY_help)
    def toggle_active_dg_rails(self, index):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        pos = toolhead.get_position()
        kin = toolhead.get_kinematics()
        for i, dg in enumerate(self.dg):
            dg_rail_pair = dg.get_rail_pair()
            self.printer.lookup_object('gcode').respond_info(
                "get_rail_pair: with %s" % repr(dg_rail_pair))

            self.printer.lookup_object('gcode').respond_info(
                        "rail_pair %s" % repr(dg_rail_pair))
            if i != index:
                dg.inactivate(pos)
                # Trying to workaround stuff below:
                for j in [0, 1]:  #range(2):
                    kin.override_rail(j + 3, dg_rail_pair[j])
            elif dg.is_active() is False:
                # Code below is wrong, and incomplete.
                self.printer.lookup_object('gcode').respond_info(
                    "pos: %s, xy_position: %s" % (pos, dg.xy_position))
                newpos = dg.xy_position + pos[2:]  # Include all position value: 
                dg.activate(newpos)
                
                for j in [0, 1]:  #range(2):
                    self.printer.lookup_object('gcode').respond_info(
                        "override_rail with %s" % repr(dg_rail_pair[j]))
                    kin.override_rail(j, dg_rail_pair[j])

                toolhead.set_position(newpos)
                for j in range(2):
                    kin.update_limits(j, dg_rail_pair[j].get_range())
    def get_status(self, eventtime=None):
        dg0, dg1 = self.dg
        if (dg0.is_active() is True):
            return { 'mode': 'FULL_CONTROL', 'active_gantry': 'GANTRY_0' }
        else:
            return { 'mode': 'FULL_CONTROL', 'active_gantry': 'GANTRY_1' }
    # def save_dg_state(self):
    #     dg0, dg1 = self.dg
    #     if (dg0.is_active() is True):
    #         mode, active_gantry = ('FULL_CONTROL', 'GANTRY_0')
    #     else:
    #         mode, active_gantry = ('FULL_CONTROL', 'GANTRY_1')
    #     self.saved_state = {
    #         'mode': mode,
    #         'active_gantry': active_gantry,
    #         'axis_positions': [dg0.xy_position, dg1.xy_position]
    #         }
    # def restore_dg_state(self):
    #     if self.saved_state is not None:
    #         # set gantry 0 active
    #         if (self.saved_state['active_gantry'] == 'GANTRY_0'
    #                     and self.dg[0].is_active() is False):
    #             self.toggle_active_dg_rails(0)
    #         # set gantry 1 active
    #         elif (self.saved_state['active_gantry'] == 'GANTRY_1'
    #                     and self.dg[1].is_active() is False):
    #             self.toggle_active_dg_rails(1)
    cmd_SET_DUAL_GANTRY_help = "Set which gantry is active"
    def cmd_SET_DUAL_GANTRY(self, gcmd):
        index = gcmd.get_int('GANTRY', minval=0, maxval=1)
        if (not(self.dg[0].is_active() == self.dg[1].is_active() == True)
                    and self.dg[index].is_active() is False):
            self.toggle_active_dg_rails(index)

### Should be good to test now.
class DualGantriesRailPair:
    ACTIVE=1
    INACTIVE=2
    def __init__(self, printer, rail_pair, active, stepper_alloc_active_x,
                stepper_alloc_active_y, stepper_alloc_inactive_x=None,
                stepper_alloc_inactive_y=None):
        self.printer = printer
        self.rail_pair = rail_pair
        self.status = (self.INACTIVE, self.ACTIVE)[active]
        self.stepper_alloc_active_x = stepper_alloc_active_x
        self.stepper_alloc_active_y = stepper_alloc_active_y
        self.stepper_alloc_inactive_x = stepper_alloc_inactive_x
        self.stepper_alloc_inactive_y = stepper_alloc_inactive_y
        
        # XXX: What is this var really needed for?
        self.xy_position = [-1, -1]
    def _stepper_alloc(self, position, active=True):
        toolhead = self.printer.lookup_object('toolhead')
        self.xy_position = position[:2]
        self.printer.lookup_object('gcode').respond_info(
                    "in stepper_alloc")
        for rail in self.rail_pair:
            rail.set_trapq(None)
        if active is True:
            self.printer.lookup_object('gcode').respond_info("in active")
            self.status = self.ACTIVE
            self.printer.lookup_object('gcode').respond_info(
                "in active: stepper_alloc_active_x: %s" % repr(self.stepper_alloc_active_x))
            self.printer.lookup_object('gcode').respond_info(
                "in active: stepper_alloc_active_y: %s" % repr(self.stepper_alloc_active_y))
            if self.stepper_alloc_active_x is not None:
                self.rail_pair[0].setup_itersolve(*self.stepper_alloc_active_x)
                self.rail_pair[0].set_position(position)
                self.rail_pair[0].set_trapq(toolhead.get_trapq())
            if self.stepper_alloc_active_y is not None:
                self.rail_pair[1].setup_itersolve(*self.stepper_alloc_active_y)
                self.rail_pair[1].set_position(position)
                self.rail_pair[1].set_trapq(toolhead.get_trapq())
        else:
            self.printer.lookup_object('gcode').respond_info("in inactive")
            self.status = self.INACTIVE
            self.printer.lookup_object('gcode').respond_info(
                "in inactive: stepper_alloc_inactive_x: %s" % repr(self.stepper_alloc_inactive_x or 'other'))
            self.printer.lookup_object('gcode').respond_info(
                "in inactive: stepper_alloc_inactive_y: %s" % repr(self.stepper_alloc_inactive_y or 'other'))
            if self.stepper_alloc_inactive_x is not None:
                self.rail_pair[0].setup_itersolve(*self.stepper_alloc_inactive_x)
                self.rail_pair[0].set_position(position)
                self.rail_pair[0].set_trapq(toolhead.get_trapq())
            if self.stepper_alloc_inactive_y is not None:
                self.rail_pair[1].setup_itersolve(*self.stepper_alloc_inactive_y)
                self.rail_pair[1].set_position(position)
                self.rail_pair[1].set_trapq(toolhead.get_trapq())
    def get_rail_pair(self):
        return self.rail_pair
    def is_active(self):
        return self.status == self.ACTIVE
    def activate(self, position):
        self._stepper_alloc(position, active=True)
    def inactivate(self, position):
        self._stepper_alloc(position, active=False)
