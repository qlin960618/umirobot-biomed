"""
Copyright (C) 2022 Murilo Marques Marinho (www.murilomarinho.info)
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program. If not,
see <https://www.gnu.org/licenses/>.
"""
import argparse
import multiprocessing as mp
import os
import sys
import numpy as np
import dqrobotics as dql

import umirobot_control.commons.controller_config as control_cfg
from umirobot_control.commons.controller_state_sm_manager import ControllerShmManager
from umirobot_control.task_space_control_keyboard._controller_init_main import \
    umirobot_communication_loop_under_subprocess

from PyQt6 import uic
# from PyQt6.QtCore import QTimer, QByteArray, QLocale, QTime, QEvent
from PyQt6 import QtCore
from PyQt6.QtWidgets import QApplication, QWidget

KEYPRESS_T_DELTA = 0.005
KEYPRESS_GRIP_DELTA = np.radians(5)

umi_configuration = {
    "controller_gain": 4.0,
    "damping": 0.01,
    "alpha": 0.999,  # Soft priority between translation and rotation [0,1] ~1 Translation, ~0 Rotation
    "use_real_master": False,
    "use_real_umirobot": False,
    "umirobot_port": "COM4"
}


class ControllerUI(QWidget):
    def __init__(self, args_in):
        super(ControllerUI, self).__init__()

        self.init_args = args_in
        self.load_ui()

        # initialize stored variable
        # translation
        self.t_offset = np.zeros([3])
        # rotation
        r_init = np.cos(np.radians(90/2))+np.sin(np.radians(90/2))*dql.i_
        r_init = r_init * (np.cos(np.radians(180/2))+np.sin(np.radians(180/2))*dql.k_)
        # r_init = dql.DQ([1])  # np.sin(np.radians(90/2))+np.cos(np.radians(90/2))*dql.j_
        self.r_offset = r_init.vec4()

        self.gripper = 0
        self.init_args["controller_sm_manager"].send_data(self.t_offset, self.r_offset, self.gripper)

    def load_ui(self):
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        uic.loadUi(path, self)  # Load the .ui file

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Escape:
            print("Killing")
            self.closeWindows()
            # self.deleteLater()
        elif event.key() == QtCore.Qt.Key.Key_Up:
            self.t_offset[2] += KEYPRESS_T_DELTA
        elif event.key() == QtCore.Qt.Key.Key_Down:
            self.t_offset[2] -= KEYPRESS_T_DELTA
        elif event.key() == QtCore.Qt.Key.Key_W:
            self.t_offset[1] += KEYPRESS_T_DELTA
        elif event.key() == QtCore.Qt.Key.Key_A:
            self.t_offset[0] += KEYPRESS_T_DELTA
        elif event.key() == QtCore.Qt.Key.Key_S:
            self.t_offset[1] -= KEYPRESS_T_DELTA
        elif event.key() == QtCore.Qt.Key.Key_D:
            self.t_offset[0] -= KEYPRESS_T_DELTA
        elif event.key() == QtCore.Qt.Key.Key_R:
            self.gripper += KEYPRESS_GRIP_DELTA
        elif event.key() == QtCore.Qt.Key.Key_F:
            self.gripper -= KEYPRESS_GRIP_DELTA
        elif event.key() == QtCore.Qt.Key.Key_Enter:
            print("Got Enter")

        self.init_args["controller_sm_manager"].send_data(self.t_offset, self.r_offset, self.gripper)
        event.accept()

    def closeWindows(self):
        self.init_args["controller_sm_manager"].exit()
        self.close()

    def closeEvent(self, event):
        self.init_args["controller_sm_manager"].exit()
        event.accept()


def main(args_in_):
    # control subprocess
    controller_shm_manager = ControllerShmManager()
    args_in_["controller_sm_manager"] = controller_shm_manager

    controller_process = mp.Process(
        target=umirobot_communication_loop_under_subprocess,
        args=(controller_shm_manager.get_shm_initializer_arg(), umi_configuration)
    )
    controller_process.start()

    # UI Process
    app = QApplication([])
    widget = ControllerUI(args_in_)
    widget.show()
    sys.exit(app.exec())

    controller_process.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control UI Init parameters")

    parser.add_argument('--rate', type=int, default=control_cfg.DEFAULT_LOOP_RATE,
                        help="Controller refresh rate")
    parser.add_argument('--motion_scale', type=float, default=control_cfg.DEFAULT_MOTION_SCALE,
                        help="Default motion scaling for controller")
    args = parser.parse_args()

    args_in = {
        "rate": args.rate,
        "motion_scale": args.motion_scale,
        # "controller_sm_manager": controller_sm_manager,
        "controller_sm_manager": None,
    }
    main(args_in)
