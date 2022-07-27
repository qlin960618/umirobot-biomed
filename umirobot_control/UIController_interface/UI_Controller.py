import sys
import os
import argparse
import logging

from PyQt6 import QtWidgets, uic
from PyQt6.QtCore import QTimer, QByteArray, QLocale, QTime, QEvent
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QApplication, QWidget, QGraphicsScene
from PyQt6.QtGui import QDoubleValidator

from scipy.spatial.transform import Rotation as R

from functools import partial

# import configuration

uiName = "UIMaster"

X_ELEMENTS = ['zt', 'yt', 'xt',
              'zr', 'yr', 'xr']

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class ControlGraphicsSceneHandle(QGraphicsScene):
    signalMouseState = pyqtSignal(float, float, float, str)

    def __init__(self, parent):
        super(ControlGraphicsSceneHandle, self).__init__(parent)

    def mouseReleaseEvent(self, QGraphicsSceneMouseEvent):
        pos = QGraphicsSceneMouseEvent.lastScenePos()
        self.signalMouseState.emit(pos.x(), pos.y(), 0, 'release')

    def mouseMoveEvent(self, QGraphicsSceneMouseEvent):
        pos = QGraphicsSceneMouseEvent.lastScenePos()
        self.signalMouseState.emit(pos.x(), pos.y(), 0, 'move')

    def mousePressEvent(self, QGraphicsSceneMouseEvent):
        pos = QGraphicsSceneMouseEvent.lastScenePos()
        self.signalMouseState.emit(pos.x(), pos.y(), 0, 'pressed')

    def wheelEvent(self, QGraphicsSceneWheelEvent):
        delta = QGraphicsSceneWheelEvent.delta()
        logger.debug("got wheel")

        self.signalMouseState.emit(0, 0, float(delta), 'wheel')


def zxy2quat(yaw, pitch, roll):
    r = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    q_w = r.as_quat()
    return [q_w[3], q_w[0], q_w[1], q_w[2]]


class ControllerUI(QWidget):
    def __init__(self, args_in):
        super(ControllerUI, self).__init__()
        self.updateRate = None
        self.udpRecvSocket = None
        self.doubleValidator = None
        self.udpSendSocket = None
        self.init_args = args_in
        self.load_ui()

        # setup loopTimer
        self.loopTimer = QTimer(self)
        self.loopTimer.setSingleShot(False)
        # self.loopDtTimer = QTime()
        # self.loopDT_T_start = self.loopDtTimer.currentTime()

        # graph control memory
        self.control_active = None
        self.control_x_init = 0
        self.control_y_init = 0

    def _zero_rot(self):
        # change default initial default rotation
        self.control_state_var['xr'] = self.init_args['cfg'].X_ROT_INIT_DEG
        self.control_state_var['yr'] = self.init_args['cfg'].Y_ROT_INIT_DEG
        self.control_state_var['zr'] = self.init_args['cfg'].Z_ROT_INIT_DEG

    def load_ui(self):
        path = os.path.join(os.path.dirname(__file__), "ui_form.ui")
        uic.loadUi(path, self)  # Load the .ui file

        self.setFixedSize(970, 470)
        self.xd_raw_control_state_var = {
            key: 0 for key in X_ELEMENTS
        }

        self.control_state_var = {
            key: 0 for key in X_ELEMENTS
        }
        self.control_state_var.update({
            key + "_v": 0 for key in X_ELEMENTS
        })
        self.control_state_var.update({
            "gripper": 0,
            'motionScale': self.init_args["motion_scale"]
        })

        # change default initial default rotation
        self._zero_rot()

        # connect signal and misc
        self.xrGraph_left.setInteractive(True)
        self.xrGraph_left_scene = ControlGraphicsSceneHandle(self)
        self.xrGraph_left_scene.signalMouseState.connect(partial(self.graph_control_cb_, 'r'))
        self.xrGraph_left.setScene(self.xrGraph_left_scene)

        self.xtGraph_left.setInteractive(True)
        self.xtGraph_left_scene = ControlGraphicsSceneHandle(self)
        self.xtGraph_left_scene.signalMouseState.connect(partial(self.graph_control_cb_, 't'))
        self.xtGraph_left.setScene(self.xtGraph_left_scene)

        self.gripperSlider_left.valueChanged.connect(partial(self.gripper_value_changed_,
                                                             self.gripperSlider_left,
                                                             'gripper'))

        self.doubleValidator = QDoubleValidator(self)
        locale = QLocale("en")
        self.doubleValidator.setLocale(locale)
        self.motionScaleEdit.setText(str(self.init_args["motion_scale"]))
        self.motionScaleEdit.setValidator(self.doubleValidator)
        self.motionScaleEdit.returnPressed.connect(self.motion_scale_changed_)

        self.zeroXdBtn.clicked.connect(self.zero_all_xd_comonents_)

        logger.debug("Done initializing")

    ##################################################################################################################
    def zero_wheel_(self):
        for source in ['t', 'r']:
            self.xd_raw_control_state_var['z' + source] = 0

    def graph_control_cb_(self, source, x, y, delta, cmd_type):
        # print("from:", source, "got position", x, y, delta, cmd_type)
        # 'zt_left', 'yt_left', 'xt_left',
        # 'zr_left', 'yr_left', 'xr_left',
        # 'yt_right', 'zt_right', 'xt_right',
        # 'yr_right', 'zr_right', 'xr_right'
        # recalculate value and map direction
        if cmd_type == 'pressed':
            self.control_active = source
            self.control_x_init = x
            self.control_y_init = y
            delta_x = 0
            delta_y = 0
            delta_z = 0
        if cmd_type == 'release':
            self.control_active = None
            delta_x = 0
            delta_y = 0
            delta_z = 0
        if cmd_type == 'move':
            delta_x = (x - self.control_x_init) / self.init_args['cfg'].PLANE_SCALING
            delta_y = -(y - self.control_y_init) / self.init_args['cfg'].PLANE_SCALING
            # self.control_x_init = x
            # self.control_y_init = y
            delta_z = 0
            if self.control_active != source:
                self.control_active = None
                delta_x = 0
                delta_y = 0
                delta_z = 0
        if cmd_type == 'wheel':
            delta_z = delta / self.init_args['cfg'].WHEEL_SCALING
            delta_x = 0
            delta_y = 0

        if source in ['r_left', 'r_right']:
            mlt = self.init_args['cfg'].ROTATION_SCALING
        else:
            mlt = 1

        self.xd_raw_control_state_var['x' + source] = delta_x * mlt
        self.xd_raw_control_state_var['y' + source] = delta_y * mlt
        self.xd_raw_control_state_var['z' + source] = delta_z * mlt

    #####################################################################################################################

    def zero_all_xd_comonents_(self):
        for key in self.xd_raw_control_state_var:
            self.xd_raw_control_state_var[key] = 0
            self.control_state_var[key] = 0
            self.control_state_var[key + "_v"] = 0

        self._zero_rot()

    # def control_btn_changed_(self, obj, key):
    #     if obj.isChecked():
    #         self.control_state_var[key] = True
    #     else:
    #         self.control_state_var[key] = False

    def gripper_value_changed_(self, obj, key):
        self.control_state_var[key] = obj.value() / 100.0

    def motion_scale_changed_(self):

        self.control_state_var['motionScale'] = float(self.motionScaleEdit.text())

    def process_recv_udp_data_(self):
        while self.udpRecvSocket.hasPendingDatagrams():
            datagram, host, port = self.udpRecvSocket.readDatagram(self.udpRecvSocket.pendingDatagramSize())
            # print("Packet Recv")
            # print(datagram)

    def start_loop_callback(self, update_rate):
        logger.debug("Starting timer")
        self.updateRate = update_rate
        self.loopTimer.setInterval(int(1000 / update_rate))  # in milliseconds, so 5000 = 5 seconds
        self.loopTimer.timeout.connect(self.timer_callback_)
        self.loopTimer.start()

    def timer_callback_(self):
        # update xd val
        logger.debug("Started time")
        for key in self.xd_raw_control_state_var:
            val = self.xd_raw_control_state_var[key] / self.updateRate
            self.control_state_var[key] += val
            self.control_state_var[key + "_v"] = val
            # self.xd_raw_control_state_var[key] = 0
        self.zero_wheel_()

        for key in self.control_state_var:
            logger.INFO(f"{key}, {self.control_state_var[key]}")
        #     if hasattr(dataobj, key):
        #         setattr(dataobj, key, self.control_state_var[key])
        #
        # q = zxy2quat(self.control_state_var['zr_left'], self.control_state_var['yr_left'],
        #              self.control_state_var['xr_left'])
        # dataobj.wqr_left, dataobj.xqr_left, dataobj.yqr_left, dataobj.zqr_left = tuple(q)
        #
        # q = zxy2quat(self.control_state_var['zr_right'], self.control_state_var['yr_right'],
        #              self.control_state_var['xr_right'])

    """    
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Escape:
            print("Killing")
            self.closeWindows()
            # self.deleteLater()
        elif event.key() == QtCore.Qt.Key.Key_Up:
            print("Got up")
        elif event.key() == QtCore.Qt.Key.Key_Down:
            print("Got down")
        elif event.key() == QtCore.Qt.Key.Key_W:
            print("Got W")
        elif event.key() == QtCore.Qt.Key.Key_A:
            print("Got A")
        elif event.key() == QtCore.Qt.Key.Key_S:
            print("Got S")
        elif event.key() == QtCore.Qt.Key.Key_D:
            print("Got D")
        elif event.key() == QtCore.Qt.Key.Key_Enter:
            print("Got Enter")
        event.accept()
    """


class cfg:
    DEFAULT_MOTION_SCALE = 1
    DEFAULT_LOOP_RATE = 30

    ROTATION_SCALING = 1000
    WHEEL_SCALING = 12000
    PLANE_SCALING = 20000

    # X_ROT_INIT_DEG=-90
    X_ROT_INIT_DEG = 0
    Y_ROT_INIT_DEG = 0
    Z_ROT_INIT_DEG = 0


def main(args_in_):
    if "controller_sm_manager" not in args_in_:
        args_in_["controller_sm_manager"] = None
    if "rate" not in args_in_:
        args_in_["rate"] = cfg().DEFAULT_LOOP_RATE
    if "motion_scale" not in args_in_:
        args_in_["motion_scale"] = cfg().DEFAULT_MOTION_SCALE
    if "cfg" not in args_in_:
        args_in_["cfg"] = cfg()

    app = QApplication([])
    widget = ControllerUI(args_in_)
    widget.start_loop_callback(args_in_["rate"])
    widget.show()
    sys.exit(app.exec())



if __name__ == '__main__':
    # Read from arguments
    # from controller_state_sm_manager import ControllerSMManager
    #
    # controller_sm_manager = ControllerSMManager()

    parser = argparse.ArgumentParser(description="Control UI Init parameters")

    parser.add_argument('--rate', type=int, default=cfg().DEFAULT_LOOP_RATE,
                        help="Controller refresh rate")
    parser.add_argument('--motion_scale', type=float, default=cfg().DEFAULT_MOTION_SCALE,
                        help="Default motion scaling for controller")

    args = parser.parse_args()

    args_in = {
        "rate": args.rate,
        "motion_scale": args.motion_scale,
        # "controller_sm_manager": controller_sm_manager,
        "controller_sm_manager": None,
    }
    main(args_in)

