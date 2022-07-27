import multiprocessing as mp
import multiprocessing.managers as mm

from umirobot import UMIRobot
from umirobot.shared_memory import UMIRobotSharedMemoryProvider

from umirobot_control.task_space_control_keyboard._umirobot_task_space_control import run

from umirobot_control.commons.controller_state_sm_manager import ControllerShmManager


def umirobot_communication_loop_under_subprocess(controller_shm_manager_args, dc):
    controller_shm_manager_ = ControllerShmManager(init_args=controller_shm_manager_args)

    with UMIRobot() as umirobot, mm.SharedMemoryManager() as smm:
        # Lock
        lock = mp.Lock()
        # Provider
        shared_memory_provider = UMIRobotSharedMemoryProvider(shared_memory_manager=smm, lock=lock)
        # Receiver
        shared_memory_receiver_process = mp.Process(
            target=run,
            args=(shared_memory_provider.get_shared_memory_receiver_initializer_args(), controller_shm_manager_args, lock)
        )
        shared_memory_receiver_process.start()

        try:
            # Control loop
            while True:

                # Connect to the serial port if requested
                if shared_memory_provider.get_port() is not None:
                    if shared_memory_provider.get_port_connect_signal():
                        if shared_memory_provider.get_port() != umirobot.get_port():
                            umirobot.set_port(shared_memory_provider.get_port())
                            shared_memory_provider.send_port_connect_signal(False)

                # If connection is open, update, handle q and qd
                if umirobot.is_open():
                    umirobot.update()
                    shared_memory_provider.send_q(umirobot.get_q())
                    shared_memory_provider.send_potentiometer_values(umirobot.get_potentiometer_values())
                    shared_memory_provider.send_digital_in_values(umirobot.get_digital_in_values())
                    umirobot.set_qd(shared_memory_provider.get_qd())

                # Always send connection status
                shared_memory_provider.send_is_open(umirobot.is_open())

                # Check if shutdown signal was sent by receiver
                if shared_memory_provider.get_shutdown_flag():
                    print('umirobot::__main__::Info::Provider shutdown by receiver.')
                    break
                if controller_shm_manager_.is_exit_set():
                    shared_memory_provider.send_shutdown_flag(True)
                    print('umirobot::__main__::Info::Provider shutdown by Controller.')
                    break

        except Exception as e:
            print('umirobot::__main__::Error::' + str(e))
        except KeyboardInterrupt:
            print('umirobot::__main__::Info::Shutdown by CTRL+C.')
        shared_memory_receiver_process.join()