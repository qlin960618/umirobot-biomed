import logging

import multiprocessing as mp
from multiprocessing import shared_memory

import numpy as np

# logger init

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class ControllerShmManager:
    def __init__(self, exit_event=None, init_args=None):
        self.data_size_dummy = np.zeros([9], dtype=np.float64)

        if init_args is None:
            # mp event
            if exit_event is None:
                self.eExit = mp.Event()
            else:
                self.eExit = exit_event

            self.eReady = mp.Event()

            # frame shared_memory for left frame
            self.data_shm = shared_memory.SharedMemory(create=True,
                                                       size=self.data_size_dummy.size * self.data_size_dummy.itemsize)
            self.shm_lock = mp.Lock()

        else:
            self.eExit = init_args[0]
            self.eReady = init_args[1]
            self.data_shm = init_args[2]
            self.shm_lock = init_args[3]

    def get_shm_initializer_arg(self):
        return (self.eExit,
                self.eReady,
                self.data_shm,
                self.shm_lock)

    def exit(self):
        self.eExit.set()

    def get_exit_event(self):
        return self.eExit

    def is_exit_set(self):
        return self.eExit.is_set()

    def send_data(self, t, r, gripper):
        self.eReady.set()
        t_array = np.asarray(t)
        r_array = np.asarray(r)
        if self.shm_lock.acquire(block=True, timeout=1):
            out_buf = np.ndarray(self.data_size_dummy.shape, dtype=self.data_size_dummy.dtype, buffer=self.data_shm.buf)
            out_buf[1:4] = t_array[:]
            out_buf[4:8] = r_array[:]
            out_buf[8] = gripper
            self.shm_lock.release()

    def get_data(self):
        if not self.shm_lock.acquire(block=True, timeout=1) or not self.eReady.is_set():
            return None, None, None
        data_array = np.ndarray(self.data_size_dummy.shape, dtype=self.data_size_dummy.dtype, buffer=self.data_shm.buf)
        self.shm_lock.release()
        return data_array[1:4], data_array[4:8], data_array[8]

