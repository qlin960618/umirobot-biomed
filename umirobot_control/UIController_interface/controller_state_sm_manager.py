import logging

import multiprocessing as mp
from multiprocessing import shared_memory

import numpy as np

# logger init

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class ControllerSMManager:
    def __init__(self, exit_event=None):
        # camera metadata
        self.fps = None
        self.width = None
        self.height = None

        # mp event
        if exit_event is None:
            self.eExit = mp.Event()
        else:
            self.eExit = exit_event

        # frame shared_memory for left frame
        self.data_dummy = np.zeros([8], dtype=np.float64)
        self.data_shm = shared_memory.SharedMemory(create=True, size=self.data_dummy.size * self.data_dummy.itemsize)
        self.shm_lock = mp.Lock()

    def exit(self):
        self.eExit.set()

    def get_exit_event(self):
        return self.eExit

    def is_exit_set(self):
        return self.eExit.is_set()

    def send_data(self, t, r):
        t_array = np.asarray(t)
        r_array = np.asarray(r)
        if self.shm_lock.acquire(block=True, timeout=1):
            out_buf = np.ndarray(self.data_dummy.shape, dtype=self.data_dummy.dtype, buffer=self.data_shm.buf)
            out_buf[1:4] = t_array[:]
            out_buf[4:] = r_array[:]
            self.shm_lock.release()

    def get_data(self):
        if not self.shm_lock.acquire(block=True, timeout=1):
            return None
        data_array = np.ndarray(self.data_dummy.shape, dtype=self.data_dummy.dtype, buffer=self.data_shm.buf)
        self.shm_lock.release()
        return data_array

