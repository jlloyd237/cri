# -*- coding: utf-8 -*-
""" RTDE proxy interface prevents the socket write buffer on the server from
growing too large by reading packages on a background thread when the client
is idle.
"""

import time
from threading import Thread, Lock

import cri.ur.rtde.rtde as rtde


class RTDEProxy(rtde.RTDE):
    """ RTDE proxy interface prevents the socket write buffer on the server
    from growing too large by reading packages on a background thread when
    the client is idle.
    
    When the socket write buffer on the server reaches its maximum size this
    appears to cause some package header misalignment on the sending side
    (possibly due to the sender not checking for partial writes to an
    almost-full socket buffer).
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._bg_receive_thread = None
        self._bg_receive_thread_running = False
        self._bg_receive_running = False
        self._receive_lock = Lock()
                
    def _bg_receive(self):
        while self._bg_receive_thread_running:
            if self._bg_receive_running:
                with self._receive_lock:
                    super().receive()               
            time.sleep(0.01)

    def disconnect(self):      
        self.send_pause()
        super().disconnect()

    def get_controller_version(self):
        bg_receive_status = self._bg_receive_running
        self._bg_receive_running = False
        with self._receive_lock:
            version = super().get_controller_version()
        self._bg_receive_running = bg_receive_status           
        return version
    
    def negotiate_protocol_version(self, protocol):
        bg_receive_status = self._bg_receive_running
        self._bg_receive_running = False
        with self._receive_lock:
            result = super().negotiate_protocol_version(protocol)
        self._bg_receive_running = bg_receive_status           
        return result
    
    def send_input_setup(self, variables, types=[]):
        bg_receive_status = self._bg_receive_running
        self._bg_receive_running = False
        with self._receive_lock:
            result = super().send_input_setup(variables, types)
        self._bg_receive_running = bg_receive_status           
        return result

    def send_output_setup(self, variables, types=[]):
        bg_receive_status = self._bg_receive_running
        self._bg_receive_running = False
        with self._receive_lock:
            result = super().send_output_setup(variables, types)
        self._bg_receive_running = bg_receive_status           
        return result        
        
    def send_start(self):
        self._bg_receive_running = False
        with self._receive_lock:
            response = super().send_start()      
        self._bg_receive_running = True        
        if not self._bg_receive_thread_running:
            self._bg_receive_thread_running = True
            self._bg_receive_thread = Thread(target=self._bg_receive, args=())
            self._bg_receive_thread.start()
        return response
    
    def send_pause(self):
        self._bg_receive_running = False
        if self._bg_receive_thread_running:
            self._bg_receive_thread_running = False
            self._bg_receive_thread.join()
        with self._receive_lock:
            response = super().send_pause()
        return response

    def receive(self):
        bg_receive_status = self._bg_receive_running
        self._bg_receive_running = False
        with self._receive_lock:
            output_data = super().receive()
        self._bg_receive_running = bg_receive_status           
        return output_data              
    