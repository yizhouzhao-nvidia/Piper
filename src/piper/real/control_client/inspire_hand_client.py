import socket
import time
from contextlib import contextmanager

import threading
import numpy as np

from .base_client import BaseClient

class InspireHandClient(BaseClient):
    def __init__(
            self,
            left_hand_ip='192.168.137.19',
            right_hand_ip='192.168.137.39',
            use_rad=True,
        ):
        super().__init__()
        self.port = 2333
        self.ips = [left_hand_ip, right_hand_ip]
        self.latest_pos = {ip: None for ip in self.ips}
        self.running = False

        set_pos_send_data = bytearray()
        set_pos_send_data.append(0xEB)  # 包头
        set_pos_send_data.append(0x90)  # 包头
        set_pos_send_data.append(1)  # 灵巧手 ID 号
        set_pos_send_data.append(0x17)
        set_pos_send_data.append(0x20)

        get_pos_send_data = bytearray()
        get_pos_send_data.append(0xEB)  # 包头
        get_pos_send_data.append(0x90)  # 包头
        get_pos_send_data.append(1)
        get_pos_send_data.append(0x04)
        get_pos_send_data.append(0x11)  # kCmd_Handg3_Read
        get_pos_send_data.append(0x0A)
        get_pos_send_data.append(0x06)
        get_pos_send_data.append(0x0C)
        get_pos_send_data.append(sum(get_pos_send_data[2:8]) & 0xFF)

        self.set_pos_send_data = set_pos_send_data
        self.get_pos_send_data = get_pos_send_data

        self.use_rad = use_rad
        self.joint_limits = np.deg2rad(np.array([93., 93., 93., 93., 39., 74.]))

    def receive_loop(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)  # Buffer size of 1024 bytes
                if len(data) != 20:
                    continue
                pos = [
                    data[7] | (data[8] << 8),
                    data[9] | (data[10] << 8),
                    data[11] | (data[12] << 8),
                    data[13] | (data[14] << 8),
                    data[15] | (data[16] << 8),
                    data[17] | (data[18] << 8),
                ]
                with self.lock:
                    if addr[0] in self.ips:
                        self.latest_pos[addr[0]] = pos
            except Exception as e:
                print(f"Receive error: {e}")

    def move(self, cmds):
        cmds = np.asarray(cmds)
        assert len(cmds) == 12
        for idx, ip in enumerate(self.ips):
            positions_rad = cmds[idx * 6: (idx + 1) * 6]
            if self.use_rad:
                positions = np.clip(1.0 - positions_rad / self.joint_limits, 0, 1) * 1000
            else:
                positions = np.clip(positions_rad, 0, 1000)

            curr_set_pos_send_data = self.set_pos_send_data.copy()

            positions = [int(pos) for pos in positions]            
            for pos in positions:
                curr_set_pos_send_data.append(pos & 0xFF)
                curr_set_pos_send_data.append((pos >> 8) & 0xFF)
            
            force = 1000
            for _ in positions[1:]:
                curr_set_pos_send_data.append(force & 0xFF)
                curr_set_pos_send_data.append((force >> 8) & 0xFF)
            
            curr_set_pos_send_data.append(sum(curr_set_pos_send_data[2:27]) & 0xFF)
            self.socket.sendto(curr_set_pos_send_data, (ip, self.port))

    def get(self):
        for ip in self.ips:
            self.socket.sendto(self.get_pos_send_data, (ip, self.port))

        current_pos = [None] * len(self.ips)
        while None in current_pos:
            time.sleep(0.005)
            with self.lock:
                current_pos = [self.latest_pos[ip] for ip in self.ips]
        
        positions = np.asarray(current_pos)
        if self.use_rad:
            positions = (1000 - positions) / 1000 * self.joint_limits
        
        return positions.flatten()

    @contextmanager
    def activate(self):
        try:
            self.running = True
            self.lock = threading.Lock()
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(('0.0.0.0', 0))  # Bind to any available port
            self.thread = threading.Thread(target=self.receive_loop)
            self.thread.start()
            yield self
        finally:
            self.running = False
            self.thread.join()
            self.socket.close()

    @property
    def cmd_shape(self):
        return (12,)

    @property
    def state_shape(self):
        return (12,)
