"""
onrobot.py
OnRobot RG2 그리퍼 Modbus TCP 제어 모듈.
레퍼지토리 공통 규약: 192.168.1.1:502, unit=65.
ROS2/DSR 노드가 아니라 순수 Modbus 유틸 (두산 코딩 규칙 영향 없음).
pymodbus 미설치/하드웨어 미연결 환경에서도 import 가 깨지지 않도록 방어.
"""

import time


class RG2:
    def __init__(self, ip="192.168.1.1", port=502, unit=65):
        self.ip = ip
        self.port = port
        self.unit = unit
        self.client = None
        self._connected = False
        self._connect()

    def _connect(self):
        try:
            from pymodbus.client.sync import ModbusTcpClient
            self.client = ModbusTcpClient(self.ip, port=self.port)
            self._connected = self.client.connect()
            if not self._connected:
                print(f"[RG2] 연결 실패: {self.ip}:{self.port}")
        except Exception as e:
            print(f"[RG2] pymodbus 미사용/연결 불가: {e}")
            self._connected = False

    @property
    def connected(self):
        return self._connected

    def _write(self, address, values):
        if not self._connected:
            print(f"[RG2] (dry-run) write addr={address} val={values}")
            return False
        try:
            self.client.write_registers(address, values, slave=self.unit)
            return True
        except Exception as e:
            print(f"[RG2] write 실패: {e}")
            return False

    def move_gripper(self, width_mm, force=400):
        """
        목표 폭(mm)과 힘(0.1N 단위)으로 그리퍼 이동.
        RG2 레지스터: 0=target force, 1=target width, 2=control(1=move)
        """
        width_val = int(max(0, min(1100, width_mm * 10)))   # 0.1mm 단위
        force_val = int(max(0, min(400, force)))
        self._write(0, [force_val, width_val, 1])
        time.sleep(0.5)

    def open(self, width_mm=100, force=400):
        """그리퍼 열기 (place 동작)."""
        self.move_gripper(width_mm, force)

    def close(self, width_mm=10, force=400):
        """그리퍼 닫기 (grip 동작)."""
        self.move_gripper(width_mm, force)

    def release(self):
        if self.client is not None and self._connected:
            try:
                self.client.close()
            except Exception:
                pass
        self._connected = False
