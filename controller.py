"""
Switch Pro Controller interface for ESP32 via UART (COM4).

Usage:
    con = SwitchController("COM4")
    con.press("a")          # tap A button
    con.press("w", 0.5)     # hold Up 500ms
    con.press(["a", "b"])   # tap A+B simultaneously
    con.close()

Key mapping (matches uart_cmd_task in main.c):
    Face:    a / b / x / y
    Shoulder: l / r / z(ZL) / c(ZR)
    System:  + / - / h(HOME) / p(Capture)
    D-pad:   w(Up) s(Down) q(Left) e(Right)
"""

import serial
import time
import threading


class SwitchController:
    def __init__(self, port: str = "COM4", baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)

    # ------------------------------------------------------------------ #
    # Low-level                                                            #
    # ------------------------------------------------------------------ #

    def send(self, key: str):
        """Send a single key character immediately (does not wait)."""
        self.ser.write(key.encode())

    # ------------------------------------------------------------------ #
    # High-level                                                           #
    # ------------------------------------------------------------------ #

    def press(self, keys, hold_sec: float = 0.08, gap_sec: float = 0.05):
        """
        Press one key or a list of keys simultaneously.

        ESP32's uart_cmd_task handles timing on its side (80ms tap),
        but for longer holds we send the key, wait, then send a neutral
        character that the firmware ignores (space).
        """
        if isinstance(keys, str):
            keys = [keys]

        # Send all keys at once (the firmware taps each 80ms on receipt)
        # For a longer hold, send the key every ~16ms until hold_sec passes.
        if hold_sec <= 0.1:
            for k in keys:
                self.ser.write(k.encode())
        else:
            deadline = time.perf_counter() + hold_sec
            while time.perf_counter() < deadline:
                for k in keys:
                    self.ser.write(k.encode())
                time.sleep(0.016)

        time.sleep(gap_sec)

    def wait(self, seconds: float):
        time.sleep(seconds)

    def close(self):
        self.ser.close()

    # ------------------------------------------------------------------ #
    # Context manager                                                      #
    # ------------------------------------------------------------------ #

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
