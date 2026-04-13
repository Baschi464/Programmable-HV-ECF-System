import serial
import time
import logging
from typing import Iterable

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

RELAY_COUNT = 8
DEFAULT_CHANNEL_MAP = [1, 1, 1, 1, 1, 1, 2, 2]
HV_MIN_VOLTAGE = 0.0
HV_MAX_VOLTAGE = 6000.0


def build_control_frame(relays: Iterable[int], hv1: float, hv2: float) -> str:
    """Builds the protocol frame: <r1,r2,r3,r4,r5,r6,r7,r8,v1,v2>."""
    values = list(relays)
    if len(values) < RELAY_COUNT:
        values = values + [0] * (RELAY_COUNT - len(values))
    values = values[:RELAY_COUNT]

    normalized_relays = []
    for item in values:
        try:
            relay = 1 if int(item) != 0 else 0
        except (TypeError, ValueError):
            relay = 0
        normalized_relays.append(str(relay))

    try:
        hv1_value = float(hv1)
    except (TypeError, ValueError):
        hv1_value = 0.0

    try:
        hv2_value = float(hv2)
    except (TypeError, ValueError):
        hv2_value = 0.0

    hv1_value = max(HV_MIN_VOLTAGE, min(HV_MAX_VOLTAGE, hv1_value))
    hv2_value = max(HV_MIN_VOLTAGE, min(HV_MAX_VOLTAGE, hv2_value))

    return f"<{','.join(normalized_relays)},{hv1_value:.1f},{hv2_value:.1f}>"


def build_mapping_frame(channel_map: Iterable[int]) -> str:
    """Builds the protocol frame: <MAP,m1,m2,m3,m4,m5,m6,m7,m8>."""
    values = list(channel_map)
    if len(values) < RELAY_COUNT:
        values = values + DEFAULT_CHANNEL_MAP[len(values):]
    values = values[:RELAY_COUNT]

    normalized = []
    for i, item in enumerate(values):
        try:
            mapped = int(item)
        except (TypeError, ValueError):
            mapped = DEFAULT_CHANNEL_MAP[i]
        if mapped not in (1, 2):
            mapped = DEFAULT_CHANNEL_MAP[i]
        normalized.append(str(mapped))

    return f"<MAP,{','.join(normalized)}>"


class SerialCommunication:
    def __init__(self, port, baudrate=115200, timeout=1, mapping_on_connect=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        if mapping_on_connect is None:
            self.mapping_on_connect = list(DEFAULT_CHANNEL_MAP)
        else:
            self.mapping_on_connect = list(mapping_on_connect)
        # Set to True when the OS/driver reports the device is gone (e.g. USB unplugged).
        # In that case we stop trying to reconnect automatically and let the GUI handle it.
        self.disconnected = False
        self.disconnected_reason = None
        self.connect()

    def connect(self):
        """Attempts to establish a serial connection."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # Wait for Arduino reset
            self.disconnected = False
            self.disconnected_reason = None
            logging.info(f"Connected to {self.port} at {self.baudrate}")

            # Send one mapping frame immediately after connect.
            self.send_mapping_update(self.mapping_on_connect)
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Failed to connect to {self.port}: {e}")
            self.ser = None

    def _mark_disconnected(self, error):
        """Marks the connection as disconnected and closes the serial port."""
        if self.disconnected:
            return
        self.disconnected = True
        self.disconnected_reason = str(error)
        try:
            self.close()
        except Exception:
            pass
        self.ser = None

    def _ensure_connection(self):
        """Checks if the connection is open.

        Note: If the device has been unplugged (disconnected=True), we do NOT auto-reconnect.
        The GUI should prompt the user to reconnect explicitly.
        """
        if self.disconnected:
            return False
        return self.ser is not None and self.ser.is_open

    def send_command(self, command):
        """Sends a command string safely."""
        if not self._ensure_connection(): return False
        try:
            # Debug: show every outgoing command (target pressures: "<10.0,0.0,...>")
            #print(f"[TX] {command}", flush=True)
            #logging.info(f"TX: {command}")
            self.ser.write(command.encode('utf-8'))
            return True
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Write error: {e}")
            self._mark_disconnected(e)
            return False

    def send_mapping_update(self, channel_map: Iterable[int]) -> bool:
        """Sends the channel-to-HV mapping frame (once on connect, or when edited)."""
        return self.send_command(build_mapping_frame(channel_map))

    def read_response(self):
        """Reads a line from serial if available, handling errors."""
        if not self._ensure_connection(): return None
        try:
            if self.ser.in_waiting > 0:
                return self.ser.readline().decode('utf-8', errors='ignore').strip()
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Read error: {e}")
            self._mark_disconnected(e)
        return None

    def read_latest_response(self):
        """
        Reads ALL waiting lines in the buffer and returns only the most recent one.
        Use this for plotting to eliminate lag.
        """
        if not self._ensure_connection(): return None
        last_line = None
        try:
            # Check if there is data in the buffer
            if self.ser.in_waiting > 0:
                # Loop to drain the buffer (read everything available)
                # We limit to 100 iterations to prevent hanging if Arduino sends faster than we can read
                for _ in range(100): 
                    if self.ser.in_waiting == 0:
                        break
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        last_line = line
            return last_line
        except (serial.SerialException, OSError, Exception) as e:
            logging.error(f"Read error: {e}")
            self._mark_disconnected(e)
        return None

    def close(self):
        if self.ser and getattr(self.ser, 'is_open', False):
            self.ser.close()
            logging.info("Serial connection closed.")