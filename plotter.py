import collections
import struct
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
SERIAL_PORT = '/dev/ttyUSB0'  # Windows: 'COM3'
BAUD_RATE = 115200
MAX_POINTS = 60  # Time window to keep in seconds
DEBUG_PRINTS = True  # Print parsed frames summary to console
MAX_PAYLOAD_LEN = 128  # Maximum expected payload size to prevent buffer hangs
def calculate_crc16(data: bytes) -> int:
    """CRC16 CCITT (Poly: 0x1021, Init: 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def send_ack(ser: serial.Serial, seq_num: int):
    """Send ACK frame back to STM32"""
    header = struct.pack('>BBBH', 0x01, 0x02, seq_num, 0)
    crc = calculate_crc16(header)
    ack_frame = b'\xAA\x55' + header + struct.pack('>H', crc)
    try:
        ser.write(ack_frame)
    except Exception as e:
        print(f"[ACK ERROR] Failed to send ACK: {e}")


class TelemetryReceiver(threading.Thread):
    """High-performance background UART receiver using buffer matching"""

    def __init__(self, port: str, baudrate: int, data_deque: collections.deque, lock: threading.Lock):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_deque = data_deque
        self.lock = lock
        self.running = True
        self.ser = None
        self.start_time = time.time()

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
            self.ser.reset_input_buffer()  # Clear garbage bytes left in UART buffer
            print(f"[OK] Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"[ERROR] Could not open port {self.port}: {e}")
            return False

    def run(self):
        if not self.connect():
            return

        rx_buffer = bytearray()

        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    rx_buffer.extend(self.ser.read(self.ser.in_waiting))
                else:
                    time.sleep(0.002)
                    continue
            except Exception as e:
                if self.running:
                    print(f"[UART ERROR] Read failed: {e}")
                time.sleep(0.1)
                continue

            # Process buffer for valid frames (Preamble 0xAA 0x55)
            while len(rx_buffer) >= 16:  # Min length for DATA frame
                sync_idx = rx_buffer.find(b'\xAA\x55')

                if sync_idx == -1:
                    # Keep last byte in case 0xAA is partially received
                    rx_buffer = rx_buffer[-1:] if rx_buffer else bytearray()
                    break

                if sync_idx > 0:
                    # Discard unaligned garbage bytes
                    del rx_buffer[:sync_idx]

                if len(rx_buffer) < 7:
                    break  # Wait for full header

                # Unpack Header
                version, msg_type, seq_num, length = struct.unpack('>BBBH', rx_buffer[2:7])

                # Validate payload length against threshold to prevent hangs
                if length > MAX_PAYLOAD_LEN:
                    del rx_buffer[:2]  # Discard invalid preamble
                    continue

                total_frame_len = 2 + 5 + length + 2

                if len(rx_buffer) < total_frame_len:
                    break  # Wait for full payload and CRC to arrive

                frame = rx_buffer[:total_frame_len]

                # Validate CRC (Header + Payload)
                payload = frame[7:7 + length]
                received_crc = struct.unpack('>H', frame[7 + length:total_frame_len])[0]
                calculated_crc = calculate_crc16(frame[2:7 + length])

                if received_crc != calculated_crc:
                    print(f"[WARN] CRC Mismatch! Calc: 0x{calculated_crc:04X}, Recv: 0x{received_crc:04X}")
                    del rx_buffer[:2]  # Only delete sync bytes to resynchronize
                    continue

                # Frame validated successfully -> consume from buffer
                del rx_buffer[:total_frame_len]

                # ИЗМЕНЕНО: Теперь длина пакета 8 байт (добавлен 1 байт btn_event)
                if length == 8:
                    # Распаковка: I (4 байта freq), B (1 байт duty), H (2 байта isr), B (1 байт btn_event)
                    freq_hz, duty_pct, isr_ticks, btn_event = struct.unpack('<IBHB', payload)
                    send_ack(self.ser, seq_num)

                    t = time.time() - self.start_time
                    isr_us = isr_ticks / 16.0

                    if DEBUG_PRINTS:
                        # Текстовое представление режима кнопки
                        event_names = ["IDLE", "SHORT", "DOUBLE", "LONG"]
                        event_str = event_names[btn_event] if btn_event < len(event_names) else f"UNKNOWN({btn_event})"

                        print(f"--> [PARSED] Freq: {freq_hz:5} Hz | Duty: {duty_pct:3}% | "
                              f"ISR: {isr_ticks:4} ticks ({isr_us:5.2f} us) | EVENT: {event_str}")

                    with self.lock:
                        self.data_deque.append((t, freq_hz, duty_pct, isr_ticks, btn_event))

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


# ==============================================================================
# GUI & MATPLOTLIB ANIMATION
# ==============================================================================
data_buffer = collections.deque(maxlen=3000)  # Stores up to 60 seconds at 50 Hz
data_lock = threading.Lock()

receiver = TelemetryReceiver(SERIAL_PORT, BAUD_RATE, data_buffer, data_lock)
receiver.start()

# ИЗМЕНЕНО: Добавлен 4-й график для кнопки (4, 1)
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
fig.canvas.manager.set_window_title('STM32 PWM, ISR & FSM Profiling')

line_freq, = ax1.plot([], [], 'r-o', label='PWM Frequency (Hz)', linewidth=1.5, markersize=3)
ax1.set_ylabel('Frequency [Hz]')
ax1.grid(True)
ax1.legend(loc='upper left')

line_duty, = ax2.plot([], [], 'b-s', label='PWM Duty Cycle (%)', linewidth=1.5, markersize=3)
ax2.set_ylabel('Duty Cycle [%]')
ax2.grid(True)
ax2.legend(loc='upper left')

line_isr, = ax3.plot([], [], 'g-^', label='ISR Duration (TIM1 Ticks)', linewidth=1.5, markersize=3)
ax3.set_ylabel('Ticks [62.5 ns]')
ax3.grid(True)
ax3.legend(loc='upper left')

# ДОБАВЛЕНО: График состояния кнопки со "ступеньками" (steps-post)
line_btn, = ax4.plot([], [], 'm-', drawstyle='steps-post', label='Button FSM State', linewidth=2)
ax4.set_xlabel('Time [s]')
ax4.set_ylabel('State')
ax4.set_yticks([0, 1, 2, 3])
ax4.set_yticklabels(["IDLE", "SHORT", "DOUBLE", "LONG"])
ax4.set_ylim(-0.5, 3.5)
ax4.grid(True)
ax4.legend(loc='upper left')


def update(frame):
    with data_lock:
        if not data_buffer:
            return line_freq, line_duty, line_isr, line_btn

        ts = [pt[0] for pt in data_buffer]
        freqs = [pt[1] for pt in data_buffer]
        duties = [pt[2] for pt in data_buffer]
        isrs = [pt[3] for pt in data_buffer]
        btns = [pt[4] for pt in data_buffer]  # Распаковка 4-го параметра

    current_time = ts[-1]
    min_time = max(0, current_time - MAX_POINTS)

    # Filter data points within current time window
    filtered_indices = [i for i, t in enumerate(ts) if t >= min_time]

    vis_ts = [ts[i] for i in filtered_indices]
    vis_freqs = [freqs[i] for i in filtered_indices]
    vis_duties = [duties[i] for i in filtered_indices]
    vis_isrs = [isrs[i] for i in filtered_indices]
    vis_btns = [btns[i] for i in filtered_indices]

    line_freq.set_data(vis_ts, vis_freqs)
    line_duty.set_data(vis_ts, vis_duties)
    line_isr.set_data(vis_ts, vis_isrs)
    line_btn.set_data(vis_ts, vis_btns)

    # Update X-axis limits for all 4 subplots
    for ax in (ax1, ax2, ax3, ax4):
        ax.set_xlim(min_time, max(2, current_time + 0.5))

    # Auto-scale Y-axis for Frequency
    if vis_freqs:
        min_f, max_f = min(vis_freqs), max(vis_freqs)
        if min_f == max_f:
            ax1.set_ylim(max(0, min_f - 100), min_f + 100)
        else:
            margin = max(10.0, (max_f - min_f) * 0.1)
            ax1.set_ylim(max(0, min_f - margin), max_f + margin)

    # Fixed Y-axis for Duty Cycle
    ax2.set_ylim(-5, 105)

    # Auto-scale Y-axis for ISR Ticks
    if vis_isrs:
        max_isr = max(vis_isrs)
        ax3.set_ylim(-1, max(20, max_isr * 1.25))

    return line_freq, line_duty, line_isr, line_btn


def on_close(event):
    receiver.stop()


fig.canvas.mpl_connect('close_event', on_close)

ani = FuncAnimation(fig, update, interval=40, cache_frame_data=False)  # Redraw at 25 FPS

plt.tight_layout()
plt.show()
