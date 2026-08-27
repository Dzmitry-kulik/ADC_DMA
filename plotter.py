import collections
import json
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

SERIAL_PORT = '/dev/ttyUSB0'  # Windows: 'COM3'
BAUD_RATE = 115200
MAX_POINTS = 10  # Окно отображения в секундах
DEBUG_PRINTS = False

class TelemetryReceiver(threading.Thread):
    def __init__(self, port, baudrate, data_deque, lock):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_deque = data_deque
        self.lock = lock
        self.running = True
        self.start_time = time.time()

    def run(self):
        try:
            # Используем timeout для неблокирующего readline
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            ser.reset_input_buffer()
            print(f"[OK] Подключено к {self.port}")
        except Exception as e:
            print(f"[ERROR] Ошибка порта: {e}")
            return

        while self.running:
            try:
                line = ser.readline()
                if line:
                    decoded = line.decode('ascii', errors='ignore').strip()
                    if not decoded:
                        continue
                    
                    data = json.loads(decoded)
                    t = time.time() - self.start_time
                    
                    raw_val = data.get("raw", 0)
                    filt_val = data.get("filt", 0)
                    missed = data.get("missed", 0)

                    if DEBUG_PRINTS:
                        print(f"Raw: {raw_val}, Filt: {filt_val}, Missed: {missed}")

                    with self.lock:
                        self.data_deque.append((t, raw_val, filt_val, missed))
                        
            except json.JSONDecodeError:
                pass # Пропускаем "битые" строки при старте
            except Exception as e:
                time.sleep(0.1)

        ser.close()

    def stop(self):
        self.running = False

# ==============================================================================
# GUI & MATPLOTLIB ANIMATION
# ==============================================================================
data_buffer = collections.deque(maxlen=1000)
data_lock = threading.Lock()

receiver = TelemetryReceiver(SERIAL_PORT, BAUD_RATE, data_buffer, data_lock)
receiver.start()

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
fig.canvas.manager.set_window_title('ADC Signal & Overrun Monitor')

# График 1: Сигналы
line_raw, = ax1.plot([], [], 'r-', alpha=0.5, label='Raw Signal')
line_filt, = ax1.plot([], [], 'b-', linewidth=2, label='Filtered Signal')
ax1.set_ylabel('Amplitude')
ax1.grid(True)
ax1.legend(loc='upper right')

# График 2: Потерянные блоки
line_missed, = ax2.plot([], [], 'm-', drawstyle='steps-post', linewidth=2, label='Missed Blocks (Overrun)')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Count')
ax2.grid(True)
ax2.legend(loc='upper right')

def update(frame):
    with data_lock:
        if not data_buffer:
            return line_raw, line_filt, line_missed

        ts = [pt[0] for pt in data_buffer]
        raws = [pt[1] for pt in data_buffer]
        filts = [pt[2] for pt in data_buffer]
        misseds = [pt[3] for pt in data_buffer]

    current_time = ts[-1]
    min_time = max(0, current_time - MAX_POINTS)

    filtered_indices = [i for i, t in enumerate(ts) if t >= min_time]
    vis_ts = [ts[i] for i in filtered_indices]
    
    line_raw.set_data(vis_ts, [raws[i] for i in filtered_indices])
    line_filt.set_data(vis_ts, [filts[i] for i in filtered_indices])
    line_missed.set_data(vis_ts, [misseds[i] for i in filtered_indices])

    ax1.set_xlim(min_time, max(2, current_time + 0.1))
    ax2.set_xlim(min_time, max(2, current_time + 0.1))

    # Динамический масштаб амплитуды
    if raws:
        ax1.set_ylim(min(raws)*0.9, max(raws)*1.1)
    
    # Масштаб для счетчика потерь
    if misseds:
        ax2.set_ylim(-0.5, max(10, max(misseds) * 1.2))

    return line_raw, line_filt, line_missed

def on_close(event):
    receiver.stop()

fig.canvas.mpl_connect('close_event', on_close)
ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.tight_layout()
plt.show()