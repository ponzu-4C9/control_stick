import tkinter as tk
from tkinter import ttk, filedialog
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams["font.family"] = "MS Gothic"
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import csv
import datetime

PORT = "COM11"
BAUD = 115200

SETTLE_WINDOW = 15
SETTLE_THRESHOLD = 0.5  # pitch degrees


class DakakuMappingApp:
    def __init__(self, root):
        self.root = root
        self.root.title("舵角マッピング")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.ser = serial.Serial(PORT, BAUD, timeout=0.05)

        # CSVに蓄積するデータ
        self.csv_cmds = []
        self.csv_pitches = []

        # 現在のストリーミング（グラフ用）
        self.times = []
        self.pitches = []
        self.current_target = None
        self.recording = False

        self.build_gui()
        self.poll_id = None
        self.graph_id = None
        self.poll_serial()

    def build_gui(self):
        top = ttk.Frame(self.root, padding=5)
        top.pack(fill=tk.X)

        ttk.Label(top, text="サーボ:").pack(side=tk.LEFT)
        self.servo_type = tk.StringVar(value="e")
        ttk.Combobox(top, textvariable=self.servo_type,
                     values=["e", "r"], width=2, state="readonly").pack(side=tk.LEFT, padx=3)

        ttk.Label(top, text="現在値:").pack(side=tk.LEFT, padx=(10, 0))
        self.val_var = tk.IntVar(value=7500)
        self.val_entry = ttk.Entry(top, textvariable=self.val_var, width=7)
        self.val_entry.pack(side=tk.LEFT, padx=3)
        self.val_entry.bind("<Return>", lambda e: self.send_current())

        ttk.Button(top, text="送信", command=self.send_current).pack(side=tk.LEFT, padx=3)

        ttk.Label(top, text="ステップ:").pack(side=tk.LEFT, padx=(15, 0))
        self.step_var = tk.IntVar(value=200)
        ttk.Entry(top, textvariable=self.step_var, width=6).pack(side=tk.LEFT, padx=3)

        self.minus_btn = ttk.Button(top, text="-", width=3, command=self.step_down)
        self.minus_btn.pack(side=tk.LEFT, padx=2)
        self.plus_btn = ttk.Button(top, text="+", width=3, command=self.step_up)
        self.plus_btn.pack(side=tk.LEFT, padx=2)

        # ステータス行
        status_frame = ttk.Frame(self.root, padding=5)
        status_frame.pack(fill=tk.X)

        self.status_var = tk.StringVar(value="待機中")
        ttk.Label(status_frame, textvariable=self.status_var, foreground="blue",
                  font=("", 10, "bold")).pack(side=tk.LEFT)

        self.pitch_var = tk.StringVar(value="Pitch: ---")
        ttk.Label(status_frame, textvariable=self.pitch_var,
                  font=("", 10)).pack(side=tk.LEFT, padx=20)

        self.count_var = tk.StringVar(value="記録: 0点")
        ttk.Label(status_frame, textvariable=self.count_var).pack(side=tk.RIGHT)

        # グラフ（時系列: X=時刻, Y=ピッチ角度）
        self.fig = Figure(figsize=(7, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self._init_ax()
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 下部
        bottom = ttk.Frame(self.root, padding=5)
        bottom.pack(fill=tk.X)

        self.save_btn = ttk.Button(bottom, text="CSV保存", command=self.save_csv,
                                   state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT)

        ttk.Button(bottom, text="最後を取消", command=self.undo_last).pack(side=tk.LEFT, padx=10)

        ttk.Button(bottom, text="CSVクリア", command=self.clear_data).pack(side=tk.LEFT, padx=3)

    def _init_ax(self):
        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("ピッチ角度 (deg)")
        self.ax.set_title("ステップ応答")

    def send_current(self):
        val = self.val_var.get()
        val = max(3500, min(11500, val))
        self.val_var.set(val)
        self._send_servo(val)

    def step_up(self):
        new_val = self.val_var.get() + self.step_var.get()
        new_val = min(11500, new_val)
        self.val_var.set(new_val)
        self._send_servo(new_val)

    def step_down(self):
        new_val = self.val_var.get() - self.step_var.get()
        new_val = max(3500, new_val)
        self.val_var.set(new_val)
        self._send_servo(new_val)

    def _send_servo(self, val):
        if self.recording:
            try:
                self.ser.write(b"s\n")
            except Exception:
                pass

        self.times.clear()
        self.pitches.clear()
        self.current_target = val
        self.recording = True
        self.status_var.set(f"移動中... cmd={val}")

        cmd = f"{self.servo_type.get()}{val}\n"
        try:
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode())
        except Exception as e:
            self.status_var.set(f"Serial ERROR: {e}")
            self.recording = False
            return

        self.start_graph_update()

    def poll_serial(self):
        try:
            reads = 0
            while self.ser.in_waiting and reads < 100:
                raw = self.ser.readline()
                if not raw:
                    break
                reads += 1
                line = raw.decode("utf-8", errors="replace").strip()

                if line.startswith("D:") and self.recording:
                    parts = line[2:].split(",")
                    if len(parts) >= 3:
                        t = int(parts[0])
                        pitch = float(parts[2])
                        self.times.append(t)
                        self.pitches.append(pitch)
                        self.pitch_var.set(f"Pitch: {pitch:.2f}°")

                        if self.check_settled():
                            self.on_settled()

        except Exception:
            pass

        # 5秒タイムアウト
        if self.recording and self.times and self.times[-1] > 5000:
            self.on_settled()

        interval = 20 if self.recording else 100
        self.poll_id = self.root.after(interval, self.poll_serial)

    def check_settled(self):
        n = len(self.pitches)
        if n < SETTLE_WINDOW:
            return False
        window = self.pitches[-SETTLE_WINDOW:]
        return (max(window) - min(window)) < SETTLE_THRESHOLD

    def on_settled(self):
        """収束検出 → 自動停止 & CSV追記"""
        self.recording = False
        try:
            self.ser.write(b"s\n")
        except Exception:
            pass

        # 収束値 = 直近の平均
        recent = self.pitches[-SETTLE_WINDOW:]
        settled_pitch = sum(recent) / len(recent)

        self.csv_cmds.append(self.current_target)
        self.csv_pitches.append(settled_pitch)

        self.count_var.set(f"記録: {len(self.csv_cmds)}点")
        self.save_btn.config(state=tk.NORMAL)
        self.status_var.set(
            f"収束: cmd={self.current_target}, pitch={settled_pitch:.2f}°")

        # 最終グラフ更新
        self.update_graph()

    def start_graph_update(self):
        if self.graph_id:
            self.root.after_cancel(self.graph_id)
        self.update_graph()

    def update_graph(self):
        self.ax.clear()
        self._init_ax()

        if self.times:
            self.ax.plot(self.times, self.pitches, "b-", linewidth=1)
            if self.pitches:
                # 収束値の水平線
                if not self.recording and self.csv_pitches:
                    self.ax.axhline(y=self.csv_pitches[-1], color="r",
                                    linestyle="--",
                                    label=f"収束: {self.csv_pitches[-1]:.2f}°")
                    self.ax.legend(loc="upper right")
            self.ax.set_title(f"ステップ応答 (cmd={self.current_target})")

        self.canvas.draw_idle()

        if self.recording:
            self.graph_id = self.root.after(100, self.update_graph)

    def undo_last(self):
        if self.csv_cmds:
            self.csv_cmds.pop()
            self.csv_pitches.pop()
            self.count_var.set(f"記録: {len(self.csv_cmds)}点")
            if not self.csv_cmds:
                self.save_btn.config(state=tk.DISABLED)
            self.status_var.set("最後の記録を取消しました")

    def clear_data(self):
        self.csv_cmds.clear()
        self.csv_pitches.clear()
        self.count_var.set("記録: 0点")
        self.save_btn.config(state=tk.DISABLED)
        self.status_var.set("CSVデータをクリアしました")

    def save_csv(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"dakaku_mapping_{timestamp}.csv"
        )
        if not path:
            return
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["servo_command", "pitch_angle_deg"])
            for cmd, pitch in zip(self.csv_cmds, self.csv_pitches):
                writer.writerow([cmd, f"{pitch:.2f}"])
        self.status_var.set(f"保存: {path}")

    def on_close(self):
        if self.recording:
            try:
                self.ser.write(b"s\n")
            except Exception:
                pass
        if self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = DakakuMappingApp(root)
    root.mainloop()
