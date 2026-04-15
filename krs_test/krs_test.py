import tkinter as tk
from tkinter import ttk, filedialog
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial
import csv

PORT = "COM9"
BAUD = 115200

class KrsTestApp:
    def __init__(self, root):
        self.root = root
        self.root.title("KRS Servo Test")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.ser = serial.Serial(PORT, BAUD, timeout=0.05)

        self.times = []
        self.positions = []
        self.recording = False
        self.target = None

        self.build_gui()
        self.poll_id = None
        self.graph_id = None

    def build_gui(self):
        top = ttk.Frame(self.root, padding=5)
        top.pack(fill=tk.X)

        self.cmd_type = tk.StringVar(value="e")
        cb = ttk.Combobox(top, textvariable=self.cmd_type, values=["e", "r"], width=2, state="readonly")
        cb.pack(side=tk.LEFT, padx=3)

        ttk.Label(top, text="値:").pack(side=tk.LEFT)
        self.val_entry = ttk.Entry(top, width=8)
        self.val_entry.insert(0, "7500")
        self.val_entry.pack(side=tk.LEFT, padx=3)
        self.val_entry.bind("<Return>", lambda e: self.send_command())

        self.send_btn = ttk.Button(top, text="送信", command=self.send_command)
        self.send_btn.pack(side=tk.LEFT, padx=3)

        ttk.Label(top, text="ステップ数:").pack(side=tk.LEFT, padx=(10, 0))
        self.step_var = tk.IntVar(value=10)
        self.step_entry = ttk.Entry(top, textvariable=self.step_var, width=5)
        self.step_entry.pack(side=tk.LEFT, padx=3)

        self.plus_btn = ttk.Button(top, text="+", width=2, command=self.step_up)
        self.plus_btn.pack(side=tk.LEFT, padx=1)
        self.minus_btn = ttk.Button(top, text="-", width=2, command=self.step_down)
        self.minus_btn.pack(side=tk.LEFT, padx=1)

        ttk.Label(top, text="閾値:").pack(side=tk.LEFT, padx=(15, 0))
        self.threshold_var = tk.IntVar(value=20)
        self.threshold_spin = ttk.Spinbox(top, from_=1, to=500, width=6,
                                          textvariable=self.threshold_var)
        self.threshold_spin.pack(side=tk.LEFT, padx=3)

        self.status_var = tk.StringVar(value="Idle")
        ttk.Label(top, textvariable=self.status_var, foreground="blue",
                  font=("", 10, "bold")).pack(side=tk.RIGHT, padx=10)

        self.fig = Figure(figsize=(7, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("Position")
        self.ax.set_title("Servo Step Response")
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        bottom = ttk.Frame(self.root, padding=5)
        bottom.pack(fill=tk.X)

        self.save_btn = ttk.Button(bottom, text="CSV保存", command=self.save_csv,
                                   state=tk.DISABLED)
        self.save_btn.pack(side=tk.LEFT)

    def step_up(self):
        self._apply_step(1)

    def step_down(self):
        self._apply_step(-1)

    def _apply_step(self, sign):
        try:
            current_val = int(self.val_entry.get().strip())
            step_val = self.step_var.get()
        except ValueError:
            self.status_var.set("Error: invalid number")
            return
            
        new_val = current_val + (sign * step_val)
        self.val_entry.delete(0, tk.END)
        self.val_entry.insert(0, str(new_val))
        self.send_command()

    def send_command(self):
        try:
            val = int(self.val_entry.get().strip())
        except ValueError:
            self.status_var.set("Error: invalid number")
            return

        c = self.cmd_type.get()
        cmd = f"{c}{val}"

        # 前回のストリーミング停止
        if self.recording:
            try:
                self.ser.write(b"s\n")
            except Exception:
                pass

        self.times.clear()
        self.positions.clear()
        self.target = val
        self.recording = True
        self.save_btn.config(state=tk.DISABLED)
        self.status_var.set("Recording...")

        try:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\n").encode())
        except Exception:
            self.status_var.set("Serial ERROR")
            return

        self.start_polling()
        self.start_graph_update()

    def start_polling(self):
        if self.poll_id:
            self.root.after_cancel(self.poll_id)
        self.poll_serial()

    def poll_serial(self):
        try:
            # GUIがフリーズしないよう1ループ最大100行までに制限
            reads = 0
            while self.ser.in_waiting and reads < 100:
                raw = self.ser.readline()
                if not raw:
                    break
                reads += 1
                line = raw.decode("utf-8", errors="replace").strip()
                if line.startswith("D:") and self.recording:
                    parts = line[2:].split(",")
                    if len(parts) == 2:
                        t = int(parts[0])
                        pos = int(parts[1])
                        self.times.append(t)
                        self.positions.append(pos)

                        if self.check_settled():
                            self.stop_recording()
        except Exception:
            pass

        # 5秒強制停止のチェック
        if self.recording and self.times and self.times[-1] > 5000:
            self.stop_recording()

        # 常にバックグラウンドでバッファを空にするためポーリングは継続する
        interval = 20 if self.recording else 100
        self.poll_id = self.root.after(interval, self.poll_serial)

    def start_graph_update(self):
        if self.graph_id:
            self.root.after_cancel(self.graph_id)
        self.update_graph()

    def update_graph(self):
        self.ax.clear()
        self.ax.set_xlabel("Time (ms)")
        self.ax.set_ylabel("Position")
        self.ax.set_title("Servo Step Response")

        if self.times:
            self.ax.plot(self.times, self.positions, "b-", linewidth=1)
            if self.target is not None:
                self.ax.axhline(y=self.target, color="r", linestyle="--",
                                label=f"Target: {self.target}")
                self.ax.legend(loc="upper right")

        self.canvas.draw_idle()

        if self.recording:
            self.graph_id = self.root.after(100, self.update_graph)

    def check_settled(self):
        n = len(self.positions)
        if n < 15:
            return False
        window = self.positions[-10:]
        return (max(window) - min(window)) < self.threshold_var.get()

    def stop_recording(self):
        self.recording = False
        try:
            self.ser.write(b"s\n")
        except Exception:
            pass
        self.status_var.set("Settled")
        self.save_btn.config(state=tk.NORMAL)
        # 最終グラフ更新
        self.update_graph()

    def save_csv(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv")],
            initialfile=f"krs_response.csv"
        )
        if not path:
            return
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_ms", "position"])
            for t, p in zip(self.times, self.positions):
                writer.writerow([t, p])
        self.status_var.set(f"Saved: {path}")

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
    app = KrsTestApp(root)
    root.mainloop()
