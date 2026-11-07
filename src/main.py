import math
import time
import network
import usocket as socket
import json
import os
from machine import ADC, PWM, Pin

# Chân phần cứng và tham số hệ thống
ADC_PH_PIN = 2
PWM_MOTOR_PIN = 5
PWM_PUMP_ACID_PIN = 6
PWM_PUMP_BASE_PIN = 7

UPDATE_INTERVAL_MS = 500
ADC_SAMPLE_COUNT = 10
ADC_SETTLING_DELAY_MS = 2000
PWM_FREQUENCY_HZ = 1000
MIXING_DURATION_MS = 5000
MAX_DOSING_CYCLES = 20
CSV_FILENAME = "data_log.csv"

# Các điểm hiệu chuẩn pH
PH_NEUTRAL = 6.8
PH_INTERMEDIATE = 6.63
PH_ACID = 1.85
PH_BASE = 8.29

VOLTAGE_PH_ACID = 3.37
VOLTAGE_PH7 = 2.67
VOLTAGE_PH_INTERMEDIATE = 2.57
VOLTAGE_PH_BASE = 2.3
ADC_DIVIDER_RATIO = 2.0


def ph_from_adc_millivolts(adc_voltage_mv):
    module_voltage = (adc_voltage_mv / 1000.0) * ADC_DIVIDER_RATIO

    acid_slope = (PH_ACID - PH_NEUTRAL) / (VOLTAGE_PH_ACID - VOLTAGE_PH7)
    intermediate_slope = (PH_INTERMEDIATE - PH_NEUTRAL) / (
        VOLTAGE_PH_INTERMEDIATE - VOLTAGE_PH7
    )
    base_slope = (PH_BASE - PH_INTERMEDIATE) / (
        VOLTAGE_PH_BASE - VOLTAGE_PH_INTERMEDIATE
    )

    if module_voltage >= VOLTAGE_PH7:
        return PH_NEUTRAL + (module_voltage - VOLTAGE_PH7) * acid_slope

    if module_voltage >= VOLTAGE_PH_INTERMEDIATE:
        return PH_NEUTRAL + (module_voltage - VOLTAGE_PH7) * intermediate_slope

    return PH_INTERMEDIATE + (module_voltage - VOLTAGE_PH_INTERMEDIATE) * base_slope


class CSVLogger:
    def __init__(self, filename=CSV_FILENAME):
        self.filename = filename
        try:
            # Mỗi lần khởi động bắt đầu một phiên ghi mới.
            with open(self.filename, "w") as f:
                f.write("timestamp_ms,ph,target_ph,pump_action,pulse_ms,kp,ki,kd\n")
        except Exception as e:
            print("Lỗi khởi tạo CSV Log:", e)

    def log(self, timestamp_ms, ph, target_ph, pump_action, pulse_ms, kp, ki, kd):
        try:
            with open(self.filename, "a") as f:
                f.write(
                    f"{timestamp_ms},{ph:.2f},{target_ph:.2f},{pump_action},"
                    f"{pulse_ms},{kp:.2f},{ki:.2f},{kd:.2f}\n"
                )
        except Exception as e:
            print("Lỗi ghi CSV:", e)

    def read_last_lines(self, lines=15):
        try:
            recent_lines = []
            with open(self.filename, "r") as f:
                for line in f:
                    recent_lines.append(line)
                    if len(recent_lines) > lines:
                        recent_lines.pop(0)
            return "".join(recent_lines)
        except Exception:
            return "Chưa có dữ liệu log."


class LightweightAIAdapter:
    def __init__(self, kp=1200.0, ki=15.0, kd=80.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.learning_rate = 0.08

    def adapt(self, current_error, prev_error, delta_ph):
        if current_error * prev_error < 0:
            self.kp *= 1.0 - self.learning_rate
            self.ki *= 0.5
        elif abs(delta_ph) < 0.08 and abs(current_error) > 0.3:
            self.kp *= 1.0 + self.learning_rate
            self.ki *= 1.0 + self.learning_rate * 0.5

        self.kp = max(200.0, min(self.kp, 4000.0))
        self.ki = max(0.0, min(self.ki, 200.0))
        self.kd = max(0.0, min(self.kd, 500.0))


class PHController:
    def __init__(self, target_ph=7.0):
        self.target_ph = target_ph
        self.adc = ADC(Pin(ADC_PH_PIN))
        self.adc.atten(ADC.ATTN_11DB)
        if hasattr(ADC, "WIDTH_12BIT") and hasattr(self.adc, "width"):
            self.adc.width(ADC.WIDTH_12BIT)

        self.pwm = {}
        for name, pin in (
            ("motor", PWM_MOTOR_PIN),
            ("acid", PWM_PUMP_ACID_PIN),
            ("base", PWM_PUMP_BASE_PIN),
        ):
            out = PWM(Pin(pin), freq=PWM_FREQUENCY_HZ)
            out.duty_u16(0)
            self.pwm[name] = out

        self.ai = LightweightAIAdapter()
        self.logger = CSVLogger()
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_ph = float("nan")

        self.system_enabled = True
        self.mode = "AUTO"
        self.manual_acid = False
        self.manual_base = False

        self.phase = "ready"
        self.phase_started_ms = time.ticks_ms()
        self.outputs_stopped_at_ms = self.phase_started_ms
        self.pulse_duration_ms = 1000
        self.dosing_cycles_count = 0
        self.pump_error_locked = False
        self.current_ph = 7.0
        self.last_pump_action = "STANDBY"
        self._stop_all()

    def _set_pwm_duty(self, name, percent):
        percent = max(0, min(100, percent))
        duty_u16 = int((percent / 100.0) * 65535)
        self.pwm[name].duty_u16(duty_u16)

    def _stop_all(self):
        self._set_pwm_duty("acid", 0)
        self._set_pwm_duty("base", 0)
        self._set_pwm_duty("motor", 0)

    def _read_ph(self):
        voltage_mv_sum = 0
        for _ in range(ADC_SAMPLE_COUNT):
            voltage_mv_sum += self.adc.read_uv() // 1000
            time.sleep_ms(1)

        voltage_mv = voltage_mv_sum // ADC_SAMPLE_COUNT
        ph = ph_from_adc_millivolts(voltage_mv)
        print(f"ADC: {voltage_mv} mV -> pH: {ph:.2f} | Phase: {self.phase.upper()}")
        return ph

    def update(self):
        now = time.ticks_ms()

        if not self.system_enabled or self.pump_error_locked:
            self._stop_all()
            self.last_pump_action = "OFF" if not self.system_enabled else "LOCKED"
            if time.ticks_diff(now, self.outputs_stopped_at_ms) > UPDATE_INTERVAL_MS:
                self.current_ph = self._read_ph()
                self.outputs_stopped_at_ms = now
            return False

        if self.mode == "MANUAL":
            if self.manual_acid:
                self._set_pwm_duty("acid", 70)
                self._set_pwm_duty("base", 0)
                self._set_pwm_duty("motor", 50)
                self.last_pump_action = "MANUAL ACID"
            elif self.manual_base:
                self._set_pwm_duty("acid", 0)
                self._set_pwm_duty("base", 70)
                self._set_pwm_duty("motor", 50)
                self.last_pump_action = "MANUAL BASE"
            else:
                self._stop_all()
                self.last_pump_action = "MANUAL IDLE"

            if time.ticks_diff(now, self.outputs_stopped_at_ms) > UPDATE_INTERVAL_MS:
                self.current_ph = self._read_ph()
                self.outputs_stopped_at_ms = now
            return True

        if self.phase == "dosing":
            if time.ticks_diff(now, self.phase_started_ms) < self.pulse_duration_ms:
                return True

            self._set_pwm_duty("acid", 0)
            self._set_pwm_duty("base", 0)
            self._set_pwm_duty("motor", 50)
            self.phase = "mixing"
            self.phase_started_ms = now
            return True

        if self.phase == "mixing":
            if time.ticks_diff(now, self.phase_started_ms) < MIXING_DURATION_MS:
                return True

            self._stop_all()
            self.outputs_stopped_at_ms = now
            self.phase = "ready"
            return True

        if time.ticks_diff(now, self.outputs_stopped_at_ms) < ADC_SETTLING_DELAY_MS:
            return True

        self.current_ph = self._read_ph()

        # Trong khoảng pH 6.0–8.0, hệ thống nghỉ bơm.
        if 6.0 <= self.current_ph <= 8.0:
            self.dosing_cycles_count = 0
            self.integral = 0.0
            self.last_pump_action = "STANDBY"
            self.logger.log(
                now, self.current_ph, self.target_ph, "none", 0,
                self.ai.kp, self.ai.ki, self.ai.kd
            )
            return True

        if self.dosing_cycles_count >= MAX_DOSING_CYCLES:
            self._stop_all()
            self.pump_error_locked = True
            self.last_pump_action = "LOCKED"
            return False

        error = self.target_ph - self.current_ph
        dt = (
            MIXING_DURATION_MS + ADC_SETTLING_DELAY_MS + self.pulse_duration_ms
        ) / 1000.0

        if math.isfinite(self.last_ph):
            self.ai.adapt(error, self.prev_error, self.current_ph - self.last_ph)

        self.integral = max(-50.0, min(50.0, self.integral + error * dt))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = (
            self.ai.kp * error
            + self.ai.ki * self.integral
            + self.ai.kd * derivative
        )
        self.prev_error = error
        self.last_ph = self.current_ph

        pump_name = "base" if output > 0 else "acid"
        self.pulse_duration_ms = int(min(3500, max(400, abs(output))))
        self.dosing_cycles_count += 1
        self.last_pump_action = "AUTO " + pump_name.upper()

        self.logger.log(
            now, self.current_ph, self.target_ph, pump_name,
            self.pulse_duration_ms, self.ai.kp, self.ai.ki, self.ai.kd
        )
        self._set_pwm_duty(pump_name, 70)
        self.phase = "dosing"
        self.phase_started_ms = now
        return True


def setup_wifi():
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid="ESP32_pH_Controller", authmode=network.AUTH_OPEN)
    time.sleep_ms(500)
    print("--> [WIFI OK] IP: http://" + str(ap.ifconfig()[0]))
    return ap.ifconfig()[0]


def create_web_server():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 80))
    s.listen(3)
    s.settimeout(0.02)
    return s


def send_csv_download(conn, filename):
    try:
        file_size = os.stat(filename)[6]
        csv_file = open(filename, "rb")
    except OSError as e:
        print("Lỗi mở file CSV:", repr(e))
        conn.sendall(
            b"HTTP/1.1 500 Internal Server Error\r\n"
            b"Content-Type: text/plain\r\n"
            b"Cache-Control: no-store\r\n"
            b"Connection: close\r\n\r\n"
            b"Cannot open data_log.csv"
        )
        return

    with csv_file:
        conn.settimeout(1.0)
        header = (
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/csv; charset=utf-8\r\n"
            "Content-Disposition: attachment; filename=data_log.csv\r\n"
            f"Content-Length: {file_size}\r\n"
            "Cache-Control: no-store\r\n"
            "Connection: close\r\n\r\n"
        )
        conn.sendall(header.encode("utf-8"))

        while True:
            chunk = csv_file.read(1024)
            if not chunk:
                break
            conn.sendall(chunk)


WEB_HTML = """<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>pH System Dashboard</title>
    <style>
        :root { --bg: #0f172a; --card: #1e293b; --accent: #38bdf8; --text: #f8fafc; --muted: #94a3b8; }
        body { font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg); color: var(--text); margin: 0; padding: 20px; }
        .container { max-width: 1000px; margin: 0 auto; display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .full { grid-column: 1 / -1; }
        .card { background: var(--card); border-radius: 16px; padding: 20px; border: 1px solid #334155; box-shadow: 0 10px 25px rgba(0,0,0,0.3); }
        h1, h2 { margin: 0 0 15px 0; font-weight: 400; letter-spacing: -0.5px; }
        .ph-value { font-size: 64px; font-weight: 700; color: var(--accent); line-height: 1; margin: 10px 0; }
        .badge { display: inline-block; padding: 6px 14px; border-radius: 20px; font-weight: 600; background: #059669; font-size: 13px; }
        .btn { padding: 12px 20px; border: none; border-radius: 8px; font-weight: 600; cursor: pointer; transition: 0.2s; color: white; background: #3b82f6; }
        .btn:active { transform: scale(0.96); }
        .btn-danger { background: #ef4444; }
        .btn-success { background: #10b981; }
        .btn-hold { background: #f59e0b; width: 100%; height: 60px; font-size: 18px; touch-action: manipulation; user-select: none; }
        .btn-hold:active { background: #d97706; }
        .flex { display: flex; gap: 10px; align-items: center; }
        pre { background: #090d16; padding: 12px; border-radius: 8px; font-size: 11px; overflow-x: auto; max-height: 180px; color: #a7f3d0; }
    </style>
</head>
<body>
    <div class="container">
        <div class="card full flex" style="justify-content: space-between;">
            <div>
                <h1>HỆ THỐNG CÂN BẰNG pH TỰ ĐỘNG</h1>
                <div style="color: var(--muted);">ESP32-S3 Edge AI &amp; FSM Controller</div>
            </div>
            <button id="pwrBtn" class="btn btn-danger" onclick="togglePower()">TẮT HỆ THỐNG</button>
        </div>

        <div class="card" style="text-align: center;">
            <div style="color: var(--muted); font-size: 14px;">GIÁ TRỊ pH REALTIME</div>
            <div id="phVal" class="ph-value">--</div>
            <div id="statusBadge" class="badge">INIT</div>
            <div style="margin-top: 15px; color: var(--muted); font-size: 13px;">
                Kp: <span id="kp">--</span> | Ki: <span id="ki">--</span> | Pha FSM: <span id="fsm">--</span>
            </div>
        </div>

        <div class="card">
            <h2>CHẾ ĐỘ VẬN HÀNH</h2>
            <div class="flex" style="margin-bottom: 15px;">
                <button id="btnAuto" class="btn btn-success" onclick="setMode('AUTO')">TỰ ĐỘNG (AUTO)</button>
                <button id="btnManual" class="btn" onclick="setMode('MANUAL')">BẰNG TAY (MANUAL)</button>
            </div>
            <div id="manualControls" style="display:none;">
                <div style="color: var(--muted); font-size: 12px; margin-bottom: 8px;">
                    Nhấn giữ để kích hoạt bơm, thả ra để dừng
                </div>
                <div class="flex">
                    <button class="btn btn-hold" onmousedown="pumpHold('acid', true)" onmouseup="pumpHold('acid', false)" onmouseleave="pumpHold('acid', false)" ontouchstart="pumpHold('acid', true)" ontouchend="pumpHold('acid', false)">BƠM AXÍT</button>
                    <button class="btn btn-hold" onmousedown="pumpHold('base', true)" onmouseup="pumpHold('base', false)" onmouseleave="pumpHold('base', false)" ontouchstart="pumpHold('base', true)" ontouchend="pumpHold('base', false)">BƠM BAZƠ</button>
                </div>
            </div>
        </div>

        <div class="card full">
            <h2>ĐỒ THỊ BIẾN THIÊN pH</h2>
            <canvas id="chart" height="90"></canvas>
        </div>

        <div class="card full">
            <div class="flex" style="justify-content: space-between;">
                <h2>NHẬT KÝ DỮ LIỆU (CSV LOG)</h2>
                <a href="/download_csv" class="btn" style="text-decoration:none;">TẢI FILE CSV</a>
            </div>
            <pre id="csvLog">Loading log...</pre>
        </div>
    </div>

    <script>
        let isPowerOn = true;
        let currentMode = 'AUTO';

        const byId = id => document.getElementById(id);
        const canvas = byId('chart');
        const ctx = canvas.getContext('2d');
        const samples = [];

        canvas.style.display = 'block';
        canvas.style.width = '100%';
        canvas.style.height = '210px';

        function drawChart() {
            const width = canvas.clientWidth;
            const height = canvas.clientHeight;
            if (!ctx || width < 80 || height < 60) return;

            const scale = window.devicePixelRatio || 1;
            canvas.width = Math.round(width * scale);
            canvas.height = Math.round(height * scale);
            ctx.setTransform(scale, 0, 0, scale, 0, 0);

            const left = 43, right = width - 12;
            const top = 12, bottom = height - 25;
            ctx.font = '12px sans-serif';
            ctx.fillStyle = '#94a3b8';

            if (!samples.length) {
                ctx.fillText('Đang chờ dữ liệu pH…', left, top + 16);
                return;
            }

            const values = samples.map(sample => sample.ph);
            const min = Math.floor((Math.min(...values) - 0.25) * 2) / 2;
            const max = Math.ceil((Math.max(...values) + 0.25) * 2) / 2;
            const firstTime = samples[0].time;
            const lastTime = samples[samples.length - 1].time;
            const duration = Math.max(1, lastTime - firstTime);

            const x = time => left + (time - firstTime) / duration * (right - left);
            const y = ph => bottom - (ph - min) / (max - min) * (bottom - top);

            ctx.strokeStyle = '#334155';
            ctx.textAlign = 'right';
            for (let i = 0; i <= 4; i++) {
                const ph = min + (max - min) * i / 4;
                const row = y(ph);
                ctx.beginPath();
                ctx.moveTo(left, row);
                ctx.lineTo(right, row);
                ctx.stroke();
                ctx.fillText(ph.toFixed(2), left - 6, row + 4);
            }

            ctx.textAlign = 'left';
            ctx.fillText(new Date(firstTime).toLocaleTimeString(), left, height - 5);
            if (samples.length > 1 && width > 230) {
                ctx.textAlign = 'right';
                ctx.fillText(new Date(lastTime).toLocaleTimeString(), right, height - 5);
            }

            ctx.strokeStyle = '#38bdf8';
            ctx.fillStyle = '#38bdf8';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(x(firstTime), y(samples[0].ph));
            for (const sample of samples.slice(1)) {
                ctx.lineTo(x(sample.time), y(sample.ph));
            }
            ctx.stroke();

            ctx.beginPath();
            ctx.arc(x(lastTime), y(samples[samples.length - 1].ph), 3, 0, 2 * Math.PI);
            ctx.fill();
        }

        function updateState() {
            fetch('/api/data').then(r => r.json()).then(d => {
                const validPH = Number.isFinite(d.ph);
                byId('phVal').innerText = validPH ? d.ph.toFixed(2) : '--';
                byId('statusBadge').innerText = d.status;
                byId('kp').innerText = d.kp.toFixed(1);
                byId('ki').innerText = d.ki.toFixed(2);
                byId('fsm').innerText = d.fsm.toUpperCase();

                isPowerOn = d.pwr;
                byId('pwrBtn').innerText = isPowerOn ? 'TẮT HỆ THỐNG' : 'BẬT HỆ THỐNG (TIẾP TỤC)';
                byId('pwrBtn').className = isPowerOn ? 'btn btn-danger' : 'btn btn-success';

                currentMode = d.mode;
                byId('btnAuto').className = currentMode === 'AUTO' ? 'btn btn-success' : 'btn';
                byId('btnManual').className = currentMode === 'MANUAL' ? 'btn btn-success' : 'btn';
                byId('manualControls').style.display = currentMode === 'MANUAL' ? 'block' : 'none';

                if (validPH) {
                    samples.push({ time: Date.now(), ph: d.ph });
                    if (samples.length > 30) samples.shift();
                    drawChart();
                }
            }).catch(console.error);

            fetch('/api/log').then(r => r.text()).then(text => {
                byId('csvLog').innerText = text;
            }).catch(console.error);
        }

        function togglePower() {
            fetch('/api/control?pwr=' + (isPowerOn ? '0' : '1')).then(updateState);
        }

        function setMode(mode) {
            fetch('/api/control?mode=' + mode).then(updateState);
        }

        function pumpHold(type, state) {
            if (currentMode !== 'MANUAL') return;
            fetch('/api/control?pump=' + type + '&state=' + (state ? '1' : '0'));
        }

        window.addEventListener('resize', drawChart);
        drawChart();
        updateState();
        setInterval(updateState, 1500);
    </script>
</body>
</html>"""


def main():
    print("Khởi chạy ESP32-S3 Advanced Web Dashboard")
    setup_wifi()
    server_socket = create_web_server()

    try:
        controller = PHController(target_ph=7.0)
    except Exception as e:
        controller = None
        print("Lỗi Controller:", e)

    while True:
        cycle_start = time.ticks_ms()

        if controller:
            controller.update()

        conn = None
        try:
            conn, _ = server_socket.accept()
            conn.settimeout(0.2)
            request = conn.recv(1024).decode("utf-8")

            if "GET /api/data" in request:
                data = {
                    "ph": controller.current_ph,
                    "status": controller.last_pump_action,
                    "kp": controller.ai.kp,
                    "ki": controller.ai.ki,
                    "fsm": controller.phase,
                    "pwr": controller.system_enabled,
                    "mode": controller.mode,
                }
                response = (
                    "HTTP/1.1 200 OK\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Content-Type: application/json\r\n\r\n"
                    + json.dumps(data)
                )
                conn.sendall(response.encode("utf-8"))

            elif "GET /api/log" in request:
                log_txt = controller.logger.read_last_lines(12)
                response = (
                    "HTTP/1.1 200 OK\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Content-Type: text/plain\r\n\r\n"
                    + log_txt
                )
                conn.sendall(response.encode("utf-8"))

            elif request.startswith("GET /download_csv "):
                try:
                    send_csv_download(conn, CSV_FILENAME)
                except OSError as e:
                    print("Lỗi gửi file CSV:", repr(e))

            elif "GET /api/control" in request:
                if "pwr=1" in request:
                    controller.system_enabled = True
                if "pwr=0" in request:
                    controller.system_enabled = False
                if "mode=AUTO" in request:
                    controller.mode = "AUTO"
                if "mode=MANUAL" in request:
                    controller.mode = "MANUAL"
                if "pump=acid&state=1" in request:
                    controller.manual_acid = True
                if "pump=acid&state=0" in request:
                    controller.manual_acid = False
                if "pump=base&state=1" in request:
                    controller.manual_base = True
                if "pump=base&state=0" in request:
                    controller.manual_base = False

                conn.sendall(
                    b"HTTP/1.1 200 OK\r\n"
                    b"Access-Control-Allow-Origin: *\r\n"
                    b"Content-Type: text/plain\r\n\r\n"
                    b"OK"
                )

            else:
                response = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: text/html; charset=utf-8\r\n\r\n"
                    + WEB_HTML
                )
                conn.sendall(response.encode("utf-8"))

        except OSError:
            pass
        finally:
            if conn is not None:
                conn.close()

        elapsed = time.ticks_diff(time.ticks_ms(), cycle_start)
        sleep_time = UPDATE_INTERVAL_MS - elapsed
        if sleep_time > 0:
            time.sleep_ms(sleep_time)


if __name__ == "__main__":
    main()