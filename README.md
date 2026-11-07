/nmdktm_project/
# 🧪 Automated pH Control System (ESP32-S3)

Dự án thiết kế và triển khai **Hệ thống Cân bằng pH Tự động** sử dụng vi điều khiển **ESP32-S3**, kết hợp thuật toán điều khiển lai **Hybrid (FSM + Adaptive PID + Edge AI Tuning)** và giao diện giám sát **Web Dashboard Real-time**.

---

## 📌 1. Tính năng nổi bật

- **Thuật toán điều khiển Hybrid:** 
  - **FSM (Finite State Machine):** Quản lý chu kỳ 3 pha (*DOSING* -> *MIXING* -> *READY*) chống nhiễu ADC.
  - **Adaptive PID + Edge AI:** Tự động điều chỉnh hệ số $K_p, K_i$ thích nghi theo độ biến thiên pH, triệt tiêu hiện tượng vọt ngưỡng (Overshoot).
- **Chế độ Vận hành Kép:**
  - **AUTO Mode:** Tự động tính xung châm PWM cho 2 bơm Axít/Bazơ và động cơ khuấy.
  - **MANUAL Mode:** Điều khiển châm thủ công bằng cơ chế **Hold-to-Run** (Nhấn giữ để bơm, thả tay dừng ngay).
- **Web Dashboard Real-time (Non-blocking Socket):**
  - Chạy trực tiếp từ Web Server nội bộ của ESP32-S3 (IP: `192.168.4.1`).
  - Đồ thị biến thiên pH Real-time tích hợp thư viện Chart.js.
  - Nút **Tắt/Bật Hệ Thống khẩn cấp (Master Power Switch)**.
  - Nút **Tải trực tiếp file dữ liệu nhật ký `data_log.csv`** về máy tính.

---

## 🛠️ 2. Sơ đồ sơ đồ kết nối phần cứng (Pinout)

| Linh kiện | Chân ESP32-S3 | Ghi chú |
| :--- | :--- | :--- |
| **Cảm biến pH (E201-C)** | `GPIO 2` (ADC1_CH1) | Đọc điện áp Analog (Mạch chia áp R_div = 2.0) |
| **Bơm Axít (PWM)** | `GPIO 6` | Tần số PWM = 1000 Hz, Duty 70% |
| **Bơm Bazơ (PWM)** | `GPIO 7` | Tần số PWM = 1000 Hz, Duty 70% |
| **Động cơ khuấy JGB37-520** | `GPIO 5` | Tần số PWM = 1000 Hz, Duty 50% |

---

## 🚀 3. Hướng dẫn chạy chương trình

1. Nạp firmware **MicroPython** cho board ESP32-S3.
2. Sao chép toàn bộ mã nguồn vào thư mục `src/`:
   - `src/main.py`
   - `src/lcd1602.py`
3. Chạy `main.py` thông qua Thonny IDE hoặc VS Code.
4. Kết nối Wi-Fi máy tính/điện thoại vào Access Point:
   - **SSID:** `ESP32_pH_Controller`
   - **Password:** *Không có*
5. Truy cập Web Dashboard trên trình duyệt (Chrome/Cốc Cốc):
   - **URL:** `http://192.168.4.1`

---

## 📊 4. Cấu trúc Lưu trữ Log (CSV Format)

Dữ liệu vận hành được tự động lưu vào bộ nhớ flash `data_log.csv`:
```csv
timestamp_ms,ph,target_ph,pump_action,pulse_ms,kp,ki,kd
10240,4.25,7.00,base,2150,1200.00,15.00,80.00
18240,5.40,7.00,base,1420,1200.00,15.00,80.00