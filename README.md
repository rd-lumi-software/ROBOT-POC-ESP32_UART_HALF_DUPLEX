# ROBOT-POC-ESP32_UART_HALF_DUPLEX
POC giao tiếp UART 1 dây trên ESP32: Chuyển tiếp TX, RX sang cổng 1 wire (Half-Duplex) kết nối với động cơ bánh xe EC4012 (chuẩn giao thức Universal Bus Motor Protocol V3 / Dynamixel v1.0).

---

## 📌 Tính năng chính

- **Giao tiếp Half-Duplex 1 dây (Single-wire Half-Duplex):**
  - Chân tín hiệu: **GPIO 10** (vừa truyền TX vừa nhận RX).
  - Tốc độ bus: **1,000,000 bps (1 Mbps)**, 8 data bits, no parity, 1 stop bit (8N1).
  - Tự động lọc Echo phần cứng khi truyền, tránh nhiễu và dội ngược gói tin.
- **USB CDC Bridge:**
  - Nhận byte từ cổng USB Native của ESP32-C6 (VID `0x303A`, PID `0x1001`).
  - Đẩy trực tiếp ra bus động cơ 1 Mbps và chuyển phản hồi status packet ngược về PC.
- **Công cụ điều khiển Web Serial đi kèm (`index.uart.html`):**
  - Kết nối trực tiếp qua trình duyệt Chrome / Edge (Web Serial API).
  - Quét ID động cơ trên bus (Bus Scan: 1~16, 1~32, 1~253 hoặc tùy chọn).
  - Điều khiển tốc độ Continuous (Tiến / Lùi / Dừng / Hold, tối đa 950 rpm).
  - Điều khiển góc đích tuyệt đối (Position mode, độ phân giải 1024 bước/vòng).
  - Đổi ID động cơ và lưu vĩnh viễn vào Flash (Register 0x03).

---

## 🔌 Sơ đồ nối dây (Wiring)

| Chân ESP32-C6 | Chân Động cơ EC4012 | Ghi chú |
| :--- | :--- | :--- |
| **GPIO 10** | **DATA (Signal)** | Cần trở kéo lên (Pull-up) 4.7kΩ ~ 10kΩ lên 3.3V |
| **GND** | **GND** | Nối chung mass (GND) giữa ESP32 và nguồn động cơ |
| — | **VCC (12V ~ 24V)** | Cấp nguồn động lực cho động cơ |

*(Cổng USB native của ESP32-C6 cắm trực tiếp vào máy tính để nạp code và điều khiển qua giao diện Web).*

---

## 🛠️ Hướng dẫn cài đặt & Biên dịch (PlatformIO)

Dự án phát triển trên nền **ESP-IDF** (thuần C) và đóng gói qua **PlatformIO**.

### 1. Yêu cầu môi trường
- [Visual Studio Code](https://code.visualstudio.com/) đã cài extension **PlatformIO IDE** (hoặc cài đặt PlatformIO Core CLI qua Python: `pip install platformio`).

### 2. Các lệnh thao tác chính (Terminal)

```powershell
# 1. Biên dịch firmware (Build)
pio run

# 2. Nạp firmware vào ESP32-C6 (Upload)
pio run -t upload

# 3. Mở Serial Monitor xem log
pio device monitor

# 4. Combo: Nạp và mở Monitor ngay
pio run -t upload -t monitor

# 5. Dọn dẹp bản build (Clean)
pio run -t clean
```

> **Lưu ý về cổng COM:** Trong file `platformio.ini` đang cấu hình mẫu:
> - Cổng nạp: `COM26`
> - Cổng monitor: `COM29` (Baud `115200`)
> 
> Nếu máy của bạn nhận cổng khác, bạn có thể chỉnh lại trong file `platformio.ini` hoặc truyền cờ:
> `pio run -t upload --upload-port COMxx`

---

## 🎮 Sử dụng Web Controller Tool (`index.uart.html`)

1. Mở file [index.uart.html](index.uart.html) trực tiếp bằng trình duyệt Google Chrome hoặc Microsoft Edge.
2. Bấm **🔌 Kết nối Serial Port** và chọn thiết bị ESP32-C6 USB.
3. Dùng khối **🔍 Quét ID Động Cơ** để dò tìm các động cơ đang nối trên bus.
4. Bấm **[Chọn]** trên động cơ tìm thấy để bắt đầu điều khiển tốc độ hoặc quay tới góc.

---

## 📁 Cấu trúc thư mục

```text
├── CMakeLists.txt              # Cấu hình ESP-IDF cấp dự án
├── platformio.ini              # Cấu hình board esp32-c6-devkitc-1
├── sdkconfig.defaults          # Cấu hình kích thước Flash 8MB
├── sdkconfig.esp32-c6-devkitc-1
├── index.uart.html             # Web Serial Controller & Bus Scanner
├── .gitignore                  # Bỏ qua thư mục build .pio
└── src/
    ├── CMakeLists.txt
    ├── main.c                  # Khởi tạo phần cứng và luồng điều khiển
    ├── uart_bridge.c           # Logic chuyển tiếp USB CDC <-> UART 1 Mbps
    └── uart_bridge.h
```
>>>>>>> 04152ca (feat: ESP32-C6 single-wire UART half-duplex bridge & Web Serial test tool)
