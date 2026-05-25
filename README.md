# Tendon-Driven Continuum Robot Controller

Desktop GUI (Python + PySide6) điều khiển robot dùng firmware ODRI [`mw_dual_motor_vel_pos_ctrl`](https://github.com/phuongnvcdt/udriver_firmware) qua CANable v2.0 (slcan firmware).

## Tính năng

### Kết nối
- Kết nối qua **slcan** (CANable v2.0), bitrate mặc định 1 Mbps (ODRI)
- Auto-detect COM port USB Serial Device
- Phím tắt **Spacebar** = E-Stop phần mềm (zero IqRef + disable motors + disable system)

### 3 chế độ điều khiển
Chuyển mode luôn kèm cảnh báo và tự reset setpoint về 0 (tránh giật khi chuyển).

| Mode | Mô tả |
|---|---|
| **⚡ Current (Iq)** | Gửi Iq reference (A) trực tiếp cho 2 motor, có clamp limit |
| **💨 Velocity** | Điều khiển vận tốc (rpm hiển thị, krpm qua CAN), có PID gains + limit |
| **📍 Position** | Điều khiển vị trí tuyệt đối (rev), có PID gains + limit + Set Home / Go Home |

### Chế độ Position
- **Set Home**: lưu vị trí encoder hiện tại làm điểm home, range spinbox tự cập nhật thành `[home ± limit]`
- **▶ Play / ⏸ Pause**: chỉ khi nhấn Play thì lệnh vị trí mới được gửi xuống firmware (tránh motor chạy theo giá trị cũ khi mới vào mode)
- **Go Home**: set spinbox về vị trí home và gửi lệnh (nếu đang Play)
- **Link**: kéo slider/spinbox một motor thì motor kia tự đi theo theo tỉ lệ trong range tương ứng

### Telemetry & monitoring
- **Status realtime**: sys enabled, motor 1/2 enabled/ready/aligning, error code
- **3 plots realtime** (pyqtgraph): Iq, position, velocity với rolling window 10s, Motor 1 (xanh) / Motor 2 (đỏ)
- **Log panel**: tất cả command gửi + sự kiện firmware

### Hệ thống
- **Watchdog thread** tự resend setpoint ở 50Hz để firmware không trip `CAN_RECV_TIMEOUT`
- Sequence chuẩn ODRI: enable system → enable motors → đợi alignment → điều khiển
- Settings tự lưu (Iq limit, vel limit, pos limit, PID gains) qua `QSettings`

## Yêu cầu

- Python 3.11+
- Windows / Linux
- CANable v2.0 với firmware **slcan** (không phải candleLight) → cắm vào xuất hiện COM port trong Device Manager
- Firmware ODRI `mw_dual_motor_vel_pos_ctrl` đã flash vào LaunchPad F28069M (dip switch ON-ON-OFF, run from flash)
- CAN bus: CANable CAN_H ↔ board CAN_H, CAN_L ↔ board CAN_L, **terminator 120Ω ở 2 đầu**

## Cài đặt

```bash
python -m venv .venv
# Windows:
.venv\Scripts\activate

pip install -e .
# dev (tests):
pip install -e ".[dev]"
```

## Chạy app

```bash
odri-can
# hoặc
python -m odri_can.main
```

### Quy trình sử dụng

1. Chọn COM port của CANable, bitrate **1 Mbps (ODRI)**, bấm **Connect**.
2. Bấm **Enable System** → **Enable Motors**. Motor alignment ~1-2s, status đổi sang "Ready ✓".
3. Chọn chế độ điều khiển (**Current / Velocity / Position**).
4. Điều khiển và theo dõi telemetry.
5. Khi xong: **Disable Motors** → **Disable System** → **Disconnect**.

### Phím tắt

| Phím | Tác dụng |
|---|---|
| **Space** | E-Stop: zero IqRef + disable motors + disable system |

## Cấu trúc code

```
src/odri_can/
├── main.py
├── protocol/
│   ├── messages.py        # CAN ID, command codes, CtrlMode, error codes
│   ├── codec.py           # Q24 encode/decode, MDL/MDH packing
│   └── odri_driver.py     # High-level: enable_*, set_iq_ref, set_vel_ref, set_hw_pos_ref
├── can/
│   ├── interface.py       # Abstract CanBackend
│   ├── slcan_backend.py   # python-can slcan wrapper
│   └── mock_backend.py    # Firmware giả lập cho offline testing
├── ui/
│   ├── main_window.py     # Main window + signal/slot wiring
│   ├── connection_panel.py
│   ├── control_panel.py   # 3 chế độ điều khiển, status, PID, Set Home
│   ├── telemetry_plots.py # 3 realtime plots
│   └── log_panel.py
└── models/
    └── motor_state.py     # SystemState, MotorState
```

## CAN Protocol

- **Bitrate**: 1 Mbit/s
- **Format**: 11-bit standard ID, 8 byte data (MDL 4 byte thấp + MDH 4 byte cao), big-endian int32
- **Q24 fixed-point**: `int = round(float × 2²⁴)`

### PC → Board

| ID | Tên | Nội dung |
|---|---|---|
| `0x000` | COMMANDS | MDL = value, MDH = command code |
| `0x005` | IQ_REF | MDL = Iq M1, MDH = Iq M2 (Q24, A) |
| `0x006` | VEL_POS_REF | MDL = ref M1, MDH = ref M2 (Q24, krpm hoặc mrev tuỳ CtrlMode) |

### Board → PC

| ID | Tên | Nội dung |
|---|---|---|
| `0x010` | STATUS | 1B: bit0=sys_en, bit1=mtr1_en, bit2=mtr1_ready, bit3=mtr2_en, bit4=mtr2_ready, bit5-7=error_code |
| `0x020` | CURRENT_IQ | Q24 × 2 (A) |
| `0x030` | POSITION | Q24 × 2 (rev) |
| `0x040` | VELOCITY | Q24 × 2 (krpm) |

Tham khảo: https://open-dynamic-robot-initiative.github.io/udriver_firmware/can/can_interface.html

## Lưu ý an toàn

- **E-Stop Spacebar là phần mềm**, không thay thế E-Stop phần cứng. Nên có:
  - Aptomat DC trên đường nguồn 24V
  - Nút cắt nguồn vật lý
- Iq limit mặc định **2.0A** (board hỗ trợ tới 15A). Tăng dần khi đã tin tưởng setup.
- Trước khi chuyển sang mode Current hoặc Velocity, app sẽ cảnh báo và tự reset setpoint về 0.
- Chế độ Position chỉ gửi lệnh khi nhấn **▶ Play**, tránh motor chạy ngay khi vào mode.

## Troubleshooting

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| Connect fail "could not open port" | Sai COM port, hoặc port đang bị process khác giữ |
| Connect OK nhưng không nhận status | Bitrate sai (firmware = 1 Mbps), CAN_H/CAN_L đấu ngược, thiếu terminator 120Ω |
| Motor không bao giờ "Ready" | Alignment fail — kiểm tra encoder wiring (Hirose DF13 5-pin), motor phase |
| Error: CAN Receive Timeout | Watchdog mất kết nối. Disable + reconnect |
| Error: Encoder Error | Encoder không đọc được — kiểm tra wiring AEDM-5810-Z12 |
| Velocity thực tế ≠ setpoint | Kiểm tra PID gains trong Velocity Control group |

## License

BSD 3-Clause (cùng với firmware ODRI gốc)
