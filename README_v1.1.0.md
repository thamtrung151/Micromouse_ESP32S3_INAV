# micromouse-tiny-v1.1.0

## Thay đổi chính

AUTO mode đã được đổi sang kiến trúc **continuous straight-run**:

- Không còn tự build script `turn + MoveCells(1)` cho từng ô.
- Chạy thẳng liên tục bằng **PWM cố định** (`AUTO_STRAIGHT_PWM`).
- **Encoder chỉ dùng cho quãng đường / mốc cell** trong AUTO mode.
- **IMU tiếp tục là nguồn yaw chính** để giữ heading và xoay.
- Mỗi khi odometry đi đủ `AUTO_CELL_MM = 180mm`, AutoRunner cập nhật `(x, y)` sang ô kế tiếp.
- Nếu cặp IR trước bắt được trigger đáng tin cậy ở vùng đã định, hệ thống sẽ dùng
  **`AUTO_FRONT_OVERRIDE_RUN_MM = 140mm`** để snap tâm ô kế tiếp và giảm lỗi encoder bị quá tâm.
- Khi tới goal, driver dùng **brake high-high** (`HwDrive::brake()`).

## Những tham số AUTO mode nên tune trước tiên

Trong `src/config.h`:

- `AUTO_STRAIGHT_PWM`
- `AUTO_CELL_MM`
- `AUTO_IR_PRE_CENTER_MM`
- `AUTO_IR_POST_CENTER_MM`
- `AUTO_FRONT_TRIGGER_FROM_CENTER_MM`
- `AUTO_FRONT_TRIGGER_BAND_MM`
- `AUTO_FRONT_PREVIEW_WALL_TH_MM`
- `AUTO_FRONT_OVERRIDE_RUN_MM`
- `AUTO_TURN_BRAKE_MS`

## Ghi chú kiến trúc

- Script mode cũ vẫn còn để test primitive nếu cần.
- AUTO mode mới dùng `AutoRunner` + direct control API trong `Motion`:
  - `autoStartStraight(...)`
  - `autoStartTurn(...)`
  - `autoBrake(...)`
- Wall correction bên hông chỉ active trong nửa sau của cell (theo cửa sổ AUTO).
- Bản đồ tường chỉ **latch tường dương tính**, không xoá tường đã xác nhận vì một mẫu IR nhiễu.
