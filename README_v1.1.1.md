# micromouse v1.1.1

## Thay đổi chính
- Bỏ cơ chế front-preview override theo IR trước.
- IR trong AUTO được bật liên tục.
- Front IR chỉ còn dùng làm ngưỡng phanh đơn giản:
  - nếu `ir2_mm` hoặc `ir3_mm` <= `AUTO_FRONT_BRAKE_TH_MM`
  - sau khi robot đã đi qua mốc `AUTO_FRONT_BRAKE_ARM_FROM_CENTER_MM`
  - AUTO ra lệnh brake thật.
- Thêm cơ chế neo lại bằng đuôi **mới**, không dùng `BackAlign` cũ:
  - chạy lùi tối đa `AUTO_REAR_ALIGN_MAX_BACK_MM`
  - nếu phát hiện stall / chạm tường sau thật sự -> tiến `AUTO_REAR_ALIGN_TO_CENTER_MM`
  - nếu không chạm tường sau -> tiến lại đúng quãng vừa lùi để quay về vị trí cũ (fallback an toàn)
- Cơ chế neo lại này được gọi ở:
  - lúc bắt đầu AUTO
  - trước khi vào turn

## Tham số cần tune
- `AUTO_FRONT_BRAKE_ARM_FROM_CENTER_MM`
- `AUTO_FRONT_BRAKE_TH_MM`
- `AUTO_FRONT_BRAKE_MS`
- `AUTO_REAR_ALIGN_MAX_BACK_MM`
- `AUTO_REAR_ALIGN_TO_CENTER_MM`
- `AUTO_REAR_ALIGN_PWM`
- `AUTO_REAR_ALIGN_STALL_ARM_MM`
- `AUTO_REAR_ALIGN_STALL_V_MM_S`
- `AUTO_REAR_ALIGN_STALL_MS`

## Ghi chú
- Đây là bản đã bỏ hoàn toàn đường code front-override theo IR trước.
- `BackAlign` cũ không được dùng trong AUTO v1.1.1.
