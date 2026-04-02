#include <config.h>

void RGB_init();
void RGB_loop();
void RGB_setFCReady(bool ready);   // gọi true khi flight controller đã khởi tạo xong
void RGB_setStarted(bool started); // gọi true khi ấn Start
void RGB_setArrived(bool arrived); // gọi true khi đến đích
void RGB_setFingerArming(bool active); // blink nhanh 10Hz khi đang giữ tay
void RGB_setFingerReady(bool active);  // blink chậm như started, chờ nhấc tay
void RGB_setRainbow(bool active);
void RGB_setSavedMapReady(bool ready);
