#include <config.h>

void RGB_init();
void RGB_loop();
void RGB_setFCReady(bool ready);   // gọi true khi flight controller đã khởi tạo xong
void RGB_setStarted(bool started); // gọi true khi ấn Start
void RGB_setArrived(bool arrived); // gọi true khi đến đích