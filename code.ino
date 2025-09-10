// Khai báo controller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

///* Khai báo động cơ bánh xe *///
#define PWM_FREQ 1000
#define PWM_RES 8
// Động cơ bánh chạy
#define Drv_In1_pin 23
#define Drv_In2_pin 16 // PWM
#define Drv_In2_chn 2
#define Drv_In3_pin 22
#define Drv_In4_pin 4 // PWM
#define Drv_In4_chn 3
// Động cơ bắn
#define Drv_In5_pin 18
#define Drv_In6_pin 0 // PWM
#define Drv_In6_chn 6


// Khai báo servo 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
#define SERVO_1_PIN 5
#define SERVO_2_PIN 13
#define SERVO_3_PIN 12
#define SERVO_4_PIN 14
int shoot_speed = 255; // Tốc độ động cơ bắn
bool isShooting = false; // Trạng thái động cơ bắn
bool lastR1State = false; // Trạng thái trước đó của nút R1
unsigned long lastR1PressTime = 0; // Thời gian nhấn R1 cuối cùng (dùng để debounce)
#define DEBOUNCE_DELAY 200 // Độ trễ debounce (ms)
