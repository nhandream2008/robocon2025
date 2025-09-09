#include <Arduino.h>
#include <Bluepad32.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>

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

// Các biến toàn cục
int motor_speed = 190; // Tốc độ motor
int mode = 0;
#define spd 160
int pos1 = 90;
int pos2 = 90;
int pos3 = 90;
int pos4 = 90;
int servo_speed = 2;

// Servo speed control
int servo_speed_delay = 20; // Delay in milliseconds between position updates

///* Khai báo cảm biến dò line *///
#define DEBUG 0
#define On_calb_led_pin 19
#define Line_ss_enable_pin 21
#define SensorCount 5
#define SensorInputPin {32, 33, 25, 26, 27}
uint16_t sensorValues[SensorCount];
QTRSensors qtr;



// Callback khi controller kết nối
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
}
// Các hàm điều khiển động cơ cơ bản
void tien() {
    digitalWrite(Drv_In1_pin, HIGH);
    digitalWrite(Drv_In3_pin, HIGH);
    ledcWrite(Drv_In2_chn, motor_speed);
    ledcWrite(Drv_In4_chn, motor_speed);
}

void lui() {
    digitalWrite(Drv_In1_pin, LOW);
    digitalWrite(Drv_In3_pin, LOW);
    ledcWrite(Drv_In2_chn, motor_speed);
    ledcWrite(Drv_In4_chn, motor_speed);
}

void retrai() {
    digitalWrite(Drv_In1_pin, HIGH);
    digitalWrite(Drv_In3_pin, LOW);
    ledcWrite(Drv_In2_chn, motor_speed);
    ledcWrite(Drv_In4_chn, motor_speed);
}

void rephai() {
    digitalWrite(Drv_In1_pin, LOW);
    digitalWrite(Drv_In3_pin, HIGH);
    ledcWrite(Drv_In2_chn, motor_speed);
    ledcWrite(Drv_In4_chn, motor_speed);
}

void dung() {
    ledcWrite(Drv_In2_chn, 0);
    ledcWrite(Drv_In4_chn, 0);
}

// Các hàm điều khiển line following
void tien_1() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd - 55);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd - 55);
}

void rephai_1() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd - 20);
}

void retrai_1() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd - 20);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void rephai1() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd - 30);
}

void retrai1() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd - 30);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void rephai2() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd - 40);
}

void retrai2() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd - 40);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void rephai3() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 1);
    ledcWrite(Drv_In4_chn, spd);
}

void retrai3() {
    digitalWrite(Drv_In1_pin, 1);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void ban() {
    digitalWrite(Drv_In5_pin, LOW);
    ledcWrite(Drv_In6_chn, 0);

   // ledcWrite(DIR_In6_pin, motor_speed);
}

void ban1() {
    digitalWrite(Drv_In5_pin, 0);
    ledcWrite(Drv_In6_chn, 150);
    //ledcWrite(DIR_In6_pin, 0)
}

void tien0() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, 100);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, 100);
}

void tienthangnhanh() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd - 40);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd - 40);
}

void rephai_4a() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd-10);
}

void rephai_4b() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd-15);
}

void rephai_5a() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd-20);
}

void rephai_5b() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd-25);
}

void retrai_1a() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd-10);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void retrai_1b() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd-15);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void retrai_2a() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd-20);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void retrai_2b() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd-25);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

void rephai6() {
    digitalWrite(Drv_In1_pin, 0);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 1);
    ledcWrite(Drv_In4_chn, spd);
}

void retrai6() {
    digitalWrite(Drv_In1_pin, 1);
    ledcWrite(Drv_In2_chn, spd);
    digitalWrite(Drv_In3_pin, 0);
    ledcWrite(Drv_In4_chn, spd);
}

// Hàm khởi tạo cảm biến line
void LineSensorInit() {
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[])SensorInputPin, SensorCount);
    qtr.setEmitterPin(Line_ss_enable_pin);
    delay(500);
    pinMode(On_calb_led_pin, OUTPUT);
    digitalWrite(On_calb_led_pin, HIGH);
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
    }
    digitalWrite(On_calb_led_pin, LOW);
#if DEBUG
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
#endif
    delay(1000);
}

// Hàm dò line
void doline() {
    unsigned int sensors[5];
    int32_t LinePosition;
    LinePosition = qtr.readLineBlack(sensorValues);
    int error = LinePosition - 2000;
    if ((error < 800) && (error > -800)) {
        tien_1();
    } else if ((error > 800) && (error < 900)) {
        rephai_1();
    } else if ((error < -800) && (error > -900)) {
        retrai_1();
    } else if ((error > 900) && (error < 1600)) {
        rephai1();
    } else if ((error < -900) && (error > -1600)) {
        retrai1();
    } else if ((error > 1600) && (error < 1800)) {
        rephai2();
    } else if ((error < -1600) && (error > -1800)){
        retrai2();
    } else if (error > 1800) {
        rephai3();
    } else if (error < -1800) {
        retrai3();
    }

    if ((error < 200) && (error > 400)) {
        tien0();
    } 
    if ((error < 400) && (error > 600)) {
        tienthangnhanh();
    }
    if ((error < 600) && (error > 800)) {
        tien0();
    }
    else if ((error > 100) && (error < -100)) {
        retrai_1a();
    } else if ((error > 300) && (error < -300)) {
        retrai_1b();
    }
    else if ((error > 400) && (error < -400)) {
        retrai_2a();
    } 
    else if ((error > 600) && (error < -600)) {
        retrai_2b();
    } 
    else if ((error > 1000) && (error < -1000)) {
        rephai_4a();
    } 
    else if ((error > 1200) && (error < -1200)) {
        rephai_4b();
    }
    else if ((error > 1300) && (error < -1300)) {
        rephai_5a();
    } else if ((error > 1500) && (error < -1500)) {
        rephai_5b();
    }
    else if (error > 2000) {
        rephai6();
    } else if (error < -2000) {
        retrai6();
    }
}

// Hàm xử lý gamepad
void processGamepad(ControllerPtr ctl) {
    // Điều khiển chuyển động bằng analog trái
    int joyY = ctl->axisY();
    int joyX = ctl->axisX();
    int joyRY = ctl->axisRY();
    int joyRX = ctl->axisRX();

    // Trạng thái nút L2 và R2
    bool isL2Pressed = ctl->l2(); // Nhấn L2 -> tiến
    bool isR2Pressed = ctl->r2(); // Nhấn R2 -> lùi

    // Xử lý di chuyển với L2 và R2 + joystick (SOCD)
    if (isL2Pressed) { // Khi nhấn L2
        if (joyX < -100) { // Giữ L2 và kéo joystick trái
            retrai();
        } else if (joyX > 100) { // Giữ L2 và kéo joystick phải
            rephai();
        } else { // Giữ L2 và joystick ở trung tâm
            tien();
        }
    } else if (isR2Pressed) { // Khi nhấn R2
        if (joyX < -100) { // Giữ R2 và kéo joystick trái
            retrai();
        } else if (joyX > 100) { // Giữ R2 và kéo joystick phải
            rephai();
        } else { // Giữ R2 và joystick ở trung tâm
            lui();
        }
    } else { // Khi không nhấn L2 hay R2, dùng logic gốc với SOCD
        bool movingLeft = joyX < -100;
        bool movingRight = joyX > 100;
        bool movingUp = joyY < -100;   // Gạt lên
        bool movingDown = joyY > 100;  // Gạt xuống

        // SOCD cho trục X (trái/phải)
        if (movingLeft && movingRight) {
            dung(); // Nhấn đồng thời trái và phải -> dừng
        } else if (movingLeft) {
            retrai();
        } else if (movingRight) {
            rephai();
        } 
        // SOCD cho trục Y (giữ logic gốc)
        else if (movingUp && movingDown) {
            dung(); // Nhấn đồng thời lên và xuống -> dừng
        } else if (movingUp) {
            rephai(); // Gạt lên -> rẽ phải (như code gốc)
        } else if (movingDown) {
            retrai(); // Gạt xuống -> rẽ trái (như code gốc)
        } else {
            dung(); // Không nhấn gì -> dừng
        }
    }

    // Các nút chức năng khác từ code gốc
    if (ctl->miscButtons() & 0x02) mode = 0;  // Share
    if (ctl->miscButtons() & 0x04) mode = 1;  // Options
    
    // Tăng tốc độ khi đẩy joystick xuống (giá trị lớn hơn 300)
    if (joyRY > 300 && motor_speed < 255) {
        motor_speed -= 5;
        Serial.print("Tăng tốc độ: "); Serial.println(motor_speed);
    }

    // Giảm tốc độ khi đẩy joystick lên (giá trị nhỏ hơn -300)
    if (joyRY < -300 && motor_speed > 0) {
        motor_speed += 5;
        Serial.print("Giảm tốc độ: "); Serial.println(motor_speed);
    }
        // Điều khiển động cơ bắn
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long currentTime = millis();
bool currentR1State = ctl->r1();

if (currentR1State && !lastR1State && (currentTime - lastR1PressTime > DEBOUNCE_DELAY)) {
    lastR1PressTime = currentTime; // Cập nhật thời điểm nhấn R1

    if (!isShooting) {
        ban1(); // Bật động cơ bắn
        isShooting = true;
        Serial.println("BẮT ĐẦU BẮN");
    } else {
        ban(); // Tắt động cơ bắn
        isShooting = false;
        Serial.println("DỪNG BẮN");
    }
}

lastR1State = currentR1State;
    lastR1State = currentR1State; // Cập nhật trạng thái trước đó của R1
    // Điều khiển servo
    if (ctl->x()) pos3 = 0;    // Nhấn x, đặt servo về 0 độ
    if (ctl->b()) pos3 = 90;   // Nhấn b, đặt servo về 90 độ

    if (ctl->a() && pos2 > 0) pos2 -= servo_speed;
    if (ctl->y() && pos2 < 180) pos2 += servo_speed;

    if (ctl->dpad() & DPAD_UP && pos3 < 180) pos3 += servo_speed;
    if (ctl->dpad() & DPAD_DOWN && pos3 > 0) pos3 -= servo_speed;

    if (ctl->dpad() & DPAD_RIGHT && pos4 < 180) pos4 += servo_speed;
    if (ctl->dpad() & DPAD_LEFT && pos4 > 0) pos4 -= servo_speed;

    servo1.write(pos1);
    servo2.write(pos2);
    servo3.write(pos3);
    servo4.write(pos4);
}

void setup() {
    Serial.begin(115200);
    
    // Khởi tạo Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    Serial.println("Waiting for controller...");
    
    // Khởi tạo các thành phần
    LineSensorInit();
    
    // Khởi tạo động cơ
    pinMode(Drv_In1_pin, OUTPUT);
    ledcSetup(Drv_In2_chn, PWM_FREQ, PWM_RES);
    ledcAttachPin(Drv_In2_pin, Drv_In2_chn);
    
    pinMode(Drv_In3_pin, OUTPUT);
    ledcSetup(Drv_In4_chn, PWM_FREQ, PWM_RES);
    ledcAttachPin(Drv_In4_pin, Drv_In4_chn);
    
    pinMode(Drv_In5_pin, OUTPUT);
    ledcSetup(Drv_In6_chn, PWM_FREQ, PWM_RES);
    ledcAttachPin(Drv_In6_pin, Drv_In6_chn);

    // Đảm bảo động cơ bắn tắt khi khởi động
    digitalWrite(Drv_In5_pin, LOW);
    //ledcWrite(PWM_In6_pin, 0);
   // ledcWrite(DIR_In6_pin, 0);
    Serial.println("Động cơ bắn: TẮT (khởi động)");

    // Khởi tạo servo
    servo1.attach(SERVO_1_PIN, 500, 2400);
    servo2.attach(SERVO_2_PIN, 500, 2400);
    servo3.attach(SERVO_3_PIN, 500, 2400);
    servo4.attach(SERVO_4_PIN, 500, 2400);
    
    // Góc ban đầu
    servo1.write(pos1);
    servo2.write(pos2);
    servo3.write(pos3);
    servo4.write(pos4);
}

void loop() {
    // Cập nhật trạng thái controller
    BP32.update();
    Serial.println(isShooting);
    
    // Xử lý input từ controller
    for (auto controller : myControllers) {
        if (controller && controller->isConnected()) {
            processGamepad(controller);
        }
    }
    
     // Chế độ line following
     if (mode == 1) {
         doline();
 }
    
     delay(20);
}
