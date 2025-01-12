#include <Arduino.h>
#include <Servo.h>
#include <ServoMotorHelper.h>

#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"
#include "BLEDevice.h"

#define PIN_L_SERVO 32
#define PIN_R_SERVO 33
#define PIN_ARM 25
#define PIN_WRIST 26
#define PIN_CLAW 27

#define ARM_MIN 0
#define ARM_MAX 120

#define WRIST_MIN 0
#define WRIST_MAX 120

#define CLAW_MIN 10
#define CLAW_MAX 50
#define CLAW_MIN_LIM 0
#define CLAW_MAX_LIM 55

#define ARM_SPEED 2

uint8_t new_mac[] = {0x7c, 0x9e, 0xbd, 0xfa, 0x0b, 0xac};

// если поворачивает в другую сторону
bool invert_x = false;

Servo servo_l, servo_r, servo_arm, servo_wrist, servo_claw;

int cur_drive_angle_l = 90;
int cur_drive_angle_r = 90;
int cur_arm_angle = 120;
int cur_wrist_angle = 40;
int cur_claw_angle = 50;

ServoMotorHelper motor_helper_l(72, 51, 111, 132);
ServoMotorHelper motor_helper_r(111, 132, 72, 51);

void change_mac()
{
  if (BLEDevice::getInitialized()) BLEDevice::deinit(true);
  esp_base_mac_addr_set(new_mac);
  BLEDevice::init("ESP32");
}

// original code https://github.com/un0038998/PS4Controller_ESP32/blob/main/Remove_Paired_Devices/Remove_Paired_Devices.ino
void removePairedDevices(){
  uint8_t pairedDeviceBtAddr[20][6];  
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for(int i = 0; i < count; i++) 
  {
    uint8_t* address = pairedDeviceBtAddr[i];
    Serial.printf("drop device - %02x:%02x:%02x:%02x:%02x:%02x\n", address[0],address[1],address[2],address[3],address[4],address[5]);
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void connect_ps4_controller()
{
    PS4.begin();

    removePairedDevices(); // to avoid re-pairing issues

    const uint8_t* address = esp_bt_dev_get_address();
    char str[100];
    sprintf(str, "ESP32's Bluetooth MAC address is - %02x:%02x:%02x:%02x:%02x:%02x\n", address[0],address[1],address[2],address[3],address[4],address[5]);
    Serial.println(str);
}

void static write_angle(Servo& servo, int angle, int& cur_angle)
{
    cur_angle = constrain(angle, 0, 180);
    servo.write(cur_angle);

    // Serial.printf("angle: %d cur_angle: %d\n", angle, cur_angle);
}

void do_drive()
{
    static byte inc_angle = 10;

    byte LY = PS4.LStickY();
    byte LX = PS4.LStickX();

    // Serial.printf("LY: %d, LX: %d\n", LY, LX);
    if (LY == 128 && LX == 128)
    {
        servo_l.write(90);
        servo_r.write(90);

        cur_drive_angle_l = 90;
        cur_drive_angle_r = 90;
        return;
    }

    int y = -map(LY, 0, 255, -100, 100);
    int x = map(LX, 0, 255, -100, 100);

    Serial.printf("y: %d, x: %d\n", y, x);

    int speed_l = map(invert_x ? (y - x) : (y + x), -100, 100, -100, 100);
    int speed_r = map(invert_x ? (y + x) : (y - x), -100, 100, -100, 100);

    byte angle_l = motor_helper_l.get_angle(speed_l);
    byte angle_r = motor_helper_r.get_angle(speed_r);

    byte _angle_l = (angle_l > cur_drive_angle_l) ? _min(cur_drive_angle_l + inc_angle, angle_l) : _max(cur_drive_angle_l - inc_angle, angle_l);
    byte _angle_r = (angle_r > cur_drive_angle_r) ? _min(cur_drive_angle_r + inc_angle, angle_r) : _max(cur_drive_angle_r - inc_angle, angle_r);

    write_angle(servo_l, _angle_l, cur_drive_angle_l);
    write_angle(servo_r, _angle_r, cur_drive_angle_r);

    // Serial.printf("_angle_l: %d, _angle_r: %d\n", _angle_l, _angle_r);
}

void do_arm()
{
    byte RX = PS4.RStickX();

    if (RX == 128)
        return;

    int limit_angle = (ARM_MAX - ARM_MIN) / 2 - 10;
    int angle;

    // взаимные ограничения
    int rel_wrist_angle = map(cur_wrist_angle, WRIST_MIN, WRIST_MAX, ARM_MAX, ARM_MIN);
    int min_angle = constrain(rel_wrist_angle - limit_angle, ARM_MIN, ARM_MAX);
    int max_angle = constrain(rel_wrist_angle + limit_angle, ARM_MIN, ARM_MAX);

    if (RX < 128) // backward
    {
        angle = constrain(cur_arm_angle + ARM_SPEED, min_angle, max_angle);
        write_angle(servo_arm, angle, cur_arm_angle);
    }
    else
    {
        angle = constrain(cur_arm_angle - ARM_SPEED, min_angle, max_angle);
        write_angle(servo_arm, angle, cur_arm_angle);
    }

    Serial.printf("arm angle: %d\n", angle);
}

void do_wrist()
{
    byte RY = PS4.RStickY();

    if (RY == 128)
        return;

    int limit_angle = (WRIST_MAX - WRIST_MIN) / 2 - 10;
    int angle;

    // взаимные ограничения
    int rel_arm_angle = map(cur_arm_angle, ARM_MIN, ARM_MAX, WRIST_MAX, WRIST_MIN);
    int min_angle = constrain(rel_arm_angle - limit_angle, WRIST_MIN, WRIST_MAX);
    int max_angle = constrain(rel_arm_angle + limit_angle, WRIST_MIN, WRIST_MAX);

    if (RY > 128) // down
    {       
        angle = constrain(cur_wrist_angle + ARM_SPEED, min_angle, max_angle);
        write_angle(servo_wrist, angle, cur_wrist_angle);
    }
    else
    {
        angle = constrain(cur_wrist_angle - ARM_SPEED, min_angle, max_angle);
        write_angle(servo_wrist, angle, cur_wrist_angle);
    }

    Serial.printf("wrist angle: %d\n", angle);
}

void do_claw()
{
    static uint32_t tmr;
    static bool claw, unclaw;

    if (PS4.Square()) // разомкнуть
    {
        write_angle(servo_claw, CLAW_MAX_LIM, cur_claw_angle);
        unclaw = true;
    }
    else if (PS4.Cross()) // замкнуть
    {
        write_angle(servo_claw, CLAW_MIN_LIM, cur_claw_angle);
        claw = true;
    }

    // плавное сведение
    int angle;
    if (PS4.R1()) // разомкнуть
    {
        angle = constrain(cur_claw_angle + ARM_SPEED, 0, 180);
        write_angle(servo_claw, angle, cur_claw_angle);
        Serial.printf("wrist angle: %d\n", angle);
    }
    else if (PS4.L1()) // замкнуть
    {
        angle = constrain(cur_claw_angle - ARM_SPEED, 0, 180);
        write_angle(servo_claw, angle, cur_claw_angle);
        Serial.printf("wrist angle: %d\n", angle);
    }
}

void stop_all_motors()
{
    servo_l.write(90);
    servo_r.write(90);

    cur_drive_angle_l = 90;
    cur_drive_angle_r = 90;
}

void setup(){
    Serial.begin(115200);

    change_mac();

    servo_l.attach(PIN_L_SERVO);
    servo_r.attach(PIN_R_SERVO);
    servo_arm.attach(PIN_ARM);
    servo_wrist.attach(PIN_WRIST);
    servo_claw.attach(PIN_CLAW);

    connect_ps4_controller();
}

void loop(){
    static uint32_t tmr;
    if (millis() - tmr >= 20)
    {
        tmr = millis();
        if (PS4.isConnected())
        {
            do_drive();
            do_arm();
            do_wrist();
            do_claw();
        }
        else
        {
            // Serial.println("false");
            stop_all_motors();
        }
    }
}