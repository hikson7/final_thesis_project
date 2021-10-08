/* Brief: Test code for drive servo
 * Modified by: Hikari Hashida
 * Written: 30-Sept-2021
 * Last Visited: 30-Sept-2021
*/

#include "BluetoothSerial.h"

// https://github.com/madhephaestus/ESP32Servo
#include "ESP32Servo.h"

// Bluetooth initialisation error
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Pins
static const int led_pin = 16;
static const int servo_steer_pin = 18;
static const int servo_drive_pin = 19;

BluetoothSerial SerialBT;
Servo servo_driver, servo_steer;

// Task for driving servo

static void setupGPIO(void) {
    // put your setup code here, to run once:
    pinMode(led_pin, OUTPUT);
    // servo init
    for (int timer=0; timer < 4; timer++) {
        ESP32PWM::allocateTimer(timer);
    }
    servo_steer.setPeriodHertz(50);
    servo_steer.attach(servo_steer_pin, 500, 2400);
    servo_driver.setPeriodHertz(50);
    servo_driver.attach(servo_drive_pin, 800, 2200);
}

static void setupBlueTooth(void) {
  Serial.begin(115200);
  SerialBT.begin("Lunar Sphere"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void setup() {
    setupGPIO();
    setupBlueTooth();
}

void loop() {
    // int drive_time = 1500;
    // int DRIVE_ORIGIN = 1500; 
    // servo_driver.write(90);
    // // servo_steer.write(90);
    // delay(5000);
    // servo_driver.write(45);
    // delay(5000);

    int pos = 90;
    servo_steer.write(pos);

    int single_step = 30/2;
    int drive_time = 1500;
    int DRIVE_ORIGIN = 1500;
    int STEER_ORIGIN = 90;
    if (SerialBT.available() >= 2) {
        char command = SerialBT.read();
        uint8_t val = SerialBT.read();
        digitalWrite(led_pin, HIGH);
        switch (command) {
            case 'F':
                servo_driver.writeMicroseconds(DRIVE_ORIGIN-val);
                delay(drive_time);
                servo_driver.writeMicroseconds(DRIVE_ORIGIN);
                delay(20);
                break;
            case 'B':
                servo_driver.writeMicroseconds(DRIVE_ORIGIN+val);
                delay(drive_time);
                servo_driver.writeMicroseconds(DRIVE_ORIGIN); 
                delay(20);
                break;

            case 'R':
                pos += val;
                if (pos >= STEER_ORIGIN+45) {
                    pos = STEER_ORIGIN+45;
                }
                servo_steer.write(pos);
                delay(75);
                break;
            case 'L':
                pos -= val;
                if (pos <= 45) {
                    pos = 45;
                }
                servo_steer.write(pos);
                delay(75);
                break;
        }
        digitalWrite(led_pin, LOW);
        SerialBT.write(command);
        SerialBT.write('\n');
    }
}
