/* Brief: Example code of FreeRTOS ESP32 from Digi-Key video tutorial
 * Modified by: Hikari Hashida
 * Written: 08-Jun-2021
 * Last Visited: 08-Jun-2021
 * Resource: https://www.youtube.com/watch?v=JIr7Xm_riRs&t=1s 
*/

# if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// essp
#include "BluetoothSerial.h"

// https://github.com/madhephaestus/ESP32Servo
#include "ESP32Servo.h"

// Bluetooth initialisation error
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Pins
static const int led_pin = 23;
static const int servo_steer_pin = 18;
static const int servo_drive_pin = 19;

BluetoothSerial SerialBT;
Servo servo_driver, servo_steer;

static int ms2Tick(int ms) {
    return ms / portTICK_PERIOD_MS;
}

// Task for led toggle
void taskToggleLED(void *parameter) {
    // vTaskDelay takes number of ticks, not milliseconds
    while (1) {
        digitalWrite(led_pin, HIGH);
        vTaskDelay(ms2Tick(500));
        digitalWrite(led_pin, LOW);
        vTaskDelay(ms2Tick(500));
    }
}

// Task for Serial Input
void taskSerialComm(void *parameter) {
  int pos = 90;
  while (1) {
    // if (Serial.available()) {
    //     SerialBT.write(Serial.read());
    //     SerialBT.write('\n');
    // }
    if (SerialBT.available()) {
        char command = SerialBT.read();
        switch (command) {
            case 'F':
                servo_driver.write(90-45);
                vTaskDelay(ms2Tick(3000));
                servo_driver.write(90);
                break;
            case 'R':
                pos += 5;
                if (pos >= 90+45) {
                    pos = 90+45;
                }
                servo_steer.write(pos);
                vTaskDelay(ms2Tick(75));
                break;
            case 'B':
                servo_driver.write(90+45);
                vTaskDelay(ms2Tick(3000));
                servo_driver.write(90); 
            break;
            case 'L':
                pos -= 5;
                if (pos <= 45) {
                    pos = 45;
                }
                servo_steer.write(pos);
                vTaskDelay(ms2Tick(75));
                break;
        }
        Serial.write(pos);
        Serial.write('\n');
    }
    vTaskDelay(ms2Tick(20));
  }
}

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

static void setupTasks() {
    /* void xTaskCreatePinnedToCore(
        task_function,          - function to be called
        name,                   - name of task
        stack size,             - stack size (bytes in ESP32, words in FreeRTOS) min 768 bytes 
        function parameters,    - Parameter to pass to func
        priority                - Task priority (0 lowest -> configMAX_PRIORITIES-1 highest)
        task handle,            - Task handle              
        cpu core                - Which CPU core to run apps on
    );
    */
    xTaskCreatePinnedToCore(taskToggleLED, "Toggle LED", 1024, NULL, 1, NULL, app_cpu);
    xTaskCreatePinnedToCore(taskSerialComm, "serial com", 1024, NULL, 5, NULL, app_cpu);
}

void setup() {
    setupGPIO();
    setupBlueTooth();
    setupTasks();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
