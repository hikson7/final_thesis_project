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

#include "BluetoothSerial.h"

// Bluetooth initialisation error
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Pins
static const int led_pin = 23;
BluetoothSerial SerialBT;

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
  while (1) {
    if (Serial.available()) {
        SerialBT.write(Serial.read());
        SerialBT.write('\n');
    }
    if (SerialBT.available()) {
        Serial.write(SerialBT.read());
        Serial.write('\n');
    }
    vTaskDelay(ms2Tick(20));
  }
}

static void setupGPIO(void) {
    // put your setup code here, to run once:
    pinMode(led_pin, OUTPUT);
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
