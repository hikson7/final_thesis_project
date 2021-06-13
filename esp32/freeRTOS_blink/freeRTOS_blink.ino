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

// Pins
static const int led_pin = 23;

// Task for led toggle
void taskToggleLED(void *parameter) {
  // vTaskDelay takes number of ticks, not milliseconds
  int ticks_500ms = 500 / portTICK_PERIOD_MS;
  while (1) {  
    digitalWrite(led_pin, HIGH);
    vTaskDelay(ticks_500ms);
    digitalWrite(led_pin, LOW);
    vTaskDelay(ticks_500ms);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(led_pin, OUTPUT);

  // for vanilla FreeRTOS, use xTaskCreate()
  xTaskCreatePinnedToCore(
                taskToggleLED,  // function to be called
                "Toggle LED",   // name of task
                1024,           // stack size (bytes in ESP32, words in FreeRTOS) min 768 bytes
                NULL,           // Parameter to pass to func
                1,              // Task priority (0 lowest -> configMAX_PRIORITIES-1 highest)
                NULL,           // Task handle
                app_cpu);       // CPU core
}

void loop() {
  // put your main code here, to run repeatedly:
  
}