
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"
#include <tusb.h>
#include "usbSerialDebug/helper.h"





#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1
#define MAX_BUFFER_SIZE 32
/* Global variables */
uint8_t rx_buffer[MAX_BUFFER_SIZE];
uint8_t tx_buffer[MAX_BUFFER_SIZE];



enum state {WAITING=1,
            READ_SENSOR=2,
            PRINT_DATA=3,
            READ_UART=4
           };
enum state programState = WAITING;

// ---- Task running USB stack ----
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}
static void btn_fxn(uint gpio, uint32_t eventMask) {
    // Tehtävä 1: Vaihda LEDin tila.
    //            Tarkista SDK, ja jos et löydä vastaavaa funktiota, sinun täytyy toteuttaa se itse.
    // Exercise 1: Toggle the LED. 
    //             Check the SDK and if you do not find a function you would need to implement it yourself. 
    // toggle_red_led();
    toggle_led();
}

static void sensor_task(void *arg){
    (void)arg;
    for(;;){
        // if (programState == READ_SENSOR){

        // }


        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void print_task(void *arg){
    (void)arg;

    while(1){
        
       

        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
// if data is available read it and echo it back.
static void uart_receive_task(void *arg){
    (void)arg;
    for(;;){
        if (programState == READ_UART){
            
            if(tud_cdc_n_available(0) > 0) {

                uint32_t count = tud_cdc_n_read(0, rx_buffer, sizeof(rx_buffer));
                tud_cdc_n_write(0, rx_buffer, count);
                tud_cdc_n_write_flush(0);
                toggle_led();
            }
            else{
                if (usb_serial_connected()) {
                    
                    usb_serial_print("buffer empty");
                    usb_serial_flush();
                    
                }
            }
            
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


int main() {
    
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.
    init_led();
    init_button1();

    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, &btn_fxn);

    TaskHandle_t hSensorTask, hPrintTask, hUartTask, hUSB = NULL;

    
    xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif
    


    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                &hSensorTask);                   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }
    result = xTaskCreate(print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hPrintTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }
    result = xTaskCreate(uart_receive_task,  // (en) Task function
                "uart_receive_task",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hUartTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }



    programState = READ_UART;
    // Initialize TinyUSB 
    tusb_init();
    //Initialize helper library to write in CDC0)
    usb_serial_init();
    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}

