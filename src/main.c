/*
Notes:
COM6 is used for usb_serial debug prints.
usb_serial_print can be used anywhere to debug things.
usb_serial_print("ICM-42670P initialized successfully!\n"); 

!! COM7 communicates with the host computer.



1. 
Buzzer works but needs to figure out how to play messages.
buzzer_play_tone(440, 500);
2. display task works but gyro does not work simultaneously.



****************************
TODO:



****************************
*/
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
#define MAX_BUFFER_SIZE 100
#define COM_PORT_7  1
#define BUFFER_SIZE 100

// if we want to use debug prints
#define DEBUG 1

/* Global variables */
uint8_t rx_buffer[MAX_BUFFER_SIZE];
uint8_t tx_buffer[MAX_BUFFER_SIZE];
uint8_t last_program_state = 0;
uint8_t switch_to_gyro_task = 0;
uint8_t switch_to_button2_task = 0;
/**
 * @struct morse_code
 * @brief Represents a mapping between a letter and its Morse code.
 * This struct is used in the global morse_table array. {'A', ".-"}, {'B', "-..."},....
 */
struct morse_code {
    char letter;
    const char *code;
};


struct morse_code morse_table[] = {
    {'A', ".-"}, {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},
    {'E', "."}, {'F', "..-."}, {'G', "--."}, {'H', "...."},
    {'I', ".."}, {'J', ".---"}, {'K', "-.-"}, {'L', ".-.."},
    {'M', "--"}, {'N', "-."}, {'O', "---"}, {'P', ".--."},
    {'Q', "--.-"}, {'R', ".-."}, {'S', "..."}, {'T', "-"},
    {'U', "..-"}, {'V', "...-"}, {'W', ".--"}, {'X', "-..-"},
    {'Y', "-.--"}, {'Z', "--.."}
};



enum state {WAITING=1,
            READ_SENSOR=2,
            PRINT_DATA=3,
            READ_UART=4,
            READ_GYRO=5,
            WRITE_UART=6,
            MENU_TASK=7
           };
enum state program_state = WAITING;

void morse_to_text(const char *message, char *output){
    int table_size = sizeof(morse_table) / sizeof(morse_table[0]);
    char temp[BUFFER_SIZE];
    int temp_index = 0;
    int output_index = 0;
    output[0] = '\0'; // make sure output is empty
    int space_count = 0; // to store spaces from message and determinen where words end or start

    for (int i = 0; message[i] != '\0'; i++) {
        //char charFromMessage = message[i];
        if (message[i] == '-' || message[i] == '.'){
            if (temp_index < sizeof(temp) - 1) {
                temp[temp_index] = message[i];
                temp_index++;
                temp[temp_index] = '\0'; //move end of line
            }
            space_count = 0; // reset space count after a dot/dash
        }
        else if (message[i] == ' '){
            space_count++;
            printf("space_count: %d\n", space_count);
        }
        if (space_count == 1 && temp_index > 0){
            // end of letter
            for (int j = 0; j < table_size; j++) {
                if (strcmp(temp, morse_table[j].code) == 0) {
                    output[output_index] = morse_table[j].letter;
                    output_index++;
                }
            }
            temp_index = 0;
            temp[0] = '\0';
           
        }
        else if (space_count == 2 && message[i+1] != ' '){
            output[output_index] = ' ';
            output_index++;
            space_count = 0; // reset space count after processing space
        }
        else{
            // end of word
            output[output_index] = '\0'; // null-terminate the output string
            space_count = 0; // reset space count after processing word
        }
        
    }
    /* TESTING */
    /*
    printf("TESTING message: %s\n", message);
    printf("TESTING temp: <%s>\n", &temp);
    printf("output:<%s>\n", output);
    */
}



/**
 * @brief function to write morse code message to display in clear text
 * 
 * @param msg 
 */

void write_to_display(const char* msg){
    /*
    char morseInput[200] = {".- .- ... ..  --- -.   \n\0"};
    char decoded[100];
     morse_to_text(morseInput, decoded);
    morse_to_text(morseInput, decoded);
    printf("Decoded text: %s\n", decoded);
    */
    
    
    clear_display();
    //char decoded[100];
    //morse_to_text(msg, decoded);
    write_text_modified(msg);
    //write_text_modified(msg);
}

void debug_print(const char* msg){
    #ifdef DEBUG
    usb_serial_print("DEBUG: ");
    usb_serial_print(msg);
    #endif
}
static void right_button_gyrotask_fxn_switch_interrupt(uint gpio, uint32_t eventMask) {
    // rgb_led_write(255,0,0); // red
    static TickType_t last_press = 0;
    TickType_t cur_time = xTaskGetTickCountFromISR();

    // 100ms debounce
    if (cur_time - last_press < pdMS_TO_TICKS(200)) {
        return; // ignore bounce
    }
    last_press = cur_time;
    
    
    
    // write_to_display("gyro task switched");
    //write_to_display(gpio == SW1_PIN ? "SW1 pressed" : "SW2 pressed");
    debug_print(gpio == SW1_PIN ? "SW1 pressed\n" : "SW2 pressed\n");
    if (gpio == SW1_PIN){
        switch_to_gyro_task = !switch_to_gyro_task;
        if (switch_to_gyro_task == 1){
            program_state = READ_GYRO;
        }
        switch_to_button2_task = 0;
        rgb_led_write(120,0,0); // red
    }
    else if (gpio == SW2_PIN){
        switch_to_button2_task = !switch_to_button2_task;
        switch_to_gyro_task = 0;
        program_state = WAITING;
        rgb_led_write(0,0,120); // blue
    }
    if (switch_to_gyro_task == 0 && switch_to_button2_task == 0){
        // waiting for testing purposes 
        program_state = READ_UART;
        //program_state = READ_UART;
        rgb_led_write(0,120,0); // green
    }
  


}

// static void left_button_interrupt(uint gpio, uint32_t eventMask) {
    
//     static TickType_t last_press = 0;
//     TickType_t cur_time = xTaskGetTickCountFromISR();

//     // 100ms debounce
//     if (cur_time - last_press < pdMS_TO_TICKS(200)) {
//         return; // ignore bounce
//     }
//     rgb_led_write(0,0,255); // blue
// }


// ---- Task running USB stack ----
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // With FreeRTOS wait for events
                                 // Do not add vTaskDelay. 
    }
}

static void sensor_task(void *arg){
    (void)arg;
    for(;;){
        // if (program_state == READ_SENSOR){

        // }


        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void uart_write_task(void *arg){
    (void)arg;

    while(1){
        if (program_state == WRITE_UART){
            tud_cdc_n_write(COM_PORT_7, tx_buffer, strlen(tx_buffer));
            usb_serial_print(tx_buffer);
            tud_cdc_n_write_flush(COM_PORT_7);
            usb_serial_print("testing write uart task\n");
            program_state = last_program_state;   
        }
       

        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/*
UI task?
*/
static void display_task(void *arg){
    (void)arg;

    while(1){
        
        while (1) {

                clear_display();
                // write_text("Hello");
                // draw_circle(1, 1, 20);
                // draw_line(0, 0, 127, 0);
                draw_square(30,10,20,20, true);
                draw_square(70,10,20,20, false);
                vTaskDelay(1000);
                clear_display();
                draw_line(30, 20, 50, 20);
                draw_line(70, 20, 90, 20);
                vTaskDelay(300);

        }

        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



// if data is available read it and echo it back.
/*
* UART receive task
* 1. Echo back received data
* 2. Toggle LED when data is received
* 3. Print received data to debug console (COM6) using usb_serial_print
* 4. Change state to indicate data has been read
* 5. Show message on display (optional) and play buzzer tone (optional)
*/
static void uart_receive_task(void *arg){
    uint8_t send_data_back = 0;
    (void)arg;
    while (!tud_mounted() || !tud_cdc_n_connected(1)){
            vTaskDelay(pdMS_TO_TICKS(50));
    }
    for(;;){
        if (program_state == READ_UART){
            // Get the number of bytes available for reading
            if(tud_cdc_n_available(COM_PORT_7) > 0) {
                uint32_t count = tud_cdc_n_read(COM_PORT_7, rx_buffer, sizeof(rx_buffer));
                // Echo back the received data
                //tud_cdc_n_write(COM_PORT_7, rx_buffer, count);
                // tud_cdc_n_write(COM_PORT_7, "Received morsecode", 19);


                char decoded[100];                
                morse_to_text((const char*)rx_buffer, decoded);

                // Write to LCD display in clear text
                
                // write_to_display(decoded);
                
                
                // ?? force write?
                //tud_cdc_n_write_flush(COM_PORT_7);
                /************** TESTING USB SERIAL ***************/

                
                

                // just test that usb_serial is working
                // this can be used for testing if the usb is connected
                if (usb_serial_connected()) {
                    //prints in com6
                    debug_print(decoded);
                    debug_print(rx_buffer);
                }
                /************** TESTING USB SERIAL ***************/
                // change state that uart data has been read
                // send message back to sender
                sprintf(tx_buffer, "Decoded message: %s\n", decoded);
                program_state = WRITE_UART;
                last_program_state = READ_UART;
                // clear rx buffer
                memset(rx_buffer, 0, count);
            }

              
           vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/****************************** */

#define MAX_SAMPLES 3
// --- buffer for last 3 accel samples ---
float ax_buffer[MAX_SAMPLES] = {0};
float ay_buffer[MAX_SAMPLES] = {0};
float az_buffer[MAX_SAMPLES] = {0};
int buffer_index = 0;
int buf_filled = 0;


/**
 * 
 * @brief Add new accel sample to buffer and loop around if buffer is full
 * 
 */

void add_accel_value(float ax, float ay, float az) {
    ax_buffer[buffer_index] = ax;
    ay_buffer[buffer_index] = ay;
    az_buffer[buffer_index] = az;

    buffer_index = (buffer_index + 1) % MAX_SAMPLES;
    if (buf_filled < MAX_SAMPLES) {
        buf_filled++;
    }
}

/**
 * 
 * @brief Get average of the accel samples in the buffer
 * 
 */
void get_avg(float *ax, float *ay, float *az) {
    float sx = 0, sy = 0, sz = 0;

    for (int i = 0; i < buf_filled; i++) {
        sx += ax_buffer[i];
        sy += ay_buffer[i];
        sz += az_buffer[i];
    }

    *ax = sx / buf_filled;
    *ay = sy / buf_filled;
    *az = sz / buf_filled;
}

// simple orientation detection
int orientation_from_accel(float ax, float ay, float az) {
    float TH = 0.5; // 50% of gravity

    if (az > TH) return 1;   // face up
    if (az < -TH) return 2;  // face down

    if (ax > TH) return 3;   // on left side
    if (ax < -TH) return 4;  // on right side
    return 0; // unknown
}


/**************************** */
// Tasks prints Accel: X=-0.57, Y=-0.01, Z=-0.83 | Gyro: X=-0.21, Y=0.63, Z=-0.17| Temp: 30.31°C
static void gyroscope_task(void *arg){
    (void)arg;
    float ax, ay, az, gx, gy, gz, t;
    if (init_ICM42670() == 0) {
        usb_serial_print("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            usb_serial_print("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        usb_serial_print("Failed to initialize ICM-42670P.\n");
    }

    // Start collection data here. Infinite loop. 
    uint8_t buf[BUFFER_SIZE];
    for(;;){
        if (program_state == READ_GYRO){
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                // sprintf(buf,"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f| Temp: %2.2f°C\n", ax, ay, az, gx, gy, gz, t);
                // sprintf(buf,"Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f\n", ax, ay, az, gx, gy, gz);
                // sprintf(buf,"aX:%.2f, aY:%.2f, aZ:%.2f\n", ax, ay, az);
                //sprintf(buf,"gX:%.2f, gY:%.2f, gZ:%.2f\n", gx, gy, gz);
                add_accel_value(ax, ay, az);
                 // get smoothed values
                float ax_s, ay_s, az_s;
                get_avg(&ax_s, &ay_s, &az_s);

                int o = orientation_from_accel(ax_s, ay_s, az_s);
                if (o == 1) {
                    sprintf(tx_buffer, "Orientation: Face Up\n");
                } else if (o == 2) {
                    sprintf(tx_buffer, "Orientation: Face Down\n");
                } else if (o == 3) {
                    sprintf(tx_buffer, "Orientation: Left Side\n");
                } else if (o == 4) {
                    sprintf(tx_buffer, "Orientation: Right Side\n");
                } 

                if (o != 0) {
                    program_state = WRITE_UART;
                    last_program_state = READ_GYRO;
                }
                
                // Direct orientation check
                // if (az > 7.0f) {
                //     usb_serial_print(".\n");   // dot
                // }
                // else if (ax > 7.0f) {
                //     usb_serial_print("-\n");   // dash
                // }

            } else {
                usb_serial_print("Failed to read imu data\n");
            }
            
        }
        
        // Do not remove this
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


int main() {
    
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.
    init_led();
    init_rgb_led();
    init_button1();
    init_button2();
    init_buzzer();
    init_display();
    rgb_led_write(0,0,0);
    clear_display();

    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_FALL, true, &right_button_gyrotask_fxn_switch_interrupt);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_FALL, true, &right_button_gyrotask_fxn_switch_interrupt);

    TaskHandle_t hSensorTask, hPrintTask, hUartTask, hGyroTask, hDisplayTask, hUSB = NULL;

    
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
    result = xTaskCreate(uart_write_task,  // (en) Task function
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

    result = xTaskCreate(gyroscope_task,  // (en) Task function
            "gyroscope_task",              // (en) Name of the task 
            DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
            NULL,                 // (en) Arguments of the task 
            2,                    // (en) Priority of this task
            &hGyroTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }

    // result = xTaskCreate(display_task,  // (en) Task function
    //         "display_task",              // (en) Name of the task 
    //         DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
    //         NULL,                 // (en) Arguments of the task 
    //         2,                    // (en) Priority of this task
    //         &hDisplayTask);         // (en) A handle to control the execution of this task

    // if(result != pdPASS) {
    //     printf("Print Task creation failed\n");
    //     return 0;
    // }


    //test program states
    // program_state = READ_GYRO;
    program_state = READ_UART;
    rgb_led_write(0,120,0); // green


    // Initialize TinyUSB 
    tusb_init();
    //Initialize helper library to write in CDC0)
    usb_serial_init();
    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}

