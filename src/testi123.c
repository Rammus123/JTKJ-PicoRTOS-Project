
#include <stdio.h>
#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <tkjhat/sdk.h>



void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        printf ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        printf ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        printf ("Accel return:  %d\n", _accel);*/
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    // Start collection data here. Infinite loop. 
    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            
            // printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f\n", ax, ay, az, gx, gy, gz);

            // Tuomas wrote if-statemets, Rasmus debugged it.
            if (ax > -1 && ax < 1 && ay > -1 && ay < 0.6 && az > 0.5 && az < 1.5) {
                    // vaakataso
                    printf("-\n");
                } else if (ax > -0.5 && ax < 0.5 && ay > 0.8 && ay < 1.5 && az > -0.6 && az < 0.6) {
                    // pystytaso
                    printf(".\n");
                } else {
                    printf("\n");
                }

        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}

int main() {
    stdio_init_all();
 // alustetaan napit käyttöön (button1 = välilyönti, button2 = lähettää datan serial monitoriin)
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);

    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);

    // Uncomment this lines if you want to wait till the serial monitor is connected
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); //Wait some time so initialization of USB and hat is done.
    init_led();
    printf("Start acceleration test\n");

    TaskHandle_t hIMUTask = NULL;

    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 2, &hIMUTask);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}



/*
vaakataso = väliviiva: x:(-0.2 - 0.3)   , y:(0-0.07)  z:(0.95-1.05)
pystytaso = piste: x:(-0.1 - 0.1) , y:(0.95 - 1.05), z:(-0.07 - 0.07)

*/

