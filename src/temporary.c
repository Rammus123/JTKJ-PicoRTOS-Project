

/*

PLAN:

    1.    ensiksi yritetään printata imusta gyron ja akkeleron dataa.
                Käytetään siihen example_imu.c koodia. 

                kirjoitetaan if lause jolla tarkistetaan onko acceleraattorin data tietyllä välillä.
                Jos on, printataan piste (.) tai viiva (-).

                Myös jos painaa nappia 2, niin printtaa välilyönnin.


    2.    toinen task olisi että yritetään saada data muokattua joko pisteeksi, viivaksi tai välilyönniksi.































*/




// Creator: Rasmus and Tuomas both.


int ICM42670_startGyro(uint16_t odr_hz, uint16_t fsr_g);

int ICM42670_start_with_default_values(void);


enum state{WAITING=1, READ_DATA, DISPLAY_DATA}; 
enum state myState = WAITING;

// Anturin käsittely
void sensorTask(void * pvParameters) {

    while (1) {
    
        if (myState == READ_DATA) {
        
            // Tilan toiminnallisuus
            read_sensor();
            
            // Tilasiirtymä READ_SENSOR -> DISPLAY_DATA
            myState = DISPLAY_DATA;				        
        }
    
        vTaskDelay(..);
    }
}

// Datan päivittely terminaaliin
void displayTask(void * pvParameters) {

    while (1) {
    
        if (myState == DISPLAY_DATA) {
        
            // Tilan toiminnallisuus
            update_data();
            
            // Tilasiirtymä DISPLAY_DATA -> WAITING
            myState = WAITING;				        
        }
    
        vTaskDelay(..);
    }
}





// Creator: both.

read_sensor();

 int init_ICM42670(void);
    int start_sensor_with_default_values(void);
    int ICM42670_read_sensor_data(float *ax, float *ay, float *az, 
                                  float *gx, float *gy, float *gz,
                                  float *t);




// kaava gyrovaluen muuntamiseksi ihmisen luettaviksi.
gyro_value[0:15] / 131 






    //
    //
    //





// _____________________________________________________________________________ from course repo example.


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
            
            printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f\n", ax, ay, az, gx, gy, gz);

        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}

int main() {
    stdio_init_all();
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




























