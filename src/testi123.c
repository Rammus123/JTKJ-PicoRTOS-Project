/*      Rasmus Kurkelan ja Tuomas Kyöstin JTKJ-lopputyö.

    Kuvaus: Tämä ohjelma lukee ICM-42670P IMU-sensorin kiihtyvyysdataa ja
    muuntaa sen sarjaporttiin lähetettäväksi pisteeksi (.) tai viivaksi (-)
    perustuen sensorin asentoon. Käyttäjä voi painaa BUTTON1 nappia
    lähettääkseen datan, ja BUTTON2 nappia yhdessä BUTTON1 kanssa lähettääkseen
    välilyönnin. Ohjelma käyttää FreeRTOS-käyttöjärjestelmää tehtävien hallintaan.
    
    Ohjelmassa ei ole tilakonetta, vaan se toimii jatkuvassa silmukassa,
    jossa se lukee sensoridataa ja päättää, mitä lähettää sarjaporttiin
    käyttäjän napinpainallusten perusteella.

    Aloitettu 22.10.2025

    Viimeisin muokkaus: 12.11.2025
    
*/


// Käytettävät kirjastot:

#include <stdio.h>
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <tkjhat/sdk.h>



// Keskeytyskäsittelijä,Tämä ohjaa tulostaako dataa vai ei.
bool lahettaa = false;

static void btn_fxn(uint gpio, uint32_t eventMask) {
    if (lahettaa == false) {
        lahettaa = true;
    } else {
        lahettaa = false;
    }

}

void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz, t;
    // Sensoreiden setuppaus. Nämä tulostuu kun aletaan ajamaan ohjelmaa. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        
        int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        printf ("Enable gyro: %d\n",_enablegyro);

        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        printf ("Gyro return:  %d\n", _gyro);

        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        printf ("Accel return:  %d\n", _accel);

    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }



    // Loputon loop. Täällä loopissa me keräillääm IMU:n kiihdytysdataa.
    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            
            // printf("Accel: X=%f, Y=%f, Z=%f | Gyro: X=%f, Y=%f, Z=%f\n", ax, ay, az, gx, gy, gz);

            // Tuomas kirjoitti if-lauseet, Rasmus debuggasi.
        if (lahettaa){
            if (gpio_get(BUTTON2)==1){
                printf(" \n");  //Tulostaa välilyönnin jos painamme BUTTON2 pohjaan ja BUTTON1.
            }
            else if (ax > -1 && ax < 1 && ay > -1 && ay < 0.6 && az > 0.5 && az < 1.5) {   //Tässä määritelty arvot, joiden perusteella tulee piste tai viiva.
                    // Sensori vaakatasossa
                    printf("-\n");
                } else if (ax > -0.5 && ax < 0.5 && ay > 0.8 && ay < 1.5 && az > -0.6 && az < 0.6) {
                    // Sensori pystytasossa
                    printf(".\n");
                } else {
                    printf("\n");
                }
                lahettaa = false;
            }

        } else {
            printf("Failed to read imu data\n");  // Jos dataa ei saada luettua, tulostaa tämän.
        }
        vTaskDelay(pdMS_TO_TICKS(500)); // Keskeytetään nykyinen tehtävä puoleksi sekunniksi FreeRTOS-järjestelmässä. Muuntaa ajanjakson millisekunneista järjestelmän tikkeihin.
    
    }

}

int main() {
    stdio_init_all();
 // Alustetaan napit käyttöön. (BUTTON1 lähettää - ja . serial clienttiin, BUTTON2 + BUTTON1 luo välilyönnin.)
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);

    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);
 
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);

    
    while (!stdio_usb_connected()){
        sleep_ms(10);
    }
    init_hat_sdk();
    sleep_ms(300); // Odottaa hetken, jotta USB ja hatin initialisointi ehtii tapahtua.
    printf("Start acceleration test\n");

    TaskHandle_t hIMUTask = NULL;

    xTaskCreate(imu_task, "IMUTask", 1024, NULL, 2, &hIMUTask);

    //  Aloittaa FreeRTOS ajon.
    vTaskStartScheduler();

    return 0;
}
