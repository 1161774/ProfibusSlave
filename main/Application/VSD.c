#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "VSD.h"

#define VSDTag "VSD"

// Task to simulate VSD control
void run(VSDSimulator* vsd) {
    while (1) {
        if (xSemaphoreTake(vsd->vsdMutex, portMAX_DELAY) == pdTRUE) {
            if (vsd->currentSpeed < vsd->speedSetpoint) {
                vsd->state = RAMPING_UP;
                if (vsd->currentSpeed < vsd->speedSetpoint) {
                    vsd->currentSpeed += 1.0; // Simulated ramp-up speed (you can adjust this)
                    //printf("%s - Current Speed: %f\n", vsd->vsdName, vsd->currentSpeed);
                }
                vsd->state = AT_SPEED;
            } else if (vsd->currentSpeed > vsd->speedSetpoint) {
                vsd->state = RAMPING_DOWN;
                if (vsd->currentSpeed > vsd->speedSetpoint) {
                    vsd->currentSpeed -= 1.0; // Simulated ramp-down speed (you can adjust this)
                    //printf("%s - Current Speed: %f\n", vsd->vsdName, vsd->currentSpeed);
                }
                vsd->state = AT_SPEED;
            }
            xSemaphoreGive(vsd->vsdMutex);
        }
    vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Method to simulate processing a Profibus command
void processProfibusCommand(VSDSimulator* vsd, const char* command) {
    // Process the received command (replace with actual command processing)
    //printf("%s - Processing Profibus Command: %s\n", vsd->vsdName, command);

vsd->speedSetpoint += 20; 
/*    if (strncmp(command, "START", 5) == 0) {
        // Nothing special needed when START is received
    } else if (strncmp(command, "STOP", 4) == 0) {
        // Nothing special needed when STOP is received
    } else if (strncmp(command, "SET_SPEED", 8) == 0) {
        float newSpeed;
        if (sscanf(command, "SET_SPEED %f", &newSpeed) == 1) {
            vsd->speedSetpoint = newSpeed;
        } else {
            printf("Invalid SET_SPEED command format.\n");
        }
    } else {
        printf("%s - Unknown command.\n", vsd->vsdName);
    }*/
}

// Task entry function
void taskEntry(void* pvParameters) {
    VSDSimulator* vsd = (VSDSimulator*)pvParameters;

    vsd->profibusSlave->State.ReadyState = SS_POWERON;
    vsd->profibusSlave->State.Frozen = 0;
    vsd->profibusSlave->State.Sync = 0;
    

    vsd->profibusSlave->Config.Address = vsd->profibusAddress;
    vsd->profibusSlave->Config.ID_HIGH = 0x70;
    vsd->profibusSlave->Config.ID_LOW  = 0x50;
    

    // Register profibus
    AddSlave(vsd->profibusAddress, vsd->profibusSlave);

    ESP_LOGI(VSDTag, "Starting VSD %s", vsd->vsdName);
    run(vsd);
}

