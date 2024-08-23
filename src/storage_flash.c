#include "stdio.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "utils.h"

#include "storage_flash.h"

#define NAMESPACE1          "partition1"
#define KEY_ANG_KP          "ANG_KP"
#define KEY_ANG_KI          "ANG_KI"
#define KEY_ANG_KD          "ANG_KD"

#define KEY_POS_KP          "POS_KP"
#define KEY_POS_KI          "POS_KI"
#define KEY_POS_KD          "POS_KD"

#define KEY_SPD_KP          "SPD_KP"
#define KEY_SPD_KI          "SPD_KI"
#define KEY_SPD_KD          "SPD_KD"

#define KEY_CENTER          "CENTER"
#define KEY_SAFETY_LIM      "SAFETY_LIM"

static const char *TAG = "Storage_flash";

nvs_handle_t storageHandle;

void storageInit(){

    ESP_ERROR_CHECK(nvs_flash_init());
}

void eraseFlash(){
    nvs_flash_erase();
}

void storageLocalConfig(robot_local_configs_t localConfig){
     
    esp_err_t err = nvs_open(NAMESPACE1,NVS_READWRITE,&storageHandle);
    if( err != ESP_OK){
        ESP_LOGE(TAG,"Error open nvs");
        return;
    }

    nvs_set_u16(storageHandle,KEY_ANG_KP,(uint16_t)(localConfig.pids[PID_ANGLE].kp*100));
    nvs_set_u16(storageHandle,KEY_ANG_KI,(uint16_t)(localConfig.pids[PID_ANGLE].ki*100));
    nvs_set_u16(storageHandle,KEY_ANG_KD,(uint16_t)(localConfig.pids[PID_ANGLE].kd*100));

    nvs_set_u16(storageHandle,KEY_POS_KP,(uint16_t)(localConfig.pids[PID_POS].kp*100));
    nvs_set_u16(storageHandle,KEY_POS_KI,(uint16_t)(localConfig.pids[PID_POS].ki*100));
    nvs_set_u16(storageHandle,KEY_POS_KD,(uint16_t)(localConfig.pids[PID_POS].kd*100));

    nvs_set_u16(storageHandle,KEY_SPD_KP,(uint16_t)(localConfig.pids[PID_SPEED].kp*100));
    nvs_set_u16(storageHandle,KEY_SPD_KI,(uint16_t)(localConfig.pids[PID_SPEED].ki*100));
    nvs_set_u16(storageHandle,KEY_SPD_KD,(uint16_t)(localConfig.pids[PID_SPEED].kd*100));

    nvs_set_i16(storageHandle,KEY_CENTER,(int16_t)(localConfig.centerAngle*100));
    nvs_set_u16(storageHandle,KEY_SAFETY_LIM,(uint16_t)(localConfig.safetyLimits*100));

    if( nvs_commit(storageHandle) != ESP_OK ){
        ESP_LOGE(TAG,"Error commit nvs");
    }
    else{
        nvs_close(storageHandle);
    }
}


robot_local_configs_t getFromStorageLocalConfig(void){
    uint16_t safetyLimits;
    int16_t centerAngle;

    pid_params_raw_t pids[CANT_PIDS];

    robot_local_configs_t localConfig;

    esp_err_t err = nvs_open(NAMESPACE1,NVS_READWRITE,&storageHandle);
    if( err == ESP_OK){
        nvs_get_u16(storageHandle,KEY_ANG_KP,&pids[PID_ANGLE].kp);
        nvs_get_u16(storageHandle,KEY_ANG_KI,&pids[PID_ANGLE].ki);
        nvs_get_u16(storageHandle,KEY_ANG_KD,&pids[PID_ANGLE].kd);

        nvs_get_u16(storageHandle,KEY_POS_KP,&pids[PID_POS].kp);
        nvs_get_u16(storageHandle,KEY_POS_KI,&pids[PID_POS].ki);
        nvs_get_u16(storageHandle,KEY_POS_KD,&pids[PID_POS].kd);

        nvs_get_u16(storageHandle,KEY_SPD_KP,&pids[PID_SPEED].kp);
        nvs_get_u16(storageHandle,KEY_SPD_KI,&pids[PID_SPEED].ki);
        nvs_get_u16(storageHandle,KEY_SPD_KD,&pids[PID_SPEED].kd);
        nvs_get_i16(storageHandle,KEY_CENTER,&centerAngle);
        nvs_get_u16(storageHandle,KEY_SAFETY_LIM,&safetyLimits);
    }

    for(uint8_t i=0;i<CANT_PIDS;i++) {   
        localConfig.pids[i] = convertPidRawToFloats(pids[i]);
    }
    localConfig.centerAngle = (float)centerAngle/100;
    localConfig.safetyLimits = (float)safetyLimits/100;

    nvs_close(storageHandle);
    return localConfig;
}

