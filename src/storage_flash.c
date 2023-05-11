
#include "stdio.h"
#include "nvs.h"
#include "nvs_flash.h"

#define NAMESPACE1      "partition1"
#define KEY_KP          "KP"
#define KEY_KI          "KI"
#define KEY_KD          "KD"
#define KEY_CENTER      "CENTER"

nvs_handle_t storageHandle;

void storageInit(){

    ESP_ERROR_CHECK(nvs_flash_init());
}

void eraseFlash(){
    nvs_flash_erase();
}

bool storageWritePidParams(uint16_t cont){

    esp_err_t err = nvs_open(NAMESPACE1,NVS_READWRITE,&storageHandle);
    if( err == ESP_OK){

        nvs_set_i16(storageHandle,KEY_CENTER,cont);
        if( nvs_commit(storageHandle) != ESP_OK ){
            return false;
        }
        else{
            nvs_close(storageHandle);
            return true;
        }
    }
    else{
        return false;
    }
}


int16_t storageReadPidParams(void){
    int16_t param =0;

    esp_err_t err = nvs_open(NAMESPACE1,NVS_READWRITE,&storageHandle);
    if( err == ESP_OK){

        esp_err_t err2 =nvs_get_i16(storageHandle,KEY_CENTER,&param);
        if( err2 != ESP_OK ){
            return false;
        }
        else if( err2 == ESP_ERR_NVS_NOT_FOUND){
            printf("Error el valor no existe");
            
        }
    }
    else{
        return false;
    }

    nvs_close(storageHandle);
    return param;
}

