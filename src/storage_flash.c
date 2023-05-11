
#include "stdio.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "storage_flash.h"

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

void storageWritePidParams(pid_params_t params){

     printf("Guardando nuevos parametros : %f\n",params.kp);

    esp_err_t err = nvs_open(NAMESPACE1,NVS_READWRITE,&storageHandle);
    if( err != ESP_OK){
        printf("ERROR ESCRITURA STORAGEWRITEPIDPARAMS 1\n");
        return;
    }

    nvs_set_u16(storageHandle,KEY_KP,(uint16_t)(params.kp*100));
    nvs_set_u16(storageHandle,KEY_KI,(uint16_t)(params.ki*100));
    nvs_set_u16(storageHandle,KEY_KD,(uint16_t)(params.kd*100));
    nvs_set_i16(storageHandle,KEY_CENTER,(int16_t)(params.centerAngle*100));

    if( nvs_commit(storageHandle) != ESP_OK ){
        printf("ERROR ESCRITURA STORAGEWRITEPIDPARAMS 2\n");
    }
    else{
        nvs_close(storageHandle);
    }
}


pid_params_t storageReadPidParams(void){
    uint16_t kp,ki,kd;
    int16_t centerAngle;
    pid_params_t readParams = {0};

    esp_err_t err = nvs_open(NAMESPACE1,NVS_READWRITE,&storageHandle);
    if( err == ESP_OK){

        esp_err_t err1 =nvs_get_u16(storageHandle,KEY_KP,&kp);
        esp_err_t err2 =nvs_get_u16(storageHandle,KEY_KI,&ki);
        esp_err_t err3 =nvs_get_u16(storageHandle,KEY_KD,&kd);
        esp_err_t err4 =nvs_get_i16(storageHandle,KEY_CENTER,&centerAngle);

        // if( err1 != ESP_OK ){
        //     printf("ERROR EN LECTURA DE PARAMETRO 1\n");
        // }
        // if( err4 != ESP_OK ){
        //     printf("ERROR EN LECTURA DE PARAMETRO 4\n");
        // }
        
        // if( err1 != ESP_OK || err2 != ESP_OK || err3 != ESP_OK || err4 != ESP_OK ){
        //     return readParams;
        // }
        // else if( err2 == ESP_ERR_NVS_NOT_FOUND){
        //     printf("Error el valor no existe");
            
        // }
    }

    readParams.kp = (float)kp/100;
    readParams.ki = (float)ki/100;
    readParams.kd = (float)kd/100;
    readParams.centerAngle = (float)centerAngle/100;

    nvs_close(storageHandle);
    return readParams;
}

