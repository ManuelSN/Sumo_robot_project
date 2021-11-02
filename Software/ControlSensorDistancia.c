/*
 * ControlSensorDistancia.c
 *
 *  Created on: 28 dic. 2019
 *      Author: lolo
 */

#include <stdint.h>
#include <stdbool.h>

// Librerias que se incluyen tipicamente para configuracion de la TIVA
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "utils/cpu_usage.h"     // TIVA: Controla uso de CPU
#include "drivers/configADC.h"   // TIVA: Funciones de configuracion de ADC para el sensor de distancia
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADCs
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de TIMERS
#include "driverlib/pwm.h"       // TIVA: Funciones API manejo de modulos PWM


// Librerias de FreeRTOS
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
#include "event_groups.h"        // FreeRTOS: definiciones relacionadas con grupo de eventos
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos

// Librerias del programador
#include "ControlMotores.h"      // PROGRAMADA: Funciones de control de motores del robot
#include "ControlEncoders.h"     // PROGRAMADA: Funciones de control de encoders (detectores de lineas y encoder de rueda)
#include "ControlSensorDistancia.h" // PROGRAMADA: Funciones de control de sensor de distancia

// Constantes
unsigned short arrayValoresCM[] = {40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 5};
unsigned short arrayValoresMediaVoltaje[] = {0x1DE, 0x1EC, 0x1F3, 0x204, 0x20A, 0x21D, 0x232, 0x24F,
                                                   0x267, 0x296, 0x2DE, 0x320, 0x38E, 0x41C, 0x4C4, 0x593, 0xA84};
// Variables
bool detectado = false;

// Variables externas
extern const float MI_PI;
extern EventGroupHandle_t flagEvents;
extern QueueHandle_t anguloDer;
extern QueueHandle_t anguloIzq;
extern QueueHandle_t distancia;
extern QueueHandle_t colaPrincipal;
extern bool girarDerecha;
extern bool girarIzq;
extern bool irRecto;
extern bool permisoSensor;
bool encontrado = false;
extern bool habilitado;
bool permiso_irRecto = true;
bool permiso_girarDer = false;

static portTASK_FUNCTION(principal,pvParameters){

    uint8_t distance;
    float angle;
    uint8_t estado = 0;

    // Espera permiso para iniciarse
    TimerEnable(TIMER2_BASE, TIMER_A);
    xEventGroupWaitBits(flagEvents, PERMISO_PRINCIPAL, pdTRUE, pdFALSE, portMAX_DELAY);
    habilitado = true;
    while(1){

            switch(estado){
            case 0:
                if ((!encontrado) && (permisoSensor)){
                    if (permiso_irRecto){
                        irRecto = true;
                        distance = 20;
                        xQueueSend(distancia, &distance, portMAX_DELAY);
                    } else if (permiso_girarDer){
                        girarDerecha = true;
                        angle = 180;
                        xQueueSend(colaPrincipal, &angle, portMAX_DELAY);
                    }
                } else{
                    estado = 1;
                }
                break;
            case 1:
                if ((encontrado) && (permisoSensor)){
                    ControlMotores_modificaCicloTrabajo(COUNT_1MS, COUNT_2MS);
                }else {
                    estado = 0;
                }
                break;
            }
    }

}
static portTASK_FUNCTION(buscarObjetivo,pvParameters){


    MuestrasADC media;

    unsigned short indice = 0;


    while(1){

        if (habilitado){
            // Espera y lee muestras del ADC (BLOQUEANTE)
            configADC_LeeADC(&media);


            // Llamo a la funcion que busca el indice del valor más aproximado a nuestra medida, para posteriormente coger su conversión en cm.
            indice = binary_lookup(arrayValoresMediaVoltaje, media.ch[0], 0, 16);


            if ((arrayValoresCM[indice] >= 5) && (arrayValoresCM[indice] <= 28)){

                encontrado = true;

            } else{
                encontrado = false;

            }
        }
    }

}

// Busqueda binaria del valor correspondiente en cm de la medida
unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax){

  unsigned int imid;

  while (imin < imax){

      imid= (imin+imax)>>1;

      if (A[imid] < key)
        imin = imid + 1;
      else
        imax = imid;
   }
   return imax;    //Al final imax=imin y en dicha posicion hay un numero mayor o igual que el buscado
}

void ControlSensorDistancia_creaTareas(void){

    if((xTaskCreate(buscarObjetivo, (portCHAR *)"buscar", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE)){
           while(1);
    }
    if((xTaskCreate(principal, (portCHAR *)"principal", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE)){
              while(1);
       }

}
