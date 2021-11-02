/*
 * ControlMotores.c
 *
 *  Created on: 25 dic. 2019
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
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADCs
#include "drivers/configADC.h"   // TIVA: Funciones de configuracion de ADC para el sensor de distancia
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

// Variables externas
extern uint32_t g_ui32CPUUsage;
extern EventGroupHandle_t flagEvents;
extern QueueHandle_t anguloDer;
extern QueueHandle_t anguloIzq;
extern QueueHandle_t distancia;
extern QueueHandle_t colaPrincipal;
extern bool girarDerecha;
extern bool girarIzq;
extern bool irRecto;
extern bool permisoSensor;
extern bool permiso_irRecto;
extern bool permiso_girarDer;
extern bool encontrado;
extern bool cambioDer;
extern bool cambioIzq;
bool reinicioIrRecto = false;

// constantes
extern const float MI_PI;


// Configura inicialmente los motores
void ControlMotores_configuraMotores(void){

    //  Habilitamos los perifericos que vamos a usar
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //Configure PWM Clock to match system
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    //Configuramos pines PF2,PF3  como PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //Configuramos las opciones de PWM
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Elegimos el periodo del PWM
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, PERIOD_PWM);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);

    // Ponemos ciclo inicial de trabajo de parada (dos ruedas paradas)
    PWMPulseWidthSet(PWM1_BASE, RUEDA_DER, STOPCOUNT_DERECHA);
    PWMPulseWidthSet(PWM1_BASE, RUEDA_IZQ, STOPCOUNT_IZQ);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
}

void ControlMotores_modificaCicloTrabajo(uint32_t width_izq, uint32_t width_derecha){

    // Actualizamos el ciclo de trabajo de cada motor
    PWMPulseWidthSet(PWM1_BASE, RUEDA_DER, width_derecha);
    // Calibracion software (+10)
    PWMPulseWidthSet(PWM1_BASE, RUEDA_IZQ, width_izq+10);
}


static portTASK_FUNCTION(ControlMotores_irRecto, pvParameters){

    volatile float distRecorrida;
    volatile float anguloRecorrido;
    uint8_t distance;

    while(1){

            distRecorrida = 0;
            anguloRecorrido = 0;
            xQueueReceive(distancia, &distance, portMAX_DELAY);


            while((distRecorrida < distance) && (permiso_irRecto) && (!encontrado) && (irRecto) && (!cambioIzq) && (!cambioDer)){

                xEventGroupWaitBits(flagEvents, MOV_RECTO, pdTRUE, pdFALSE, portMAX_DELAY);

                ControlMotores_modificaCicloTrabajo(COUNT_1MS_MITAD, COUNT_2MS_MITAD);

                anguloRecorrido = anguloRecorrido + (ANGULO_TRIANGULO);
                distRecorrida = anguloRecorrido*2*(RADIO);

                xEventGroupClearBits(flagEvents, MOV_RECTO);

            }


            if ((permisoSensor) && (distRecorrida >= distance) && (permiso_irRecto) && (!reinicioIrRecto)){

                permiso_girarDer = true;
                permiso_irRecto = false;

            } else if ((distRecorrida >= distance) && (permiso_irRecto) && (permisoSensor) && (reinicioIrRecto)){
                reinicioIrRecto = false;
            }



    }

}

static portTASK_FUNCTION(ControlMotores_giroPrincipal, pvParameters){

    float angle;
    volatile float anguloGiroDer;
    volatile float anguloGiroRecorridoDer;

    while(1){
            anguloGiroDer = 0;
            anguloGiroRecorridoDer = 0;

                xQueueReceive(colaPrincipal, &angle, portMAX_DELAY);


                volatile float anguloRadDer = ((angle*2*MI_PI)/360);

                while((anguloGiroRecorridoDer < anguloRadDer) && (permiso_girarDer) && (!encontrado) && (girarDerecha)){

                    // Espera a que se active el flag que salte
                    xEventGroupWaitBits(flagEvents, MOV_GIRAR_DERECHA, pdTRUE, pdFALSE, portMAX_DELAY);

                    ControlMotores_modificaCicloTrabajo(COUNT_2MS_MITAD, COUNT_2MS_MITAD);

                    anguloGiroDer = anguloGiroDer + (ANGULO_TRIANGULO*2);

                    anguloGiroRecorridoDer = ((RADIO*anguloGiroDer)/DIST_ENTRE_RUEDAS);

                    xEventGroupClearBits(flagEvents, MOV_GIRAR_DERECHA);
                }

                if ((permisoSensor) && (anguloGiroRecorridoDer >= anguloRadDer) && (permiso_girarDer)){

                      permiso_irRecto = true;
                      irRecto = true;
                      permiso_girarDer = false;
                }

    }


}

static portTASK_FUNCTION(ControlMotores_girarDerecha, pvParameters){

    float angle;
    volatile float anguloGiroDer;
    volatile float anguloGiroRecorridoDer;

    while(1){

            anguloGiroDer = 0;
            anguloGiroRecorridoDer = 0;
            xQueueReceive(anguloDer, &angle, portMAX_DELAY);
            if (cambioDer){
                 cambioDer = false;
                 reinicioIrRecto = true;
                 girarDerecha = true;
             }

        volatile float anguloRadDer = ((angle*2*MI_PI)/360);


        while((anguloGiroRecorridoDer < anguloRadDer) && (!girarIzq) && (girarDerecha) && (!cambioDer)){

            // Espera a que se active el flag que salte
            xEventGroupWaitBits(flagEvents, MOV_GIRAR_DERECHA, pdTRUE, pdFALSE, portMAX_DELAY);

            ControlMotores_modificaCicloTrabajo(COUNT_2MS_MITAD, COUNT_2MS_MITAD);

            anguloGiroDer = anguloGiroDer + (ANGULO_TRIANGULO*2);

            anguloGiroRecorridoDer = ((RADIO*anguloGiroDer)/DIST_ENTRE_RUEDAS);

            xEventGroupClearBits(flagEvents, MOV_GIRAR_DERECHA);

        }
          girarDerecha = false;
           if ((!permisoSensor) && (anguloGiroRecorridoDer >= anguloRadDer)){
               permisoSensor = true;
               irRecto = true;
               permiso_irRecto = true;
           }


    }

}


static portTASK_FUNCTION(ControlMotores_girarIzq, pvParameters){

    float angle;
    volatile float anguloGiroIzq;
    volatile float anguloGiroRecorridoIzq;

    while(1){

            anguloGiroIzq = 0;
            anguloGiroRecorridoIzq = 0;
            xQueueReceive(anguloIzq, &angle, portMAX_DELAY);
            if (cambioIzq){
               cambioIzq = false;
               reinicioIrRecto = true;
               girarIzq = true;
            }
            volatile float anguloRadIzq = ((angle*2*MI_PI)/360);


            while((anguloGiroRecorridoIzq < anguloRadIzq) && (!girarDerecha) && (!irRecto) && (girarIzq) && (!cambioIzq)){

                // Espera a que se active el flag que salte
                xEventGroupWaitBits(flagEvents, MOV_GIRAR_IZQ, pdTRUE, pdFALSE, portMAX_DELAY);

                ControlMotores_modificaCicloTrabajo(COUNT_1MS_MITAD, COUNT_1MS_MITAD);

                anguloGiroIzq = anguloGiroIzq + (ANGULO_TRIANGULO*2);

                // Ponemos ciclo inicial de trabajo a la mitad del maximo (dos ruedas hacia delante)
                anguloGiroRecorridoIzq = ((RADIO*anguloGiroIzq)/DIST_ENTRE_RUEDAS);


                xEventGroupClearBits(flagEvents, MOV_GIRAR_IZQ);

            }

            girarIzq = false;
            if ((!permisoSensor) && (anguloGiroRecorridoIzq >= anguloRadIzq)){
                permisoSensor = true;
                irRecto = true;
                permiso_irRecto = true;
             }


    }


}

void ControlMotores_creaTareas(void){

    if ((xTaskCreate(ControlMotores_irRecto, (portCHAR *)"irRecto", 512, NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE)){

      while(1);
    }
    if ((xTaskCreate(ControlMotores_girarDerecha, (portCHAR *)"girarDerecha", 512, NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE)){

         while(1);
    }
    if ((xTaskCreate(ControlMotores_girarIzq, (portCHAR *)"girarIzq", 512, NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE)){

         while(1);
    }
    if ((xTaskCreate(ControlMotores_giroPrincipal, (portCHAR *)"giroPrincipal", 512, NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE)){

            while(1);
    }


}
