/*
 * DetectorLineas.c
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
#include "drivers/configADC.h"   // TIVA: Funciones de configuracion de ADC para el sensor de distancia
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de TIMERS
#include "driverlib/pwm.h"       // TIVA: Funciones API manejo de modulos PWM
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADCs

// Librerias de FreeRTOS
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "queue.h"              // FreeRTOS: definiciones relacionadas con tareas
#include "event_groups.h"        // FreeRTOS: definiciones relacionadas con grupo de eventos
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos

// Librerias del programador
#include "ControlMotores.h"      // PROGRAMADA: Funciones de control de motores del robot
#include "ControlEncoders.h"     // PROGRAMADA: Funciones de control de encoders (detectores de lineas y encoder de rueda)
#include "ControlSensorDistancia.h" // PROGRAMADA: Funciones de control de sensor de distancia

// Variables globales
uint8_t status;
uint8_t valor;
extern EventGroupHandle_t flagEvents;
extern SemaphoreHandle_t xSemaphore;
extern QueueHandle_t anguloDer;
extern QueueHandle_t anguloIzq;
extern QueueHandle_t distancia;
extern bool girarDerecha;
extern bool girarIzq;
extern bool irRecto;
extern bool permisoSensor;
extern bool permiso_girarDer;
extern bool permiso_irRecto;
extern bool habilitado;
extern bool reinicioIrRecto;
bool cambioIzq = false;
bool cambioDer = false;

// constantes
extern const float MI_PI;


void ControlEncoders_configuraEncoders(void){

    // Habilitamos periferico de los encoders de suelo (PUERTO A)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Habilitamos periferico del encoder de rueda (PUERTO B)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);

    // Definimos pines del puerto A como entradas (TODO: incorporar pines GPIO_PIN_4|GPIO_PIN_5)
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5);

    // Configuramos la interrupcion por flancos de subida
    // TODO: incorporar pines GPIO_PIN_4|GPIO_PIN_5
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_RISING_EDGE);

    // Configuramos la interrupcion del encoder de rueda por flanco de bajada
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5, GPIO_FALLING_EDGE);

    // Se habilitan las interrupciones de los bits del puerto A (correspondiente al encoder de suelo)
    // Indica que pines de entrada generan interrupciones del puerto GPIO
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
    // Hacemos lo mismo con el pin del encoder de rueda
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5);
    // Damos maxima prioridad a los encoders de suelo
    IntPrioritySet(INT_GPIOA, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    // Damos prioridad al encoder de rueda
    IntPrioritySet(INT_GPIOB, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    // Habilitamos las interrupciones del puerto A
    IntEnable(INT_GPIOA);
    // Habilitamos interrupciones del puerto B
    IntEnable(INT_GPIOB);
}

void ControlEncoders_sueloRTI(void){

    float angle = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (habilitado){
        status = GPIOIntStatus(GPIO_PORTA_BASE, true);
        valor = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);

        permisoSensor = false;
        permiso_girarDer = false;
        permiso_irRecto = false;
        reinicioIrRecto = true;
        switch (status){
            case GPIO_PIN_2: // Delantero izquierdo
                      // Blanco
                     if (valor & GPIO_PIN_2){
                          girarDerecha = true;
                          girarIzq = false;
                          irRecto = false;
                          cambioDer = true;
                          angle = 180;
                          xQueueOverwriteFromISR(anguloDer, &angle, &xHigherPriorityTaskWoken);
                      }
                      break;
            case GPIO_PIN_3: // Delantero Derecho
                      // Blanco
                      if (valor & GPIO_PIN_3){
                          girarIzq = true;
                          girarDerecha = false;
                          irRecto = false;
                          cambioIzq = true;
                          angle = 180;
                          xQueueOverwriteFromISR(anguloIzq, &angle, &xHigherPriorityTaskWoken);
                      }
                      break;
            case GPIO_PIN_4: // Trasero izquierdo
                      // Blanco
                     if (valor & GPIO_PIN_4){
                          girarDerecha = true;
                          girarIzq = false;
                          irRecto = false;
                          cambioDer = true;
                          angle = 40;
                          xQueueOverwriteFromISR(anguloDer, &angle, &xHigherPriorityTaskWoken);
                      }
                      break;
           case GPIO_PIN_5:
                      // Trasero derecho
                      if (valor & GPIO_PIN_5){
                          girarIzq = true;
                          girarDerecha = false;
                          irRecto = false;
                          cambioIzq = true;
                          angle = 40;
                          xQueueOverwriteFromISR(anguloIzq, &angle, &xHigherPriorityTaskWoken);
                      }
                     break;
            default:
                break;

         }
    }

    // Antes de finalizar la rutina de interrupcion, borramos los flags de interrupcion, o la interrupcion del puerto continuara activa y se volvera a ejecutar la rutina de interrupción
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2|GPIO_INT_PIN_3|GPIO_INT_PIN_4|GPIO_INT_PIN_5);
    //portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
 }

void ControlEncoders_ruedaRTI(void){

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    status = GPIOIntStatus(GPIO_PORTB_BASE, true);


    if ((girarDerecha) && (status & GPIO_PIN_4)){

         // Activo el flag MOV_GIRAR_DERECHA
         xEventGroupSetBitsFromISR(flagEvents, MOV_GIRAR_DERECHA, &xHigherPriorityTaskWoken);

    }else if ((girarIzq) && (status & GPIO_PIN_5)){

        // Activo el flag MOV_GIRAR_IZQ
        xEventGroupSetBitsFromISR(flagEvents, MOV_GIRAR_IZQ, &xHigherPriorityTaskWoken);

    }else if ((irRecto) && (status & GPIO_PIN_5)){

        // Activo el flag MOV_RECTO
        xEventGroupSetBitsFromISR(flagEvents, MOV_RECTO, &xHigherPriorityTaskWoken);
    }


    // Antes de finalizar la rutina de interrupcion, borramos los flags de interrupcion,
    // o la interrupcion del puerto continuara activa y se volvera a ejecutar la rutina de interrupción

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

