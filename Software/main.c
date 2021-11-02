/*
 * main.c
 *
 *  Created on: 25 dic. 2019
 *      Author: lolo
 *
 *  Práctica Final Microbótica Curso 2019/20
 *
 *  GR01: Manuel Sánchez Natera y Ahmed Adil Mohamed Pérez
 */
// Librerias del programa

#include <stdint.h>
#include <stdbool.h>

// Librerias que se incluyen tipicamente para configuracion de la TIVA
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de TIMERS
#include "utils/cpu_usage.h"     // TIVA: Controla uso de CPU
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADCs
#include "drivers/configADC.h"   // TIVA: Funciones de configuracion de ADC para el sensor de distancia
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
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

//Globales
uint32_t g_ui32CPUUsage;
uint32_t espera;

EventGroupHandle_t flagEvents;
QueueHandle_t anguloDer;
QueueHandle_t anguloIzq;
QueueHandle_t distancia;
QueueHandle_t colaPrincipal;
bool girarDerecha = false;
bool girarIzq = false;
bool irRecto = false;
bool permisoSensor = true;
bool habilitado = false;

// constantes
const float MI_PI = 3.141592654;

/* PINOUT DEL ROBOT
 * GPIO_PIN_A --> Encoders de suelo
 *      GPIO_PIN_2 : Encoder suelo delantero izquierdo
 *      GPIO_PIN_3 : Encoder suelo delantero derecho
 *      GPIO_PIN_4 : Encoder suelo trasero izquierdo
 *      GPIO_PIN_5 : Encoder suelo trasero derecho
 * GPIO_PIN_B --> Encoder de rueda
 *      GPIO_PIN_4 : Encoder rueda derecha
 *      GPIO_PIN_5 : Encoder rueda izquierda
 * GPIO_PIN_E --> Sensor Distancia
 *      GPIO_PIN_3 : Sensor Distancia
 */

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
// Esta funcion se llama si la biblioteca driverlib o FreeRTOS comprueban la existencia de un error (mediante
// las macros ASSERT(...) y configASSERT(...)
// Los parametros nombrefich y linea contienen informacion de en que punto se encuentra el error...
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea)
{
    while(1) //Si la ejecucion esta aqui dentro, es que el RTOS o alguna de las bibliotecas de perifericos han comprobado que hay un error
    { //Mira el arbol de llamadas en el depurador y los valores de nombrefich y linea para encontrar posibles pistas.
    }
}
#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************
//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

// Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
    static uint8_t count = 0;

    if (++count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        count = 0;
    }
    //return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
//#if 0
void vApplicationIdleHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
        SysCtlSleep();
        //SysCtlDeepSleep();
}
//#endif


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
    while(1);
}


// Funcion principal del programa
int main(void){


    // Elegimos el reloj (16MHz/5 = 3.2MHz)
    SysCtlClockSet( SYSCTL_SYSDIV_5 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Configuracion inicial de motores del robot (inicialmente parados)
    ControlMotores_configuraMotores();

    // Configura cuenta de espera inicial (3s)
    // Habilitamos periferico TIMER2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    // Habilitamos periferico TIMER1
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    // Configuramos TIMER2 para cuenta periodica de 32 bits
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    // Configuramos TIMER1 para cuenta periodica de 32 bits
    //TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    IntPrioritySet(INT_TIMER2A, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    // Habilitamos interrupcion del modulo TIMER
    IntEnable(INT_TIMER2A);
    // Y habilitamos, dentro del modulo TIMER2, la interrupcion "Fin de cuenta"
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    // Carga la cuenta en el TIMER2 (correspondiente a 3s)
    espera = (SysCtlClockGet()*3);
    TimerLoadSet(TIMER2_BASE, TIMER_A, espera);





    /*****          Creacion de tareas          *****/

    // Creo grupo de eventos
    flagEvents = xEventGroupCreate();
    // Crea colas de mensajes para enviar variables
    anguloDer = xQueueCreate(1, sizeof(float));
    anguloIzq = xQueueCreate(1, sizeof(float));
    distancia = xQueueCreate(1, sizeof(uint8_t));
    colaPrincipal = xQueueCreate(1, sizeof(float));

    if ((anguloDer == NULL) || (distancia == NULL) ||(anguloIzq == NULL) || (colaPrincipal == NULL)){
        while(1);
    }


    // Configuracion inicial de detectores (encoders) de suelo y rueda
     ControlEncoders_configuraEncoders();

    // Configura el ADC
    configADC_IniciaADC();

    ControlMotores_creaTareas();
    ControlSensorDistancia_creaTareas();

    // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
    vTaskStartScheduler();
    // el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas
    // De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.

    while(1);
}

void timerEsperaRTI(void){

     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

     // Borramos interrupcion del timer
     TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
     // Deshabilitamos interrupciones del timer
     IntDisable(INT_TIMER2A);
     TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
     // Deshabilitamos operaciones de dicho timer
     TimerDisable(TIMER2_BASE, TIMER_A);

     // Doy permiso a tarea principal
     //habilitado = true;
     xEventGroupSetBitsFromISR(flagEvents, PERMISO_PRINCIPAL, &xHigherPriorityTaskWoken);


     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

