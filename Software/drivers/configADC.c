#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


static QueueHandle_t cola_adc;

//Provoca el disparo de una conversion (hemos configurado el ADC con "disparo software" (Processor trigger)
void configADC_DisparaADC(void)
{
	ADCProcessorTrigger(ADC0_BASE,3);
}


void configADC_IniciaADC(void)
{
        uint32_t ui32period;

        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
        SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

        // Habilitamos periferico TIMER3
        SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
        TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
        ui32period = SysCtlClockGet()/2000;
        TimerLoadSet(TIMER3_BASE, TIMER_A, ui32period);
        TimerControlTrigger(TIMER3_BASE, TIMER_A, true);
        TimerEnable(TIMER3_BASE, TIMER_A);

        //HABILITAMOS EL GPIOE
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
        // Enable pin PE3 for ADC AIN0
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

        // Hago oversample, recoge 4 muestras y hace la media
         ADCHardwareOversampleConfigure(ADC0_BASE, 64);

        //CONFIGURAR SECUENCIADOR 3 - Una unica muestra
        //Configuramos la velocidad de conversion al maximo (1MS/s)
        ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);
        //Disparo software (disparo periodico con TIMER0)
        ADCSequenceConfigure(ADC0_BASE, 3,ADC_TRIGGER_TIMER,0);
        ADCSequenceStepConfigure(ADC0_BASE, 3,0,ADC_CTL_CH0|ADC_CTL_IE |ADC_CTL_END);  //La ultima muestra provoca la interrupcion
        ADCSequenceEnable(ADC0_BASE, 3); //ACTIVO LA SECUENCIA

        //Habilita las interrupciones
        ADCIntEnable(ADC0_BASE, 3);
        IntPrioritySet(INT_ADC0SS3,configMAX_SYSCALL_INTERRUPT_PRIORITY);
        IntEnable(INT_ADC0SS3);


        //Creamos una cola de mensajes para la comunicacion entre la ISR y la tarea que llame a configADC_LeeADC(...)
        cola_adc=xQueueCreate(1,sizeof(MuestrasADC));
        if (cola_adc==NULL)
        {
            while(1);
        }
}


void configADC_LeeADC(MuestrasADC *datos)
{
	xQueueReceive(cola_adc,datos,portMAX_DELAY);
}

void configADC_ISR(void)
{
	portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

	MuestrasLeidasADC leidas;
	MuestrasADC finales;
	ADCIntClear(ADC0_BASE, 3);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
	ADCSequenceDataGet(ADC0_BASE, 3,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS

	//Pasamos de 32 bits a 16 (el conversor es de 12 bits, as� que s�lo son significativos los bits del 0 al 11)
	finales.ch[0]=leidas.chan1;


	//Guardamos en la cola
	xQueueOverwriteFromISR(cola_adc,&finales,&higherPriorityTaskWoken);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
