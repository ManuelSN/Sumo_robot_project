//*****************************************************************************
// i2c_if.c
//
// I2C interface APIs. Operates in a polled mode.
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//Adaptada a FreeRTOS por J. M. Cano,  E Gonzalez e Ignacio Herrero
// Para mejorar la eficiencia gestion del bus I2C aprovechando el RTOS se utilizan los siguientes mecanismos:
// -> Las transacciones I2C las realiza una rutina de interrupci�n
// -> Las funciones I2C_IF_Read, I2C_IF_ReadFrom e I2C_IF_Write envian los datos de la transaccion a realizar mediante una cola de mensaje.
// -> La ISR va procesando las peticiones de transaccion/realizando nuevas transacciones. Cuando finaliza cada transaccion utiliza las DirectToTaskNotification para desbloquear la tarea.
// Se mantiene la compatibilidad hacia atras, por eso las funciones de las bibliotecas bma222drv.c y tmp000drv.c no hay que cambiarlas.

//2018. Adaptado de la CC3200 a la TIVA.
// --> La TIVA no tiene flag de interrupcion por error y otras causas (NACK), hay que tratarlo de otra manera (Mediante una funcion que comrprueba si ha habido error).
// --> Arreglado un fallo por el cual no saltaba la notificaci�n directa a tarea en el caso de este error.

// Standard includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

// Common interface include
#include "i2c_if.h"

//Include FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "portmacro.h"


typedef struct {
	TaskHandle_t OriginTask;	/* Tarea que origina la peticion */
	uint8_t *buffer;	/* puntero a los datos TX/RX */
	uint8_t rxlenght;	/* longitud a recibir */
	uint8_t txlenght;	/* longitud a transmitir */
	uint8_t command;	/* comando */
	uint8_t dev_address; /* direccion I2C */
} I2C_Transaction;



//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define SYS_CLK                 configCPU_CLOCK_HZ
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//ISR states....
#define STATE_IDLE              0
#define STATE_WRITE_NEXT        1
#define STATE_WRITE_FINAL       2
#define STATE_READ_NEXT         3
#define STATE_READ_FINAL        4

//Comandos (tipos de transaccion) que se pueden realizar
#define I2C_COMMAND_WRITE 0
#define I2C_COMMAND_READ 1
#define I2C_COMMAND_READ_FROM 2

//Numero maximo de ordenes de transaccion que se pueden acumular en la cola
#define MAX_I2C_TRANSACTIONS 16

//Flags para las DirectToTaskNotifications
#define I2C_NOTIFY_READ_COMPLETE (0x01)
#define I2C_NOTIFY_WRITE_COMPLETE (0x02)
#define I2C_NOTIFY_ERR (0x04)


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************
static int I2CTransact(unsigned long ulCmd);


//Controla los estados que atraviesa la ISR mientras se va realizando la transaccion
//De esta forma cada vez que salta la ISR se ejecuta un comportamiento que depende de su estado
static volatile uint16_t g_i2cisrstate=STATE_IDLE;
static QueueHandle_t g_I2Cqueue;


//****************************************************************************
//
//! Invokes the transaction over I2C
//!
//! \param ulCmd is the command to be executed over I2C
//! 
//! This function works in a polling mode,
//!    1. Initiates the transfer of the command.
//!    2. Waits for the I2C transaction completion
//!    3. Check for any error in transaction
//!    4. Clears the master interrupt
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
static int 
I2CTransact(unsigned long ulCmd)
{
    //
    // Clear all interrupts
    //
    MAP_I2CMasterIntClearEx(I2C3_BASE,MAP_I2CMasterIntStatusEx(I2C3_BASE,false));
    //
    // Set the time-out. Not to be used with breakpoints.
    //
    MAP_I2CMasterTimeoutSet(I2C3_BASE, I2C_TIMEOUT_VAL);
    //
    // Initiate the transfer.
    //
    MAP_I2CMasterControl(I2C3_BASE, ulCmd);


    //
    // Check for any errors in transfer
    //
    if(MAP_I2CMasterErr(I2C3_BASE) != I2C_MASTER_ERR_NONE)
    {
        switch(ulCmd)
        {
        case I2C_MASTER_CMD_BURST_SEND_START:
        case I2C_MASTER_CMD_BURST_SEND_CONT:
        case I2C_MASTER_CMD_BURST_SEND_STOP:
            MAP_I2CMasterControl(I2C3_BASE,
                         I2C_MASTER_CMD_BURST_SEND_ERROR_STOP);
            break;
        case I2C_MASTER_CMD_BURST_RECEIVE_START:
        case I2C_MASTER_CMD_BURST_RECEIVE_CONT:
        case I2C_MASTER_CMD_BURST_RECEIVE_FINISH:
            MAP_I2CMasterControl(I2C3_BASE,
                         I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP);
            break;
        default:
            break;
        }
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Invokes the I2C driver APIs to write to the specified address
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucData is the pointer to the data to be written
//! \param ucLen is the length of data to be written
//! \param ucStop should be set to 1. Mantained for compatibility
//! 
//! This function works in a polling mode,
//!    1. Writes the device register address to be written to.
//!    2. In a loop, writes all the bytes over I2C
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Write(unsigned char ucDevAddr,
		unsigned char *pucData,
		unsigned char ucLen,
		unsigned char ucStop)
{
	uint32_t notifVal=0;
	I2C_Transaction transaction;

	RETERR_IF_TRUE(pucData == NULL);
	RETERR_IF_TRUE(ucLen == 0);
	RETERR_IF_TRUE(ucStop == 0); //XXX quitar parametro ucStop!!


	transaction.OriginTask=xTaskGetCurrentTaskHandle();
	transaction.buffer=pucData;
	transaction.txlenght=ucLen;
	transaction.rxlenght=0;
	transaction.dev_address=ucDevAddr;
	transaction.command=I2C_COMMAND_WRITE;

	//Envia la transaccion a la cola de mensajes...
	xQueueSend(g_I2Cqueue,&transaction,portMAX_DELAY);

	if (g_i2cisrstate==STATE_IDLE)
	{
		IntPendSet(INT_I2C3);	//Produce un disparo software de la ISR (comienza a transmitir)....
	}

	//Espera a que se produzca la transacci�n (o haya error)...
	while (!(notifVal&(I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_ERR)))
	{
		xTaskNotifyWait( 0, I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
	}

	if (notifVal&I2C_NOTIFY_ERR) return FAILURE;

	return SUCCESS;

}

//****************************************************************************
//
//! Invokes the I2C driver APIs to read from the device. This assumes the 
//! device local address to read from is set using the I2CWrite API.
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucData is the pointer to the read data to be placed
//! \param ucLen is the length of data to be read
//! 
//! This function works in a polling mode,
//!    1. Writes the device register address to be written to.
//!    2. In a loop, reads all the bytes over I2C
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Read(unsigned char ucDevAddr,
		unsigned char *pucData,
		unsigned char ucLen)
{
	//unsigned long ulCmdID;
	uint32_t notifVal=0;
	I2C_Transaction transaction;

	RETERR_IF_TRUE(pucData == NULL);
	RETERR_IF_TRUE(ucLen == 0);


	transaction.OriginTask=xTaskGetCurrentTaskHandle();
	transaction.buffer=pucData;
	transaction.txlenght=0;
	transaction.rxlenght=ucLen;
	transaction.dev_address=ucDevAddr;
	transaction.command=I2C_COMMAND_READ;

	xQueueSend(g_I2Cqueue,&transaction,portMAX_DELAY);

	if (g_i2cisrstate==STATE_IDLE)
	{
		IntPendSet(INT_I2C3);	//Produce un disparo software (comienza a transmitir)....
	}


	//Espera a que se produzca la transacci�n (o haya error)...
	while (!(notifVal&(I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR)))
	{
		xTaskNotifyWait( 0, I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
	}

	if (notifVal&I2C_NOTIFY_ERR) return FAILURE;

	return SUCCESS;
}

//****************************************************************************
//
//! Invokes the I2C driver APIs to read from a specified address the device. 
//! This assumes the device local address to be of 8-bit. For other 
//! combinations use I2CWrite followed by I2CRead.
//!
//! \param ucDevAddr is the 7-bit I2C slave address
//! \param pucWrDataBuf is the pointer to the data to be written (reg addr)
//! \param ucWrLen is the length of data to be written
//! \param pucRdDataBuf is the pointer to the read data to be placed
//! \param ucRdLen is the length of data to be read
//! 
//! This function works in a polling mode,
//!    1. Writes the data over I2C (device register address to be read from).
//!    2. In a loop, reads all the bytes over I2C
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_ReadFrom(unsigned char ucDevAddr,
            unsigned char *pucWrDataBuf,
            unsigned char ucWrLen,
            unsigned char *pucRdDataBuf,
            unsigned char ucRdLen)
{
	I2C_Transaction transaction;
	uint32_t notifVal=0;
	volatile int i=0;

	    RETERR_IF_TRUE(pucRdDataBuf == NULL);
	    RETERR_IF_TRUE(pucWrDataBuf == NULL);
	    RETERR_IF_TRUE(ucWrLen == 0);
	    RETERR_IF_TRUE(ucWrLen > ucWrLen);

	    memcpy(pucRdDataBuf,pucWrDataBuf,ucWrLen);
	    transaction.OriginTask=xTaskGetCurrentTaskHandle();
	    transaction.buffer=pucRdDataBuf;
	    transaction.txlenght=ucWrLen;
	    transaction.rxlenght=ucRdLen;
	    transaction.dev_address=ucDevAddr;
	    transaction.command=I2C_COMMAND_READ_FROM;

	    xQueueSend(g_I2Cqueue,&transaction,portMAX_DELAY);

	    if (g_i2cisrstate==STATE_IDLE)
	    {
	    	IntPendSet(INT_I2C3);	//Produce un disparo software....
	    }

	    //CEspera a que se complete la operacion de escritura/lectura o se produza error
	    while (!(notifVal&(I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR)))
	    {
	    	xTaskNotifyWait( 0, I2C_NOTIFY_WRITE_COMPLETE|I2C_NOTIFY_READ_COMPLETE|I2C_NOTIFY_ERR, &notifVal, portMAX_DELAY);
	    	i++;
	    }

	    if (notifVal&I2C_NOTIFY_ERR)
	    {
	        return FAILURE;
	    }

    return SUCCESS;
}


//Rutina de interrupcion.
//Esta rutina parece muy larga, pero s�lo se ejecuta una parte u otra seg�n el estado en el que estemos...
//Utiliza una m�quina de estados para cambiar el comportamiento cuando se produce la interrupcion, ya que lo que se debe realizar depende de si estamos o no
// en una transacci�n, del tipo de transaccion (escritura, lectura o escritura-lectura, y de que punto de dicha transacci�n estamos.
// Para ello se utiliza la variable de estado g_i2cisrstate.

void I2C_IF_ISR(void)
{
	BaseType_t xHigherPriorityTaskWoken=pdFALSE;

	//The following two variables should maintain values between calls to the ISR
	static I2C_Transaction transaction;
	static uint8_t *tmpptr;

	I2CMasterIntClear(I2C3_BASE); //Borra el flag de interrupcion

	//Primero tratamos posible error... (por ejemplo NACK)
	if (I2CMasterErr(I2C3_BASE)&&(g_i2cisrstate!=STATE_IDLE))
	{
		I2CMasterIntDisable(I2C3_BASE);
		xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
		xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
		switch (g_i2cisrstate)
		{
			case STATE_READ_NEXT:
		 	case STATE_READ_FINAL:
				I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
				break;
			case STATE_WRITE_NEXT:
			case STATE_WRITE_FINAL:
				I2CMasterControl(I2C3_BASE,I2C_MASTER_CMD_BURST_SEND_FINISH);
				break;
		}
		g_i2cisrstate=STATE_IDLE;
		if (xQueuePeekFromISR(g_I2Cqueue,&transaction))
		{
		        IntPendSet(INT_I2C3);
		}
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);    //Esto es necesario antes del return...
		return;
	}

	//Ejecuta la maquina de estados para responder al evento. Miramos primero en qu� estado estamos.
	switch(g_i2cisrstate)
	{
		case STATE_IDLE:
		{
			//Arranca la transaccion....
			if (xQueuePeekFromISR(g_I2Cqueue,&transaction)==pdTRUE)
			{
				//Hay algo en la cola... puedo comenzar
				switch (transaction.command)
				{
					case I2C_COMMAND_WRITE:
					case I2C_COMMAND_READ_FROM:
					{	//Disparo una escritura
						tmpptr=transaction.buffer;
						MAP_I2CMasterSlaveAddrSet(I2C3_BASE, transaction.dev_address, false); // Set I2C codec slave address
						I2CMasterDataPut(I2C3_BASE, *tmpptr);	 // Write the first byte to the controller.
						if (I2CTransact(I2C_MASTER_CMD_BURST_SEND_START)==SUCCESS)
						{
							transaction.txlenght--;
							tmpptr++;
							I2CMasterIntEnable(I2C3_BASE);
							if (transaction.txlenght>0)
								g_i2cisrstate=STATE_WRITE_NEXT;	//Cambia de estado. La proxima interrupcion Escribe otro caracter
							else
								g_i2cisrstate=STATE_WRITE_FINAL; //Cambia de estado. La proxima interrupcion finaliza la transmision
						}
						else
						{	//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
							//Luego borro los flags de interrupci�n y compruebo si hay mas transacciones pendientes
							xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
							xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
							if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
						}
					}
					break;
					case I2C_COMMAND_READ:
					{   //Disparo una lectura (simple o multiple, segun el caso)
						tmpptr=transaction.buffer;
						MAP_I2CMasterSlaveAddrSet(I2C3_BASE, transaction.dev_address, true); // Set I2C codec slave address
						transaction.rxlenght--;
						if(transaction.rxlenght==0)
						{	//Lectura simple
							if (I2CTransact(I2C_MASTER_CMD_SINGLE_RECEIVE)==SUCCESS)
							{
								g_i2cisrstate=STATE_READ_FINAL; //La siguiente ISR sera el final de lectura
								I2CMasterIntEnable(I2C3_BASE); //,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
							}
							else
							{
								//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
								//Luego borro los flags de interrupci�n y compruebo si hay mas transacciones pendientes
								xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
								xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
								if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);

							}
						}
						else
						{	//lectura multiple
							if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_START)==SUCCESS)
							{
								g_i2cisrstate=STATE_READ_NEXT;	//La siguente ISR sera la recepcion de un dato
								I2CMasterIntEnable(I2C3_BASE); //,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
							}
							else
							{
								//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
								//Luego borro los flags de interrupci�n y compruebo si hay mas transacciones pendientes
								xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
								xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
								//I2CMasterIntClearEx(I2C3_BASE,i2cflags);
								if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
							}
						}
					}
					break;
				}
			}
			else
			{	//No habia nada en la cola de ordenes I2C
				//��Que ha pasao??
			}
		}
		break; //FIN DEL CASO STATE_IDLE...

		case STATE_WRITE_NEXT:
		{	//Continuacion de escritura...Envio el siguiente byte
			I2CMasterDataPut(I2C3_BASE, *tmpptr);
			if (I2CTransact(I2C_MASTER_CMD_BURST_SEND_CONT)==SUCCESS)
			{
				transaction.txlenght--;
				tmpptr++;
				if (transaction.txlenght>0)
					g_i2cisrstate=STATE_WRITE_NEXT; //Si hay mas que enviar, no cambio de estado (en realidad podia no hacer nada)
				else
					g_i2cisrstate=STATE_WRITE_FINAL; //Si no hay mas que enviar, la siguiente ISR finaliza la transmision (condicion de STOP)
			}
			else
			{
				//Fallo de transmision. Elimino la transaccion en curso y aviso a la tarea
				//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
				//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
				I2CMasterIntDisable(I2C3_BASE);
				g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
				xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
				xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
				if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
			}
		}
		break; //FIN DEL ESTADO STATE_WRITE_NEXT

		case STATE_WRITE_FINAL:
		{	//Fin transmision. Si era una transmision simple, finalizo y paso a la siguiente
			if (transaction.command!=I2C_COMMAND_READ_FROM)
			{
				//Transaccion finalizada
				//Deshabilito las ISR, vuelvo al estado IDLE, elimino la transaccion de la cola
				//Finalmente compruebo si hay almacenada en la cola para dispararla
				I2CTransact(I2C_MASTER_CMD_BURST_SEND_STOP);
				I2CMasterIntDisable(I2C3_BASE);
				g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
				xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
				xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_WRITE_COMPLETE,eSetBits,&xHigherPriorityTaskWoken);	//Transaccion correcta
				if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
			}
			else //Si era la operacion READ_FROM...
			{
				//Operacion READ_FROM. Finaliza la parte de envio, ahora pasamos a recepcion
				//Comenzar una recepcion!!!
				tmpptr=transaction.buffer;
				MAP_I2CMasterSlaveAddrSet(I2C3_BASE, transaction.dev_address, true); // Set I2C codec slave address
				transaction.rxlenght--;
				if(transaction.rxlenght==0)
				{	//Recepcion de un solo byte
					if (I2CTransact(I2C_MASTER_CMD_SINGLE_RECEIVE)==SUCCESS)
					{
						g_i2cisrstate=STATE_READ_FINAL;	//Cambia de estado a finalizar. La proxima ISR finaliza la RX
						I2CMasterIntEnable(I2C3_BASE);;
					}
					else
					{	//Error
						//Fallo de recepci�n. Elimino la transaccion en curso y aviso a la tarea
						//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
						//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
						I2CMasterIntDisable(I2C3_BASE);
						g_i2cisrstate=STATE_IDLE;	//Vuelve al estado IDLE
						xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
						xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
						if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
					}
				}
				else
				{	//Recepcion de varios bytes
					if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_START)==SUCCESS)
					{
						g_i2cisrstate=STATE_READ_NEXT;	//La proxima ISR continua la recepcion
						I2CMasterIntEnable(I2C3_BASE);
					}
					else
					{
						//Fallo de recepci�n. Elimino la transaccion en curso y aviso a la tarea
						//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
						//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
					    //La verdad es que esto podia ponerlo en una subfuncion, porque se repite muchiiiiisimo...
						I2CMasterIntDisable(I2C3_BASE);
						g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
						xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
						xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
						if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
					}
				}
			}
		}
		break; //FIN DEL ESTADO STATE_WRITE_FINAL

		case STATE_READ_NEXT:
		{	//Lectura "larga" en curso... Intento leer datos  continuar...
			*tmpptr = MAP_I2CMasterDataGet(I2C3_BASE);
			transaction.rxlenght--;
			tmpptr++;
			if (transaction.rxlenght==0)
			{	//Ya no tengo que recibir mas. Ordeno la recepcion del ultimo byte y la condicion de stop
				if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_FINISH)==SUCCESS)
				{
					g_i2cisrstate=STATE_READ_FINAL;	//Ultimo dato, la siguiente ISR finaliza la recepci�n
				}
				else
				{
					//Fallo de recepci�n. Elimino la transaccion en curso y aviso a la tarea
					//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
					//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
					I2CMasterIntDisable(I2C3_BASE);
					g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
					xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
					xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
					if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
				}
			}
			else
			{	//Tengo que continuar recibiendo
				if (I2CTransact(I2C_MASTER_CMD_BURST_RECEIVE_CONT)==SUCCESS)
				{
					//No state change
				}
				else
				{
					//Fallo de recepci�n. Elimino la transaccion en curso y aviso a la tarea
					//Deshabilita las ISR y vuelve al estado inicial. De parar la transaccion ya se encarga internamente I2CTransact (creo)
					//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
					I2CMasterIntDisable(I2C3_BASE); //,I2C_MASTER_INT_DATA|I2C_MASTER_INT_TIMEOUT|I2C_MASTER_INT_NACK);
					g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
					xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
					xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_ERR,eSetBits,&xHigherPriorityTaskWoken);
					if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
				}
			}
		}
		break; //FIN DEL ESTADO STATE_READ_NEXT
		case STATE_READ_FINAL:
		{
			//Fin de la lectura/recepcion. Elimino la transaccion en curso y aviso a la tarea
			//Deshabilita las ISR y vuelve al estado inicial.
			//Finalmente ompruebo si hay mas transacciones pendientes para volver a lanzar una
			//Ademas borro los flags de interrupcion (aqui no se llama a transact)
			*tmpptr = MAP_I2CMasterDataGet(I2C3_BASE);
			I2CMasterIntDisable(I2C3_BASE);
			g_i2cisrstate=STATE_IDLE; //Vuelve al estado IDLE
			xQueueReceiveFromISR(g_I2Cqueue,&transaction,&xHigherPriorityTaskWoken);
			xTaskNotifyFromISR(transaction.OriginTask,I2C_NOTIFY_READ_COMPLETE,eSetBits,&xHigherPriorityTaskWoken);
			if (xQueuePeekFromISR(g_I2Cqueue,&transaction)) IntPendSet(INT_I2C3);
		}
		break; //FIN DEL ESTADO STATE_READ_FINAL
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


//****************************************************************************
//
//! Enables and configures the I2C peripheral
//!
//! \param ulMode is the mode configuration of I2C
//! The parameter \e ulMode is one of the following
//! - \b I2C_MASTER_MODE_STD for 100 Kbps standard mode.
//! - \b I2C_MASTER_MODE_FST for 400 Kbps fast mode.
//! 
//! This function works in a polling mode,
//!    1. Powers ON the I2C peripheral.
//!    2. Configures the I2C peripheral
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Open(unsigned long ulMode)
{

    // Inicializacion del interfaz con los sensores
    //
    // The I2C3 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    MAP_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1,
    GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD_WPU);
    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //por el clock gating
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C3);


    //
    // Configure I2C module in the specified mode
    //
    switch(ulMode)
    {
        case I2C_MASTER_MODE_STD:       /* 100000 */
            MAP_I2CMasterInitExpClk(I2C3_BASE,SYS_CLK,false);
            break;

        case I2C_MASTER_MODE_FST:       /* 400000 */
            MAP_I2CMasterInitExpClk(I2C3_BASE,SYS_CLK,true);
            break;

        default:
            MAP_I2CMasterInitExpClk(I2C3_BASE,SYS_CLK,true);
            break;
    }

    
    //Empezamos por estado IDLE (no hay transaccion en marcha)
    g_i2cisrstate=STATE_IDLE;

    g_I2Cqueue=xQueueCreate(MAX_I2C_TRANSACTIONS,sizeof(I2C_Transaction));
    if (g_I2Cqueue==NULL)
    	while(1);

    MAP_IntPrioritySet(INT_I2C3,configMAX_SYSCALL_INTERRUPT_PRIORITY); //jose: La prioridad debe ser mayor o igual que configMAX_SYSCALL_INTERRUPT_PRIORITY
    MAP_IntEnable(INT_I2C3);

    return SUCCESS;
}

//****************************************************************************
//
//! Disables the I2C peripheral
//!
//! \param None
//! 
//! This function works in a polling mode,
//!    1. Powers OFF the I2C peripheral.
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int 
I2C_IF_Close()
{
    //
    // Power OFF the I2C peripheral
    //

	//xxx Deber�a comprobarse antes si no hay una transaccion en marcha!!!

	IntDisable(INT_I2C3);	//Deshabilita la ISR
	SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C3);
    if (g_I2Cqueue!=NULL)
    {
    		vQueueDelete(g_I2Cqueue);
    		g_I2Cqueue=NULL;
    }

    return SUCCESS;
}



