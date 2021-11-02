/*
 * ControlSensorDistancia.h
 *
 *  Created on: 28 dic. 2019
 *      Author: lolo
 */

#ifndef CONTROLSENSORDISTANCIA_H_
#define CONTROLSENSORDISTANCIA_H_

#include <stdint.h>
#include <stdbool.h>

#define PERMISO_PRINCIPAL 0x01000
#define ADC_TIMER TIMER1_BASE
#define MAX_CANALES 1
#define MUESTRAS 4


unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax);
void ControlSensorDistancia_creaTareas(void);

#endif //CONTROLSENSORDISTANCIA_H_

