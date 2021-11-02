/*
 * DetectorLineas.h
 *
 *  Created on: 25 dic. 2019
 *      Author: lolo
 */

#ifndef CONTROLENCODERS_H_
#define CONTROLENCODERS_H_

#include <stdint.h>
#include <stdbool.h>

#define MOV_RECTO 0x00001
#define MOV_GIRAR_DERECHA 0x00010
#define MOV_GIRAR_IZQ 0x00011
#define RADIO 2.9
#define MAX_VUELTA 18
#define ANGULO_TRIANGULO ((10*2*MI_PI)/360)
#define DIST_ENTRE_RUEDAS 8.5


void ControlEncoders_configuraEncoders(void);
void ControlEncoders_sueloRTI(void);
void ControlEncoders_ruedaRTI(void);

#endif /* CONTROLENCODERS_H_ */
