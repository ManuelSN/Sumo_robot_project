/*
 * ControlMotores.h
 *
 *  Created on: 25 dic. 2019
 *      Author: lolo
 */

#ifndef CONTROLMOTORES_H_
#define CONTROLMOTORES_H_

#include <stdint.h>
#include <stdbool.h>


#define PERIOD_PWM 64000   // Ciclos de reloj para conseguir una signal periodica de 50Hz = 20ms -->16Mhz%5 = 3.2MHz/ 50Hz = 64000ticks un periodo completo
#define COUNT_1MS 3200  // Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido) --> 1ms/20ms = 0.05% de ticks* 64000 = 3200
#define STOPCOUNT_DERECHA 4864      // Ciclos para amplitud de pulso de parada (1.52ms) -->  1.52ms/20ms = 0.076% de ticks*64000 = 4864
#define COUNT_2MS 6400   // Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido) --> 2ms/20ms = 0.1% de ticks* 64000 = 6400
#define NUM_STEPS 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud tras pulsacion
#define RUEDA_DER PWM_OUT_6 // Salida de PWM de rueda derecha
#define RUEDA_IZQ PWM_OUT_7 // Salida de PWM de rueda izquierda
#define STOPCOUNT_IZQ 4874  // Calibrado software de parada de rueda izquierda
#define COUNT_2MS_MITAD 5632
#define COUNT_1MS_MITAD 4032



void ControlMotores_configuraMotores(void);
void ControlMotores_modificaCicloTrabajo(uint32_t width_izq, uint32_t width_derecha);
void ControlMotores_creaTareas(void);



#endif /* CONTROLMOTORES_H_ */
