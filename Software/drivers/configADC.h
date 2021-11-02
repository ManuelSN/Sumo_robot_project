#ifndef CONFIGADC_H_
#define CONFIGADC_H_

#include<stdint.h>


typedef struct
{
    uint16_t ch[1];
} MuestrasADC;

typedef struct
{
    MuestrasADC array[1];
} MuestrasARR;

typedef struct
{
	uint32_t chan1;

} MuestrasLeidasADC;


void configADC_ISR(void);
void configADC_DisparaADC(void);
void configADC_LeeADC(MuestrasADC *datos);
void configADC_IniciaADC(void);


#endif /* CONFIGADC_H_ */
