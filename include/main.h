/*
 * main.h
 *
 *  Created on: Apr 19, 2016
 *      Author: talman
 */

#ifndef MAIN_H_
#define MAIN_H_

/* DMA Handle callback */
void dmaM0Complete(DMA_HandleTypeDef *hdma);
void dmaM1Complete(DMA_HandleTypeDef *hdma);
void halfComplete(DMA_HandleTypeDef *hdma);
void dmaError(DMA_HandleTypeDef *hdma);




#endif /* MAIN_H_ */
