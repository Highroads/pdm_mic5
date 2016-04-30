//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
#include "main.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "mxconstants.h"
#include "RingBuffer.h"

// ----------------------------------------------------------------------------
//
// Semihosting STM32F4 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate semihosting, display a message on the standard output
// and another message on the standard error.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;
/* Prescaler declaration */
static RingBuffer fifo(1024);
uint32_t uwPrescalerValue = 0;
int32_t output=0;
uint16_t dacoutput=0;
UART_HandleTypeDef huart2;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* DMA Handle callback */
void dmaM0Complete(DMA_HandleTypeDef *hdma);
void dmaM1Complete(DMA_HandleTypeDef *hdma);
void dmaHalfComplete(DMA_HandleTypeDef *hdma);
void dmaError(DMA_HandleTypeDef *hdma);
//
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_USART2_UART_Init(void);
void MX_ADC1_Init(void);
void MX_DAC_Init(void);
void MX_I2S_Init(void);


namespace
{
  // ----- Timing definitions -------------------------------------------------

  // Keep the LED on for 2/3 of a second.
  constexpr Timer::ticks_t BLINK_ON_TICKS = Timer::FREQUENCY_HZ * 3 / 4;
  constexpr Timer::ticks_t BLINK_OFF_TICKS = Timer::FREQUENCY_HZ
      - BLINK_ON_TICKS;
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
// ----- LED definitions ------------------------------------------------------

#if defined(STM32F401xE)

#warning "Assume a NUCLEO-F401RE board, PA5, active high."

// PA5
#define BLINK_PORT_NUMBER         (0)
#define BLINK_PIN_NUMBER          (5)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[1] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER, BLINK_ACTIVE_LOW },
  };

#elif defined(STM32F407xx)

#warning "Assume a STM32F4-Discovery board, PC12-PC15, active high."

#define BLINK_PORT_NUMBER         (3)
#define BLINK_PIN_NUMBER_GREEN    (12)
#define BLINK_PIN_NUMBER_ORANGE   (13)
#define BLINK_PIN_NUMBER_RED      (14)
#define BLINK_PIN_NUMBER_BLUE     (15)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[4] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_GREEN, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_ORANGE, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_RED, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_BLUE, BLINK_ACTIVE_LOW },
  };

#elif defined(STM32F411xE)

#warning "Assume a NUCLEO-F411RE board, PA5, active high."

#define BLINK_PORT_NUMBER         (0)
#define BLINK_PIN_NUMBER          (5)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[1] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER, BLINK_ACTIVE_LOW },
  };

#elif defined(STM32F429xx)

#warning "Assume a STM32F429I-Discovery board, PG13-PG14, active high."

#define BLINK_PORT_NUMBER         (6)
#define BLINK_PIN_NUMBER_GREEN    (13)
#define BLINK_PIN_NUMBER_RED      (14)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLeds[2] =
  {
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_GREEN, BLINK_ACTIVE_LOW },
    { BLINK_PORT_NUMBER, BLINK_PIN_NUMBER_RED, BLINK_ACTIVE_LOW },
  };

#else

#warning "Unknown board, assume PA5, active high."

#define BLINK_PORT_NUMBER         (0)
#define BLINK_PIN_NUMBER          (5)
#define BLINK_ACTIVE_LOW          (false)

BlinkLed blinkLed(BLINK_PORT_NUMBER, BLINK_PIN_NUMBER, BLINK_ACTIVE_LOW);

#endif

//------ set up pdm function --------------------------------------------------
#undef PDMTEST
#ifdef PDMTEST
uint16_t testcount = 0;
const uint16_t testsamples[8] = {
        /*          SPL
         * shift    100%FS   Aztec    F0, Hz
         *    0                       7812.5
         *    1     -150     -150     3906.25
         *    2      184      172     1953.125
         *    3      202      190      976.5625
         *    4      206      192      488.28125
         *    5      208      194      244.140625
         *    6      208      194      122.0703125
         *    7      208      194       61.03515625
         *    8      210      194       30.517578125
         *    9      210      196       15.2587890625
         */
        0xffff, 0xffff, 0xffff, 0xffff, 0x0000, 0x0000, 0x0000, 0x0000, // 100%FS
//      0x5555, 0xf5f5, 0xffff, 0x5f5f, 0x5555, 0x5050, 0x0000, 0x0505, // Aztec
};
uint8_t TESTSHIFT=3;
#define TESTMASK (((sizeof testsamples) / (sizeof testsamples[0]))-1)
#endif

/* CIC filter state */
#define CIC2_R 8
typedef int32_t CICREG;
CICREG s2_sum1 = 0;
CICREG s2_comb1_1 = 0;
CICREG s2_comb1_2 = 0;
CICREG s2_sum2 = 0;
CICREG s2_comb2_1 = 0;
CICREG s2_comb2_2 = 0;
CICREG s2_sum3 = 0;
CICREG s2_comb3_1 = 0;
CICREG s2_comb3_2 = 0;
int s2_count = CIC2_R/2;

#define TWOSTEPCIC
#ifdef TWOSTEPCIC
const int8_t pdmsum8[256] = {
#   define S(n) (2*(n)-8)
#   define B2(n) S(n),  S(n+1),  S(n+1),  S(n+2)
#   define B4(n) B2(n), B2(n+1), B2(n+1), B2(n+2)
#   define B6(n) B4(n), B4(n+1), B4(n+1), B4(n+2)
        B6(0), B6(1), B6(1), B6(2)
#undef S
#undef B6
#undef B4
#undef B2
};
#endif


#define I2S_BUFFERSIZE 256
#define DMA_TRANSFERCOUNT (I2S_BUFFERSIZE<<1)
uint8_t Buffer0_rdy=0;
uint8_t Buffer1_rdy=0;


RingBuffer circbuffer(512);
uint16_t dmabuffer[2][DMA_TRANSFERCOUNT];//DMA Double Buffer
void init_dmabuffer(void){
    for (int i =0;i < I2S_BUFFERSIZE;i++){
        dmabuffer[0][i*2] = 0;// 1st Buffer Lch
        dmabuffer[0][i*2+1] = 0;// 1nd Buffer Rch
        dmabuffer[1][i*2] = 0;// 2nd Buffer Lch
        dmabuffer[1][i*2+1] = 0;// 2nd Buffer Rch
    }
}

void HandlePdmData(uint16_t buffer[])
{
    for (int i =0;i < I2S_BUFFERSIZE;i++){
    // loop through this 256 times for 256 16 bit words
            uint16_t pdm = buffer[i*2];
#ifndef TWOSTEPCIC
            uint16_t m;
#endif

#ifdef PDMTEST
            pdm = testsamples[((++testcount)>>TESTSHIFT) & TESTMASK];
#endif

//            SONARSET;

#ifdef TWOSTEPCIC
            // First stage of PDM to PCM is to count the set bits in each of the
            // captured bytes, with each bit rescaled to a signed +/- 1 range.
            // This is the equivalent of an order-1 CIC filter with R=8, M=1, N=1.
            //
            // The bit growth of the output of this filter is then N*log2(R*M)
            // or 3, for 4 total significant bits out from the single bit in.
            // The numeric range at this stage is -8 to 8, so it can't fit in
            // a 4-bit 2's complement variable, but that is moot.
            //
            // The actual counting is done by lookup in the pdmsum8[] table.
            //
            // Now feed the 4 bit result to a second CIC with N=3, R=8, M=2
            // which has bit growth of 12, for a total of 16 significant bits
            // out. The counter scount is used to implement the decimation.
            s2_sum1 += pdmsum8[pdm&0xff] ;
            s2_sum2 += s2_sum1;
            s2_sum3 += s2_sum2;
            s2_sum1 += pdmsum8[pdm>>8] ;
            s2_sum2 += s2_sum1;
            s2_sum3 += s2_sum2;
#else
            // PDM bits arrive 16 at a time, MSB first. So we feed them one bit
            // at a time to the CIC filter with N=2, R=128, M=2 which has a bit
            // growth of 16, for a total of 17 significant bits out. The counter
            // scount implements the decimation, which must be by a multiple of
            // a power of two for the CIC and also be a multiple of 16 since the
            // SPI collects bits 16 at a time.
            for (m=0x8000; m; m>>=1) {
                s2_sum1 += (pdm&m) ? 1 : -1 ;
                s2_sum2 += s2_sum1;
            }
#endif

            if (!--s2_count) {
                CICREG stage3,stage2,stage1;
                CICREG Rout2 = s2_sum3;
//
                s2_count = CIC2_R/2;
//
                stage1 = Rout2 - s2_comb1_2;
                s2_comb1_2 = s2_comb1_1;
                s2_comb1_1 = Rout2;
//
                stage2 = stage1 - s2_comb2_2;
                s2_comb2_2 = s2_comb2_1;
                s2_comb2_1 = stage1;
//
                stage3 = stage2 - s2_comb3_2;
                s2_comb3_2 = s2_comb3_1;
                s2_comb3_1 = stage2;
                output=stage3+0x8FF;
                circbuffer.put((uint16_t)output);
//           SONARCLR;
            }
            }

#ifdef PULSEPIN
            // Also manage a pulse with width expressed in ticks at MCLK/16, which should
            // in the 16 us ballpark (assuming MCLK is about 1MHz).
#endif
}


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int
main(int argc, char* argv[])
{
  // Show the program parameters (passed via semihosting).
  // Output is via the semihosting output channel.
  trace_dump_args(argc, argv);

  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // Send a message to the standard output.
  puts("Standard output message.");
  init_dmabuffer();
  // Send a message to the standard error.
  fprintf(stderr, "Standard error message.\n");
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  trace_puts("HAL Init");
  /* Configure the system clock */
  SystemClock_Config();
  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);
  /* MCU Configuration----------------------------------------------------------*/

  /* Initialize all configured peripherals */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  trace_printf("GPIO Init\n");
  MX_DMA_Init();
  trace_printf("DMA Init\n");
  MX_USART2_UART_Init();
  trace_printf("UART\n");
  MX_ADC1_Init();
  trace_printf("ADC\n");
  MX_DAC_Init();
  trace_printf("DAC\n");
  MX_I2S_Init();
  trace_printf("I2S\n");


  // Perform all necessary initialisations for the LED.
  blinkLed.powerUp();
//
  Timer timer;
//  timer.start();
  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  TIMx_CLK_ENABLE();

  /*##-2- Configure the NVIC for TIMx ########################################*/
  /* Set the TIMx priority */
  HAL_NVIC_SetPriority(TIMx_IRQn, 0, 1);

  /* Enable the TIMx global Interrupt */
  HAL_NVIC_EnableIRQ(TIMx_IRQn);
  uwPrescalerValue = (uint32_t)((SystemCoreClock / 2) / 1800000) - 1;

    /* Set TIMx instance */
    TimHandle.Instance = TIMx;

    /* Initialize TIMx peripheral as follows:
         + Period = 10000 - 1
         + Prescaler = ((SystemCoreClock / 2)/10000) - 1
         + ClockDivision = 0
         + Counter direction = Up
    */
    TimHandle.Init.Period            = 40;
    TimHandle.Init.Prescaler         = uwPrescalerValue;
    TimHandle.Init.ClockDivision     = 0;
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
//    TimHandle.Init.RepetitionCounter = 0;

    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {

    }
 trace_puts("Start");
 hdma_spi2_rx.XferCpltCallback=dmaM0Complete;
 hdma_spi2_rx.XferM1CpltCallback=dmaM1Complete;
 hdma_spi2_rx.XferHalfCpltCallback=dmaHalfComplete;
 hdma_spi2_rx.XferErrorCallback=dmaError;

    //* DMA interrupt init */
        /* Sets the priority grouping field */


//      __HAL_DMA_ENABLE_IT(&hdma_spi2_rx,DMA_IT_TC);
 HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0xFFF);
 HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
 HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,0x000);
 HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
 if(HAL_DMAEx_MultiBufferStart_IT(&hdma_spi2_rx ,(uint32_t )(&SPI2->DR), (uint32_t)&dmabuffer[0][0] ,(uint32_t)&dmabuffer[1][0] ,DMA_TRANSFERCOUNT)!=HAL_OK) trace_printf("Error in HAL_DMAEx_MultiBufferStart_IT \n\r");
    trace_puts("MultiBufferStart");

    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;
//    __HAL_I2S_CLEAR_OVRFLAG(&hi2s2);
      __HAL_DMA_DISABLE_IT(&hdma_spi2_rx,DMA_IT_HT);
    /* Enable Rx DMA Request */
    SPI2->CR2 |= SPI_CR2_RXNEIE;
    SPI2->CR2 |= SPI_CR2_RXDMAEN;

//
/* Start Channel1 */
   if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
      {
        /* Starting Error */
//        Error_Handler();
      }


    while (1) {

    if (Buffer0_rdy) {
//          trace_printf("0\n");
          HandlePdmData(&dmabuffer[0][0]);
          Buffer0_rdy=0;
      }
      if (Buffer1_rdy) {
//          trace_printf("1\n");
          HandlePdmData(&dmabuffer[1][0]);
          Buffer1_rdy=0;
      }
//     trace_printf("DMA CR =%4x %4x %4x\n\r" ,DMA1_Stream3->CR ,DMA1->HISR,DMA1->LISR);


    }

  return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//
  // queue the finished PCM sample
//     blinkLed.turnOn();
     circbuffer.get(&dacoutput);
     HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dacoutput);
//
//                  trace_printf("out %d dac %d",output,dacoutput);
//
     HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
//     blinkLed.turnOff();
//
}

void dmaM0Complete(DMA_HandleTypeDef *hdma)
{
// toggle led1 at interrupt
//    mypin=0;
    Buffer0_rdy=1;
//    trace_printf("Xfer Cplt M0\n");
//    trace_printf("DMA CR =%4x %4x %4x\n\r" ,DMA1_Stream3->CR ,DMA1->HISR,DMA1->LISR);
}

void dmaHalfComplete(DMA_HandleTypeDef *hdma)
{
// toggle led1 at interrupt
//    mypin=0;
//    Buffer0_rdy=1;
    trace_printf("12\n");
//    trace_printf("DMA CR =%4x %4x %4x\n\r" ,DMA1_Stream3->CR ,DMA1->HISR,DMA1->LISR);
}

void dmaM1Complete(DMA_HandleTypeDef *hdma)
{
// toggle led1 at interrupt
//    mypin=1;
    Buffer1_rdy=1;
//    trace_printf("Xfer Cplt M1\n");
//    trace_printf("DMA CR =%4x %4x %4x\n\r" ,DMA1_Stream3->CR ,DMA1->HISR,DMA1->LISR);
}
void dmaError(DMA_HandleTypeDef *hdma)
{
// toggle led1 at interrupt
//    myled=0;
    trace_printf("0 DMA error %x\n",hdma->ErrorCode);
    trace_printf("1 DMA CR =%4x %4x %4x\n\r" ,DMA1_Stream3->CR ,DMA1->HISR,DMA1->LISR);
    trace_printf("DMA M0AR =%4x M1AR =%4x PAR = %4x \n\r"  ,DMA1_Stream3->M0AR  ,DMA1_Stream3->M1AR ,DMA1_Stream3->PAR);
    trace_printf("I2S CR2 = %4x \n\r" ,SPI2->CR2);
    trace_printf("I2S SR = %4x \n\r" ,SPI2->SR);
    trace_printf("I2S I2SCFGR = %4x \n\r" ,SPI2->I2SCFGR);
    trace_printf("I2S I2SPR = %4x \n\r" ,SPI2->I2SPR);
    trace_printf("RCC->PLLI2SCFGR = %4x \n\r" ,RCC->PLLI2SCFGR);
    while(1) {};
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 4;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* I2S2 init function */
void MX_I2S_Init(void)
{
  hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
    hi2s2.Init.Standard = I2S_STANDARD_LSB;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_192K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    HAL_I2S_Init(&hi2s2);
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
   __HAL_RCC_DMA1_CLK_ENABLE();

   /* DMA interrupt init */
   /* DMA1_Stream3_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);


}
/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
