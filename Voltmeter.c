
/* Схема подключения выводов GPIOA к сегментам
Выводы PORTA: 					 	  _ PA1 _ _ PA4 _ PA6 PA7 PA8 PA9 PA10 PA11       PA12       _   _         PA15
Выводы 7-мисегментного индикатора:       a       b     c   d   e   f   g    dp    катод INT_SEG           катод FRACT_SEG
*/

/* При подключении библиотеки нужно в настройках препроцессора прописать дефайн с нужным МК */
#include "stm32f4xx.h"  
/* Библиотека для вывода printf() по последовательному порту через USART2 */
#include <stdio.h> 		
/* Библиотеки LL для шин и переферии */
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"


/* Defines */
#define VDDA				3.3      // Напряжение питания аналоговой части
#define ADC_MAX_VAL         0x3FF	 // Максимальное значение ADC->DR при разрадности 10 бит
#define SamplesNumber       10       // Количество измерений (сэмплов) на одно итоговое значение на индикаторы
#define SamplingFreq        1600     // Константа определяет значение переполнения Timer3 и собственно частоту сэмплирования (16МГц/1600=10кГц)
#define ENABLE_SEG(SEG)     ((GPIOA->ODR) |= (SEG))
#define DISABLE_SEG(SEG) 	((GPIOA->ODR) &= ~(SEG))

/* Initialize Serial interface */
extern int stdout_init (void);

/* Initialization functions */
void SystemClockInit(void);  
void Timer3Init(void);
void GPIOAInit(void);
void DMA2Init(uint32_t SrcAdr, uint32_t DstAdr);
void ADC1INit(void);

/* Interrupts */
void DMA2_Stream0_IRQHandler(void);


/* Массив кода каждой цифры сегмента. Важно! Используется PORTA, некоторые выводы которого уже заняты, 
поэтому первый диод "а" сегмента не обязательно будет на нулевом выводе порта и так далее.
Также нужно заметить, что на сегменте целого числа должна гореть точка, для этого можно просто добавлять константу к значению 
кода цифры из массива */
uint16_t SegmentValCode[10]= {
	0b001111010010,		// 0
	0b000001010000, 	// 1
	0b010110010010,		// 2
	0b010011010010,		// 3
	0b011001010000,		// 4
	0b011011000010,		// 5
	0b011111000010,		// 6
	0b000001010010,		// 7
	0b011111010010,		// 8
	0b011011010010		// 9
};

enum Segment { INT_SEG = LL_GPIO_PIN_12, FRACT_SEG =  LL_GPIO_PIN_15 };    

struct Voltmeter {
	uint16_t averageMeasurement;			// Среднее значение ADC1->DR из SamplesNumber измерений
	uint16_t integerVal;      				// Целая часть от измеренного ADC значения
	uint16_t fractionalVal;  		    	// Дробная часть от измеренного ADC значения
	enum Segment EnableSeg;		       	 	// Сегмент, который в данный момент включён
} VM;

uint16_t delay=500;                     	// Переменная определяет задержку после включения ADC1

uint16_t measureADC[SamplesNumber]={0,};	// В массиве хранятся измерения ADC1, из которых получаем среднее значение



void DMA2_Stream0_IRQHandler(void)
{
	uint16_t sum=0;
	uint16_t i;
	
	/* Если все пункты обработчика не выполнятся до слеующего измерения и запроса ADC1 к переносу нового значения в массив, то
	либо в крайнем случае, при выполнении обработчика наступит новое прерывание, в данном случае лучше отключить в начале
	обработчика DMA, несколько значений ADC1 потеряются, но точность в данном проекте не столь важна */
	//LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_0);

	for( i=0; i < SamplesNumber; i++)
	{
		sum+=measureADC[i];
	};

	VM.averageMeasurement = (uint16_t)(sum/SamplesNumber);
	VM.EnableSeg=( VM.EnableSeg == INT_SEG ) ? FRACT_SEG : INT_SEG;   // Переключение сегментов

	LL_DMA_ClearFlag_TC0(DMA2);
};

/* General functions */
void NumberSeparation(struct Voltmeter *VMeter)
{
	float temp=0;

	temp=(float)VMeter->averageMeasurement;
	temp=(float)( VDDA * 10 * ( temp / ADC_MAX_VAL) );     // Умножаю на 10 для дальнейшего извлечения дробной и целой частей
	VMeter->integerVal=(uint16_t)( temp / 10);             // Получаем целую часть числа
	// Оператор % работает только с целочисленными типами, поэтому нужно сначала привести temp к uint16_t, затем выполнять операцию
	VMeter->fractionalVal=((uint16_t)temp) % 10;		   // Получаем дробную часть числа
};

int main(void) {

	VM.averageMeasurement=0;
	VM.integerVal=0;
	VM.fractionalVal=0;
	VM.EnableSeg=INT_SEG;
	
	SystemClockInit();
	Timer3Init();
	//Сначала нужно инициализировать DMA и GPIOA, затем включать ADC, чтобы не пропустить показания ADC(выключать в таком же порядке!)
	DMA2Init((uint32_t) &(ADC1->DR), (uint32_t) measureADC);
	GPIOAInit();
	ADC1INit();

	/* Настройка прерывания */
	__enable_irq (); 						// Глобальное разрешение прерываний
	 NVIC_EnableIRQ(DMA2_Stream0_IRQn);		// Разрешаем прерывание по DMA2
	
	stdout_init();                          /* Initialize Serial interface */

	while(1){
		NumberSeparation(&VM);
		//printf ("Hello World\n\r");
		if ( VM.EnableSeg == INT_SEG)
		  {
			ENABLE_SEG(INT_SEG);
			DISABLE_SEG(FRACT_SEG);
			//GPIOA->ODR &= (uint16_t)( 0xF02D | ( SegmentValCode[VM.integerVal] | ((uint16_t)(1<<12)) ));
			// Перед установкой нового кода цифры, необходимо сбросить предыдущее значение маской 0b111111010010
			MODIFY_REG( GPIOA->ODR, 0b111111010010, ( SegmentValCode[VM.integerVal] | ((uint16_t)(1<<11))) );
		  }
		else if( VM.EnableSeg == FRACT_SEG)
		  {
			ENABLE_SEG(FRACT_SEG);
			DISABLE_SEG(INT_SEG);
			//GPIOA->ODR &= (uint16_t)( 0xF02D | SegmentValCode[VM.fractionalVal] );
			// Перед установкой нового кода цифры, необходимо сбросить предыдущее значение маской 0b111111010010
			MODIFY_REG( GPIOA->ODR, 0b111111010010, SegmentValCode[VM.fractionalVal] );
		  }
	}
		
}




void SystemClockInit(void)
{
	/* Сбросим RCC clock configuration в начальное состояние:  
		- HSI ON (16MHz) and used as system clock source, PLL OFF
		- AHB, APB1 and APB2 prescaler set to 1.
		- CSS, MCO OFF
		- All interrupts disabled */
	LL_RCC_DeInit();

	/*  Настраивать Wait states (WS) (LATENCY) из-за медленной работы Flash memory по отношению к CPU
	здесь излишне, частота HSI входит в допустимый диапазон при 0 WS (стр. 66 RM0390)*/
	//LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	
	/*Each time the core clock (HCLK) changes, this function must be called
	to update SystemCoreClock variable value. Otherwise, any configuration
	based on this variable will be incorrect.*/
	SystemCoreClockUpdate();	
	
	//НАстроили тактирование системных шин, теперь необходиво включить тактирование используемых переферийных модулей
	
	/*( ВАЖНО! в errata с.12 написано, что после включения тактирования переферии необходимо подождать несколько циклов )
	в LL это уже предусмотренно, там есть дополнительная операция чтения регистра */

	/* AHB1: DMA2 Clocking and GPIOA Clocking */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	/* APB1 Timer Clocking : TIM3, TIM6 */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
	/* APB2 Peripheral Clocking : ADC */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
}

void Timer3Init(void)
{
	/* Настройка TIM3 CH1, RM0390 стр.541, 
	Некоторые функции здесь излишни, так как устанавливают параметры по умолчанию, но они полезны для понимания настройки таймера */
	LL_TIM_DeInit(TIM3);  	// Сбрасываем таймер в начальное состояние
	LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL); 	// Источник тактирования таймера внутренний (по умолчанию)
	LL_TIM_SetPrescaler(TIM3, 0); 	// Предделитель не используется (по умолчанию)
	LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP); 	// Режим счёта таймера вверх (по умолчанию)
	LL_TIM_SetAutoReload(TIM3, SamplingFreq); 	 // Значение регистра ARR, указывает значение переполнения счётчика
	LL_TIM_OC_SetCompareCH1(TIM3, 0);	 // Событие Compare будет происходить, после каждого сброса счётчика
	LL_TIM_OC_SetMode(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_TOGGLE); 	// Настраиваем выход OC1REF на переключение состояния
	LL_TIM_OC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH); // Задаём начальльное состояние выхода OC1REF (по умолчанию)
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1); 	// Включаем канал TIM3_CH1 (сигнал подаётся на TIM3_CH1, но не на вывод МК)
	LL_TIM_EnableCounter(TIM3);	 // Включаем Timer3
}

void ADC1INit(void)
{
	LL_ADC_DeInit(ADC1); 	// Cбрасываем ADC1 в начальное состояние
	LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM3_CH1); // Сигнал начала преобразования от внешнего источника 
	LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISINGFALLING); // Преобразование происходит и по переднему, и по заднему фронтам
	LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_10B); 	//Разрешение ADC1 10бит (больше в условиях поставленной задачи не нужно)
	// Запросы к DMA2 отправляются постоянно после преобразования, этот режим работает вместе с Circular режимом DMA2
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED); 	
	LL_ADC_Enable(ADC1); 	// Включаем ADC1
	while (delay--){}; 		// После включения необходима небольшая задержка (errata sheet)
}

void DMA2Init(uint32_t SrcAdr, uint32_t DstAdr)
{
	LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_0, LL_DMA_CHANNEL_0);  // Выбираем канал CH1 потока STREAM0 4
	// Устанавливаем напраление передачи данных, адрес источника и адрес места назначения
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_0, SrcAdr, DstAdr, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	// Устанавливаем количество передач данных (data transfers) в одной транзакции (DMA transaction) (RM0390 c.206)
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_0, SamplesNumber); 
	// Инкрементируем адрес пункта назначения данных в памяти (инкрементируется number of data transfers раз) 
	LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT); 
	// Размер передаваемого значения, передаем полслова(16 бит) 
	LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD); 
	// Адрес источника не инкрементируем, так как данные берутся из одного регистра ADC1->DR
	LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);
	// Устанавливаем режим работы DMA в Curcular Mode, в данном режиме после окончания транзакции, новая начинается автоматически
	LL_DMA_SetMode(DMA2, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);	
	LL_DMA_ClearFlag_TC0(DMA2);	   // Очищаем флаг прерывания по завершению передачи данных
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);    // Разрешаем прерывание по завершению транзакции
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);   // Включаем DMA
}

void GPIOAInit(void){
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);	// Напряжение измеряется на PA0, устанавливаем его в  Analog mode
	// Выводы подключённые к сегментам устанавливаем в режим Output, I/O output speed - Medium speed
	// Данный порт управляет как анодомами, так и катодами сегментов
	GPIOA->MODER &= ~0x80000000;    // Начальное значение 0xA800 0000 for port A, сл-но MODER15[1:0]=0b10, поэтому нужно сбросить 
	GPIOA->MODER |= 0b01000001010101010101000100000100;
	GPIOA->OSPEEDR |= 0b01000001010101010101000100000100;
};