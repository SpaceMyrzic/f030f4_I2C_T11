#include <stm32f0xx.h>


#define I2C_Displ_ADDRES					0x3C				// адрес дисплея по шине I2C

#define FLASH_KEY1			((uint32_t)0x45670123)				// 1 ключь для разблокировки Flash
#define FLASH_KEY2			((uint32_t)0xCDEF89AB)				// 2 ключь для разблокировки Flash

#define ADDRES_FLASH					0x8004000				// сам адрес памяти в виде данных в переменной
#define ADDRES_FLASH_DATA	(*(uint8_t*)0x8004000)				// такой формой записи мы можем указать адрес памяти переменной. при uint8_t указывается на первую ячейку памяти

/* флаги */
_Bool Flag_Pusk;    											// флаг запуск сварки
//_Bool Flag_Plus;     											// флаг увеличения
//_Bool Flag_Minus;     										// флаг уменьшения
_Bool Flag_Korotko;    											// флаг короткого нажатия
_Bool Flag_Dolgo;    											// флаг долгого нажатия

/* переменные */
__IO uint8_t Flag_Zero;  										// переход через ноль
__IO uint16_t Time;  											// время 
__IO uint8_t Punkt_Menu;										// пункт меню
__IO uint8_t Flag_Plus;     									// флаг увеличения
__IO uint8_t Flag_Minus;     									// флаг уменьшения

// выбор с какими данными будем работать
typedef enum Type_Data
{
	Power_1 = 0,
	Time_Power_1 = 1,
	Pauza = 2,
	Power_2 = 3,
	Time_Power_2 = 4,
	All_data = 5
}Type_Data;

// выбор режимов дисплея: команды, данные по однаму, даные сплошником.
typedef enum ssd1360_control_bit
{
	COM = 0x80,   													// команды
	DAT = 0xC0,   													// данные по однаму
	DATS = 0x40														// данные непрерывно
}control_bit;

// выбор режим записи или чтения для I2C
typedef enum i2c_write_read
{
	Write,
	Read
}i2c_naprovlenie;

const uint8_t Svarka_data_flash[5] = {82,2,2,65,2}; 			// шаблонные данные для копирования в память при первом включении

uint8_t Svarka_data[5] = {};									// массив данных для работы сварки

uint8_t Set_Shablon[] = { 0x21, 0, 15, 0x22, 1, 5 };			// установка кординат шаблона

uint8_t Set_Data[] = {0x21, 23, 38, 0x22, 1, 1};				// установка кординат данных

uint8_t Set_Kyrsor[] = { 0x21, 42, 49, 0x22, 1, 1 };			// установка кординат курсора

uint8_t Cler_dis[] = { 0 };										// переменная для очистки дисплея

// инициализация дисплея
uint8_t Init_dis[] = {
	0xAE,		// Выключить дисплей
	0xD5,		// Настройка частоты обновления дисплея
	0x80,	
	///+----- делитель 0-F/ 0 - деление на 1
	//+------ частота генератора. по умочанию 0x80
	0xA8,		// Установить multiplex ratio
	0x3F,		// 1/64 duty (значение по умолчанию), 0x1F - 128x32, 0x3F - 128x64
	0xD3,		// Смещение дисплея (offset)
	0x00,		// Нет смещения
	0x40,		// Начала строки начала разверки 0x40 с начала RAM
	0x8D,		// Управление внутреним преобразователем
	0x14,		// 0x10 - отключить (VCC подается извне) 0x14 - запустить внутрений DC/DC
	0x20,		// Режим автоматической адресации
	0x00,		// 0-по горизонтали с переходом на новую страницу (строку)
				// 1 - по вертикали с переходом на новую строку
				// 2 - только по выбранной странице без перехода
	0xA0,		// Режим разверки по странице (по X)
				// A1 - нормальный режим (слева/направо) A0 - обратный (справа/налево)
	0xC0,		// Режим сканирования озу дисплея
	            // для изменения системы координат
	            // С0 - снизу/верх (начало нижний левый угол)
	            // С8 - сверху/вниз (начало верний левый угол)
	0xDA,		// Аппаратная конфигурация COM
	0x12,		// 0x02 - 128x32, 0x12 - 128x64
	0x81,		// Установка яркости дисплея
	0x8F,		// 0x8F..0xCF
	0xD9,		// Настройка фаз DC/DC преоразователя
	0xF1,		// 0x22 - VCC подается извне / 0xF1 для внутренего
	0xDB,		// Установка уровня VcomH
	0x40,		// Влияет на яркость дисплея 0x00..0x70
	0xA4,		// Режим нормальный
	0xA6,		// 0xA6 - нет инверсии, 0xA7 - инверсия дисплея
	0xAF		// Дисплей включен
};

// курсор указывающий что изменяют
uint8_t Kursor[8] = { 0x18, 0x3C, 0x7E, 0x18, 0x18, 0x18, 0x18, 0x00 };

// Шрифт цифр
uint8_t Cifry[10][8] = {
	{ 0x3E, 0x7F, 0x71, 0x59, 0x4D, 0x7F, 0x3E, 0x00 },	 			// 0
	{ 0x00, 0x02, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00 },	 			// 1
	{ 0x62, 0x73, 0x59, 0x49, 0x4F, 0x46, 0x00, 0x00 },	 			// 2
	{ 0x22, 0x63, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00 },				// 3
	{ 0x18, 0x1C, 0x16, 0x13, 0x7F, 0x7F, 0x10, 0x00 },	 			// 4
	{ 0x27, 0x67, 0x45, 0x45, 0x7D, 0x39, 0x00, 0x00 },				// 5
	{ 0x3C, 0x7E, 0x4B, 0x49, 0x79, 0x30, 0x00, 0x00 },				// 6
	{ 0x01, 0x01, 0x71, 0x79, 0x0F, 0x07, 0x00, 0x00 },	 			// 7
	{ 0x36, 0x7F, 0x49, 0x49, 0x7F, 0x36, 0x00, 0x00 },	 			// 8
	{ 0x06, 0x4F, 0x49, 0x69, 0x3F, 0x1E, 0x00, 0x00 }				// 9
};

// Шрифт текста
uint8_t Shablon[10][8] = { 
	{ 0x00, 0x7F, 0x7F, 0x30, 0x18, 0x30, 0x7F, 0x7F },  			// W
	{ 0x00, 0x7C, 0x7C, 0x04, 0x04, 0x7C, 0x78, 0x00 },  			// n
	{ 0x00, 0x00, 0x01, 0x01, 0x7F, 0x7F, 0x01, 0x01 },	  			// T
	{ 0x00, 0x7C, 0x7C, 0x04, 0x04, 0x7C, 0x78, 0x00 },	  			// n
	{ 0x00, 0x7F, 0x7F, 0x09, 0x09, 0x0F, 0x06, 0x00 },	 			// P
	{												 },	 			// 
	{ 0x00, 0x7F, 0x7F, 0x30, 0x18, 0x30, 0x7F, 0x7F },	 			// W
	{ 0x00, 0x48, 0x5C, 0x54, 0x54, 0x74, 0x24, 0x00 },	 			// s
	{ 0x00, 0x00, 0x01, 0x01, 0x7F, 0x7F, 0x01, 0x01 },	 			// T
	{ 0x00, 0x48, 0x5C, 0x54, 0x54, 0x74, 0x24, 0x00 },	 			// s
};


/* настройка тактирования */
void RCC_Init(void)
{
	/* выбор источника тактирования */
	
	/* тактирование от внешнего генератора */
	RCC->CR |= RCC_CR_HSEON;        							// выбираем источник тактирование внешний кварц (HSE)
	while(!(RCC->CR & RCC_CR_HSERDY));        					// ждём пока флаг готовности внешнего тактирования не скажет что все норм (6 тактов занимает)
	
	RCC->CFGR &= (uint32_t)0x08FFB80C;       					// сброс SYSCLK чтоб была возможность сконфигурировать PLL
		
	/* настройка системной частоты SYSCLK */
	
	/* настройка претдилителей для шин AHB и APB1 */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;       						// делитель AHB равен /1
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;       						// делитель APB1 равен /1
	
	RCC->CFGR |= RCC_CFGR_SW_HSE;      							// выбор в качестве источника HSE для системной шины SYSCLK
	while(!(RCC->CFGR & RCC_CFGR_SWS_HSE));        				// ждём когда источник HSE для SYSCLK выбирется	
}

/* настройка портов */
void Set_Gpio(void)
{
	/* настройка портов */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    						// включение тактирования порта A
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;    						// включение тактирования порта B
	
	GPIOA->MODER &= ~GPIO_MODER_MODER0_0;    					// режим вход PA0
	
	GPIOA->MODER &= ~GPIO_MODER_MODER1_0;    					// режим вход PA1
	
	GPIOA->MODER &= ~GPIO_MODER_MODER2_0;    					// режим вход PA2
	
	GPIOA->MODER &= ~GPIO_MODER_MODER3_0;    					// режим вход PA3
	
	GPIOA->MODER &= ~GPIO_MODER_MODER4_0;    					// режим вход PA4
	
	GPIOB->MODER |= GPIO_MODER_MODER1_0;    					// режим выход PB1
	GPIOB->OTYPER |= GPIO_OTYPER_OT_1;    						// режим выхода OD
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR1_0;    				// скорость редняя
	GPIOB->BSRR |= GPIO_BSRR_BS_1;       						// включаем порт
}

/* настройка внешних прерываний */
void Set_Exti(void)
{
	/* настройка EXTI */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    						// Включили тактирование EXTI
	SYSCFG->EXTICR[0] = 0;    										// мултиплексы EXTI 0-3 выставлен на порт A
	EXTI->IMR |= 0xC;    											// разрешить прерывание по EXTI 2-3	по EXTI 0-1 запрещён
	EXTI->FTSR |= 0xF;    											// прерывание по спадающему фронту
	EXTI->RTSR |= 0x8;												// прерывания по возрастающиму фронту для кнопки
	NVIC->ISER[0] |= 0x60;    										// разрешаем прерывание по векторам прерывания EXTI0-1 и EXTI2-3
}

/* настройка системного таймера Systick */
void Set_Systick(void)
{
	/* настройка таймера Systick */
	SysTick->LOAD = 799;    										// установка величины которую бдет считать счётчик (0,1мкс)
	SysTick->VAL = 0;    											// сброс счётчика в ноль
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;    				// брать частоту процессора
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;    					// разрешить работу прерывания таймера
}

/* инициализация I2C */
void i2c_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;     					// тактирование I2C1
	RCC->CFGR3 |= RCC_CFGR3_I2C1SW;     						// тактирование I2C1 от SYSCLK
	
	
	GPIOA->MODER |= GPIO_MODER_MODER9_1;     					// альтернативные функции AF
	GPIOA->OTYPER |= GPIO_OTYPER_OT_9;     						// опендрей OD
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0;     			// средняя скорость m_speed
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0;     					// подтяжку к питания PU
	
	
	GPIOA->MODER |= GPIO_MODER_MODER10_1;     					// альтернативные функции AF
	GPIOA->OTYPER |= GPIO_OTYPER_OT_10;     					// опендрей OD
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_0;     			// средняя скорость m_seed
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR10_0;     					// подтяжка к питанию PU
	
	GPIOA->AFR[1] |= 0x440;     								// включение альтернативных функций для порта А9 и А10
	
	//I2C1->TIMINGR = 0x2000090E;          						// настраивается тайминги для 8 MGh 100 KHz
	I2C1->TIMINGR = 0x0000020B;									// настраивается тайминги для 8 MGh 400 KHz
  	I2C1->CR1 &= ~(I2C_CR1_ANFOFF & I2C_CR1_DFN);     			// включение аналогово филтра и выключение цифрового филтра
	I2C1->CR1 |= I2C_CR1_PE;     								// включение переферии
}

/* вектор прерывания для EXTI0 и EXTI1 */
void EXTI0_1_IRQHandler(void)
{
	if (EXTI->PR & (1 << 0))								// Прерывание от EXTI0
		{
			EXTI->PR |= (1 << 0);   						// Сбросить флаг EXTI0 
		    Flag_Zero++;   									// флаг переход через ноль
		}
	if (EXTI->PR & (1 << 1))								// Прерывание от EXTI1
		{
			EXTI->PR |= (1 << 1);   						// Сбросить флаг EXTI1  
		    Flag_Pusk = 1;   								// флаг запуска сварки
			Flag_Zero = 0;   								// обнуляем флаг переход через ноль	
		} 

}

/* вектор прерывания для EXTI2 и EXTI3 энкодер */
void EXTI2_3_IRQHandler(void)
{
	if (Flag_Pusk){EXTI->PR |= ((1 << 2)&(1 << 3));}				//  если сварка начато то управление отключаем
	
	if (EXTI->PR & (1 << 2))										// Прерывание от EXTI2
		{
			EXTI->PR |= (1 << 2);   								// Сбросить флаг EXTI2  
			
			if(!(GPIOA->IDR & GPIO_IDR_4))							// если прерывание энкодера было позже то увеличиваем
			{
				if (Flag_Korotko){Flag_Plus ++; }					// еслм нажата кнопка коротко то меняем значения +
				//if(Flag_Korotko){Flag_Plus = 1; }					// еслм нажата кнопка коротко то меняем значения +
				if ((Flag_Dolgo) & (!Flag_Korotko))					// 
				{
					Punkt_Menu++; 									// увеличиваем значения пунктов меню
					if(Punkt_Menu == 5) {Punkt_Menu = 0; }			// ограничение по пунктам меню
				}
			}
			else
			{
				if (Flag_Korotko){Flag_Minus ++; }					// еслм нажата кнопка коротко то меняем значения -
				//if(Flag_Korotko){Flag_Minus = 1; }					// еслм нажата кнопка коротко то меняем значения -
				if ((Flag_Dolgo) & (!Flag_Korotko))					// 
				{
					Punkt_Menu--;									// уменьшаем значение пунктов меню
					if (Punkt_Menu == 255) {Punkt_Menu = 4; }		// ограничение по пунктам меню
				}
				
			}
		}
	if (EXTI->PR & (1 << 3))										// Прерывание от EXTI3
		{
			EXTI->PR |= (1 << 3);   								// Сбросить флаг EXTI3  
			
		    if(!(GPIOA->IDR & GPIO_IDR_3))							// если кнопка нажата (переход из высокого в ниское)
			{
				if(Flag_Dolgo){Flag_Korotko = !Flag_Korotko;}		// меняем значении короткого нажатия только когда произащло долгое нажатие
				SysTick->VAL = 0;       							// сброс счётчика в ноль
				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;			// включаем таймер		 
			}
			else													// если кнопка отпущина (переход из низкого в высокий)
			{
				if (Time >= 3000)									// если таймер дощитал нужное время
				{
					SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     	// выключаем таймер
					Time = 0;										// сброс счётчика
					Flag_Korotko = 0;
					Flag_Dolgo = !Flag_Dolgo;						// установить флаг о долгом удержании кнопки
					if(Flag_Dolgo) 
					{
						EXTI->IMR &= ~0x3;							// запретить прерывания по EXTI 0-1
					}
					else 
					{
						EXTI->IMR |= 0x3;							// разрешаем прерывание по EXTI 0-1
					}

				}
				else												// таймер не дощитал
				{
					SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      // выключаем таймер
					Time = 0;										// сбросить счётчик
				}
			}
		} 
}

/* системный таймер */
void SysTick_Handler(void)
{
	Time++;
}

/* разблокировак флеш памяти */
void FLASH_Unlock(void)
{
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}	

/* блокировак флеш памяти */
void FLASH_Lock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;
}

/* флеш свободна и можно с ней работать */
_Bool flash_ready(void) 
{
	return !(FLASH->SR & FLASH_SR_BSY);
}

/* стирание или запись завершена */
_Bool check_EOP(void)
{
	if (FLASH->SR & FLASH_SR_EOP)
	{	
		FLASH->SR |= FLASH_SR_EOP;
		return 1;
	}	
	return 0;
}	

/* очистка страници */
_Bool FLASH_Erase_Page(uint32_t address) 
{
	while (!flash_ready()) ;										//Ожидаем готовности флеша к записи
	
	FLASH->CR |= FLASH_CR_PER;										//Устанавливаем бит стирания одной страницы
	FLASH->AR = address;											// Задаем её адрес
	FLASH->CR |= FLASH_CR_STRT;										// Запускаем стирание
	while(!flash_ready());											//Ждем пока страница сотрется
	FLASH->CR &= ~FLASH_CR_PER;										//Сбрасываем бит стирания одной страницы
	
	return check_EOP();												//операция завершена, очищаем флаг
}

/* запись данных в память */
_Bool FLASH_Program_Word(uint32_t address, uint32_t data)
{
	while (!flash_ready()) ;										//Ожидаем готовности флеша к записи
	
	FLASH->CR |= FLASH_CR_PG;										//Разрешаем программирование флеша
	*(__IO uint16_t*)address = (uint16_t)data;						//Пишем младшие 2 байта
	while(!flash_ready());											//Ждем завершения операции
	if (!check_EOP())return 0;
	
	address += 2;													//Прибавляем к адресу два байта
	data >>= 16;													//Сдвигаем данные
	*(__IO uint16_t*)address = (uint16_t)data;						//Пишем старшие 2 байта
	while(!flash_ready());											//Ждем завершения операции
	FLASH->CR &= ~(FLASH_CR_PG);									//Запрещаем программирование флеша
	
	return check_EOP();
}

/* копируем данные из Flash в Ram память */
void Copi_Flash_Ram(uint8_t *addres) 
{	
	for (uint8_t i = 0; i < 5; i++)
	{
		Svarka_data[i] = *(addres++);								// мы берем из адреса 1 байт даных и постепенно увеличивая адрес заполняем массив
	}
}

/* запись данных из Ram в Flash память */
void Write_Flash_Ram(void)
{
	uint32_t data_1, data_2;
	data_1 = (*(__IO uint32_t*) Svarka_data);
	data_2 = Svarka_data[4];
	FLASH_Unlock();  												// разблокировка памяти
	FLASH_Erase_Page(ADDRES_FLASH);  								// очистка страници памяти по указаному адрессу
	FLASH_Program_Word(ADDRES_FLASH, data_1);   					// запись в память 4 байт
	FLASH_Program_Word((ADDRES_FLASH + 4), data_2);   				// запись в память 4 байт
	FLASH_Lock(); 	
	
//	test1 = (*(__IO uint32_t*) Svarka_data);
//	test3 = Svarka_data[4];
//	FLASH_Unlock(); 												// разблокировка памяти
//	FLASH_Erase_Page(ADDRES_FLASH); 								// очистка страници памяти по указаному адрессу
//	FLASH_Program_Word(ADDRES_FLASH, test1); 						// запись в память 4 байт
//	FLASH_Program_Word((ADDRES_FLASH + 4), test3); 					// запись в память 4 байт
//	FLASH_Lock(); 													// блокировка памяти
}

/* задержка времени */
void Delay()
{
	int i;
	for (i = 0; i < 1000000; i++)
		asm("nop");
}

void i2c_Stop(void)
{
	// остановка
	I2C1->CR2 |= I2C_CR2_STOP;        								// Выдать стоп на шину
	while(I2C1->ISR & I2C_ISR_BUSY);        						// Ожидать выдачу стопа
	// Очищаю флаги - необходимо для дальнейшей работы шины
	I2C1->ICR |= I2C_ICR_STOPCF;        							// STOP флаг
	I2C1->ICR |= I2C_ICR_NACKCF;        							// NACK флаг
	// Если есть ошибки на шине - очищаю флаги
	if(I2C1->ISR & (I2C_ISR_ARLO | I2C_ISR_BERR))
	{
		I2C1->ICR |= I2C_ICR_ARLOCF;
		I2C1->ICR |= I2C_ICR_BERRCF;
	}
}

void i2c_Start(i2c_naprovlenie wr_re, uint8_t addres, uint8_t size)
{
	if (wr_re) I2C1->CR2 |= I2C_CR2_RD_WRN;      					// Режим ппередачи
	else I2C1->CR2 &= ~I2C_CR2_RD_WRN;     							// Режим приём
	I2C1->CR2 &= ~I2C_CR2_NBYTES;           						// Очистить размер данных
	I2C1->CR2 |= size << 16;            							// Установить размер данных
	I2C1->CR2 &= ~I2C_CR2_SADD;           							// Очистить адрес ведомого устройства
	I2C1->CR2 |= addres << 1;                						// Установить адрес ведомого устройства
	I2C1->CR2 |= I2C_CR2_START;           							// Выдать старт на шину
	while((I2C1->ISR & I2C_ISR_BUSY) == 0);          				// Ожидать выдачу старта
}

/* отправка данных на дисплей */
void Disp_Write(uint8_t addres, control_bit registr, uint8_t *data, uint16_t size)
{
	
	if (registr == DATS)												// режим отправки данных непрерывно (DATS)
		{
			uint8_t k = 0;  											// контрольный флаг для записи одинаковых значений
			//if(size == 1024) k = 1;   									// если необходима заполнить весь дисплей одними значениями
			if(data == Cler_dis) k = 1;      							// если необходима чтото в дисплее стереть
		
			while(size >= 128)											// при большом количестве данных делим отправку на страници по 128 байт
			{
				size = size - 128;
				i2c_Start(Write, addres, 128 + 1);    								// старт. данные + контрольный байт только в начале
				// ждём пока можно будет оправить данные
				while((((I2C1->ISR & I2C_ISR_TXIS) == 0)  && ((I2C1->ISR & I2C_ISR_NACKF) == 0)) && (I2C1->ISR & I2C_ISR_BUSY));
				if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = registr;    				// Отправляю контрольный байт
				// передаём байты до тех пор, пока не взлетит TC-флаг.
				// Если взлетит NACK-флаг, приём прекращаем.
				while((((I2C1->ISR & I2C_ISR_TC) == 0) && ((I2C1->ISR & I2C_ISR_NACKF) == 0)) && (I2C1->ISR & I2C_ISR_BUSY))
				{
					if (k == 1)	{if (I2C1->ISR & I2C_ISR_TXIS)  I2C1->TXDR =  *data; }   		// отправляю данные без изменения адреса масива данных
					else		{if (I2C1->ISR & I2C_ISR_TXIS)  I2C1->TXDR =  *(data++); }   	// отправляю данные	с изменением адреса массима данных			
				}
			}
		
			if (size)																	// проверка на то что данные есть
				{
					i2c_Start(Write, addres, size + 1);      							// старт. данные + контрольный байт только в начале
					// ждём пока можно будет оправить данные
					while((((I2C1->ISR & I2C_ISR_TXIS) == 0)  && ((I2C1->ISR & I2C_ISR_NACKF) == 0)) && (I2C1->ISR & I2C_ISR_BUSY));
					if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = registr;    				// Отправляю контрольный байт
					// передаём байты до тех пор, пока не взлетит TC-флаг.
					// Если взлетит NACK-флаг, приём прекращаем.
					while((((I2C1->ISR & I2C_ISR_TC) == 0) && ((I2C1->ISR & I2C_ISR_NACKF) == 0)) && (I2C1->ISR & I2C_ISR_BUSY))
					{
						if (k == 1)	{if (I2C1->ISR & I2C_ISR_TXIS)  I2C1->TXDR =  *data; }   		// отправляю данные без изменения адреса масива данных
						else		{if (I2C1->ISR & I2C_ISR_TXIS)  I2C1->TXDR =  *(data++); }   	// отправляю данные	с изменением адреса массима данных	
					}
				}
		
		}
	else																// режим отправки команд/данных по однаму (COM, DAT)
		{
			i2c_Start(Write, addres, size * 2);   								// старт. данные *2 так как перед каждыми данными идёт контрольный байт
		
			while((((I2C1->ISR & I2C_ISR_TC) == 0)  && ((I2C1->ISR & I2C_ISR_NACKF) == 0)) && (I2C1->ISR & I2C_ISR_BUSY))
			{
				while (size)
				{
					// ждём когда можно будет оправить данные
					while((((I2C1->ISR & I2C_ISR_TXIS) == 0) && ((I2C1->ISR & I2C_ISR_NACKF) == 0)) && (I2C1->ISR & I2C_ISR_BUSY));
					if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR = registr;   		// отправляем контрольный байт
	
					// Принимаем байты до тех пор, пока не взлетит TC-флаг.
					// Если взлетит NACK-флаг, приём прекращаем.
					while((((I2C1->ISR & I2C_ISR_TXIS) == 0) && ((I2C1->ISR & I2C_ISR_NACKF) == 0)) && (I2C1->ISR & I2C_ISR_BUSY));
					if (I2C1->ISR & I2C_ISR_TXIS) I2C1->TXDR =  *(data++);   	// отправляем данные    
					size--;    													// уменьшаем счётчик
				}
			}
		
		}
	i2c_Stop();
}

/* вывод на дисплей значений по однаму или всех разом */
void Disp_Write_Data(Type_Data data)
{
	uint8_t p = 1; 																// повторение цикла 1 раз
	switch(data)																// в зависимости от типа данных и количества выбирается кординаты выводимых данных 
	{
		case 5:																	// данная адресация вебирается только когда необходимо отправить все данные
			Set_Data[4] = 1;     												// начало страници 1
			Set_Data[5] = 5;     												// конец страници 5
			p = 5;   															// повторение цикла 5 раз
			break;		
		default:																// так как мы знаем какие данные унас пришли то мы можем их подогнать под адрес
			Set_Data[4] = data + 1; 											// наши данные меньше чем адрес по этому увеличиваем на 1
			Set_Data[5] = data + 1; 											// наши данные меньше чем адрес по этому увеличиваем на 1
			break;
	}
	Disp_Write(I2C_Displ_ADDRES, COM, Set_Data, 6);   							// установка кординат для вывода данных
	
	if(data == All_data) {data = 0; }											// если надо вывести все значения то начинаем с начала массива данных
	for (; p > 0; p--)
	{
		uint8_t data_1 = Svarka_data[data];
		uint8_t data_10 = 0;
		if ((data == Power_1)||(data == Power_2)){data_1 = 100 - data_1; }		// если принятые данные являются Power_1 или Power_2 то их необходимо преобразовать
		while(data_1 >= 10)														// пересчёт значений на единици и десятые
		{
			data_1 = data_1 - 10;
			data_10++;
		}
		if (data_10){Disp_Write(I2C_Displ_ADDRES, DATS, Cifry[data_10], 8); }	// если есть десятые то выводим десятые
		else{Disp_Write(I2C_Displ_ADDRES, DATS, Shablon[5], 8); }				// вывод пробела
		Disp_Write(I2C_Displ_ADDRES, DATS, Cifry[data_1], 8);   				// вывод единиц
		data++;   																// увеличиваем начение для перехода по массиву
	}
	
}

/* установка курсора в заданное положение и предварительная очистка дисплея от курсоров */
void Disp_Write_Kursor(uint8_t data)
{
	Disp_Write(I2C_Displ_ADDRES, COM, Set_Kyrsor, 6);							// устанавливаем курсор по ранее заданным кординатам
	Disp_Write(I2C_Displ_ADDRES, DATS, Cler_dis, 8); 							// стераем курсор
	Set_Kyrsor[4] = data + 1;   												// наши данные меньше чем адрес по этому увеличиваем на 1
	Set_Kyrsor[5] = data + 1;   												// наши данные меньше чем адрес по этому увеличиваем на 1
	Disp_Write(I2C_Displ_ADDRES, COM, Set_Kyrsor, 6);							// устанавливаем курсор новым кординатам
	Disp_Write(I2C_Displ_ADDRES, DATS, Kursor, 8);      						// вывод курсора в виде стрелки
}

/* изменение данных на дисплее */
void Izmeniti_Data(uint8_t znaki)
{
	while (Flag_Korotko)														// входим по короткаму нажатию
	{
		if (Flag_Plus)															// при вращении энкодера в бльшую сторону
		{
			Svarka_data[znaki] = Svarka_data[znaki] + Flag_Plus;  				// прибовляем к данным 1
			if (Svarka_data[znaki] > 99) {Svarka_data[znaki] = 99;}				// проверяем чтобы данные небыли больше 99
			Disp_Write_Data(znaki);												// выводим данные
			Flag_Plus = 0;														// убираем флаг
		}		
		if (Flag_Minus)															// при вращении энкодера в меньшую сторону
		{
			Svarka_data[znaki] = Svarka_data[znaki] - Flag_Minus;  				// убовляем от данных 1
			if(Svarka_data[znaki] == 0) {Svarka_data[znaki] = 1; }				// проверяем чтобы данные небыли меньше 1
			Disp_Write_Data(znaki);												// выводим данные
			Flag_Minus = 0;														// убираем флаг
		}
	}
}
/*
void Izmeniti_Data2(uint8_t znaki)
{
	while (Flag_Korotko)														// входим по короткаму нажатию
		{
			if (Flag_Plus)															// при вращении энкодера в бльшую сторону
				{
					Svarka_data[znaki] += 1; 											// прибовляем к данным 1
					if(Svarka_data[znaki] > 99) {Svarka_data[znaki] = 99; }				// проверяем чтобы данные небыли больше 99
					Disp_Write_Data(znaki); 												// выводим данные
					Flag_Plus = 0; 														// убираем флаг
				}		
			if (Flag_Minus)															// при вращении энкодера в меньшую сторону
				{
					Svarka_data[znaki] -= 1; 											// убовляем от данных 1
					if(Svarka_data[znaki] == 0) {Svarka_data[znaki] = 1; }				// проверяем чтобы данные небыли меньше 1
					Disp_Write_Data(znaki); 												// выводим данные
					Flag_Minus = 0; 														// убираем флаг
				}
		}
}*/

/* проверка данных в памяти */
void Prowerka(void)
{
	if (ADDRES_FLASH_DATA == 0xFF)												// если в памяти по первому адрессу пусто (тоесть все FF)
	{
		for (uint8_t i = 0; i < 5; i++)
		{
			Svarka_data[i] = Svarka_data_flash[i];  							// копируем в Ram шаблонные данные при первом пуске
		}
		Write_Flash_Ram(); 														// запись данных из Ram в Flash память
	}
}

/* Функция сварки */
void Svarka(void)
{
	if (Flag_Pusk)
	{
		if (Flag_Zero)
		{
			/* функция предногрева */
			for (uint8_t i = 0; i < Svarka_data[Time_Power_1]; i++)
			{
				while (Flag_Zero == 0) ;							// ждём когда произойдет переход через ноль во второй и последующии заходы
				Time = 0;     										// сброс времеени
				SysTick->VAL = 0;     								// сброс счётчика в ноль
				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;     		// включаем таймер
				while(Time < Svarka_data[Power_1]);      			// ждём пока время не дощитает
				GPIOB->BSRR |= GPIO_BSRR_BR_1;     					// включаем симистр
				while(Time <= Svarka_data[Power_1]);      			// ждём один периуд времени
				GPIOB->BSRR |= GPIO_BSRR_BS_1;     					// выключаем симистр
				SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     		// выключаем таймер
				Flag_Zero = 0;     									// сброс флага переход через ноль
			}
			
			/* поуза */
			while (Flag_Zero < Svarka_data[Pauza]) ;				// пауза между преднагревом и сваркой			
			Flag_Zero = 0;    										// сброс флага переход через ноль
			
			/* функция сварки */
			while(Flag_Zero == 0);   								// ждём когда произайдёт переход через ноль
			for(uint8_t p = 0 ; p < Svarka_data[Time_Power_2]; p++)
			{
				while (Flag_Zero == 0) ;							// ждём когда произойдет переход через ноль во второй и последующии заходы
				Time = 0;     										// сброс времеени
				SysTick->VAL = 0;     								// сброс счётчика в ноль
				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;     		// включаем таймер
				while(Time < Svarka_data[Power_2]);      			// ждём пока время не дощитает
				GPIOB->BSRR |= GPIO_BSRR_BR_1;     					// включаем симистр
				while(Time <= Svarka_data[Power_2]);      			// ждём один периуд времени
				GPIOB->BSRR |= GPIO_BSRR_BS_1;     					// выключаем симистр
				SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;     		// выключаем таймер
				Flag_Zero = 0;     									// сброс флага переход через ноль
			}
			
			Flag_Pusk = 0;   										// сброс флага запуска сварки
		}
	}
}

/* Меню управление данными на дисплее */
void Menu(void)
{
	if (!(Flag_Pusk))															// когда сварка начата нет возможности редактировать данные
	{
		uint8_t step_menu = 1;													
		while (Flag_Dolgo)														// входит только когда долго ударживали энкодер
		{
			if (step_menu != Punkt_Menu)										// входит только при изменения значения
				{
					Disp_Write_Kursor(Punkt_Menu); 								// выводим курсор
					step_menu = Punkt_Menu; 									// сохраняем значение
				}
			Izmeniti_Data(Punkt_Menu); 											// изменение значений
			
			if(!Flag_Dolgo)														// когда выходим удаляем курсор
			{
				Disp_Write(I2C_Displ_ADDRES, COM, Set_Kyrsor, 6);  				// устанавливаем курсор по ранее заданным кординатам
				Disp_Write(I2C_Displ_ADDRES, DATS, Cler_dis, 8);   				// стераем курсор
				Punkt_Menu = 0; 												// сбрасываем меню в начало
				Write_Flash_Ram();												// запись данных из Ram в Flash память
				Copi_Flash_Ram(&ADDRES_FLASH_DATA);								// копировать из Flash в Ram память данные
				//EXTI->IMR |= 0x3; 												// разрешить прерывания по EXTI 0-1
			}
		} 
	}
}



		
int main()
{
		
	RCC_Init();													// настройка тактирования
	Set_Gpio();													// настройка портов для управления и внешних прерываний
	i2c_Init();													// инициализация I2C и настройка портов для I2C
	Set_Exti(); 												// настройка внешних прерываний
	Set_Systick(); 												// настройка системного таймера
	
	Prowerka();													// если в Flash ятёрты данные то их необходимо заполнить шаблоном
	Copi_Flash_Ram(&ADDRES_FLASH_DATA); 						// копировать из Flash в Ram память данные
	
	__enable_irq();  											// разрешение глобальных прерываний
	
	Delay();													// задержка после включения модуля
	
	Disp_Write(I2C_Displ_ADDRES, COM, Init_dis, 25);			// инициализация дисплея
	Disp_Write(I2C_Displ_ADDRES, DATS, Cler_dis, 1024);			// очистка дисплея

	Disp_Write(I2C_Displ_ADDRES, COM, Set_Shablon, 6);			// установка кординат для шаблона
	Disp_Write(I2C_Displ_ADDRES, DATS, Shablon[0], 80);			// вывод на дисплей шаблона
	
	Disp_Write_Data(All_data);									// выводим на дисплей все данные
	
	EXTI->IMR |= 0x3;											// разрешить прерывания по EXTI 0-1

	while(1)
	{
		Svarka();
		Menu();
	}
}