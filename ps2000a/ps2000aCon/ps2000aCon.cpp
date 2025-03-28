/******************************************************************************
 *
 * Filename: ps2000aCon.c
 *
 * Description:
 *   This is a console mode program that demonstrates how to perform 
 *	 operations using a PicoScope 2200 Series device using the 
 *	 PicoScope 2000 Series (ps2000a) driver functions.
 *   
 *	Supported PicoScope models:
 *
 *		PicoScope 2205 MSO & 2205A MSO
 *		PicoScope 2405A
 *		PicoScope 2206, 2206A, 2206B, 2206B MSO & 2406B
 *		PicoScope 2207, 2207A, 2207B, 2207B MSO & 2407B
 *		PicoScope 2208, 2208A, 2208B, 2208B MSO & 2408B
 *
 * Examples:
 *    Collect a block of samples immediately
 *    Collect a block of samples when a trigger event occurs
 *	  Collect a block of samples using Equivalent Time Sampling (ETS)
 *    Collect samples using a rapid block capture with trigger
 *    Collect a stream of data immediately
 *    Collect a stream of data when a trigger event occurs
 *    Set Signal Generator, using standard or custom signals
 * 
 * Digital Examples (MSO veriants only): 
 *    Collect a block of digital samples immediately
 *    Collect a block of digital samples when a trigger event occurs
 *    Collect a block of analogue & digital samples when analogue AND digital trigger events occurs
 *    Collect a block of analogue & digital samples when analogue OR digital trigger events occurs
 *    Collect a stream of digital data immediately
 *	  Collect a stream of digital data and show aggregated values
 *    
 *	To build this application:-
 *
 *		If Microsoft Visual Studio (including Express) is being used:
 *
 *			Select the solution configuration (Debug/Release) and platform (x86/x64)
 *			Ensure that the 32-/64-bit ps2000a.lib can be located
 *			Ensure that the ps2000aApi.h and PicoStatus.h files can be located
 *
 *		Otherwise:
 *
 *			 Set up a project for a 32-/64-bit console mode application
 *			 Add this file to the project
 *			 Add ps2000a.lib to the project (Microsoft C only)
 *			 Add ps2000aApi.h and PicoStatus.h to the project
 *			 Build the project
 *
 *  Linux platforms:
 *
 *		Ensure that the libps2000a driver package has been installed using the
 *		instructions from https://www.picotech.com/downloads/linux
 *
 *		Place this file in the same folder as the files from the linux-build-files
 *		folder. In a terminal window, use the following commands to build
 *		the ps2000aCon application:
 *
 *			./autogen.sh <ENTER>
 *			make <ENTER>
 *
 * Copyright (C) 2011-2018 Pico Technology Ltd. See LICENSE file for terms.
 *
 ******************************************************************************/
#pragma once
#include <stdio.h>
#include "windows.h"
#include <conio.h>
#include "ps2000aApi.h"
#include <time.h>
#include <istream>



#define PREF4 __stdcall

#define		BUFFER_SIZE 	1024
#define		DUAL_SCOPE		2
#define		QUAD_SCOPE		4

#define		AWG_DAC_FREQUENCY      20e6
#define		AWG_DAC_FREQUENCY_MSO  2e6
#define		AWG_PHASE_ACCUMULATOR  4294967296.0

int32_t cycles = 0;

typedef enum
{
	ANALOGUE,
	DIGITAL,
	AGGREGATED,
	MIXED
}MODE;

typedef struct
{
	int16_t DCcoupled;
	int16_t range;
	int16_t enabled;
}CHANNEL_SETTINGS;

typedef struct tTriggerDirections
{
	PS2000A_THRESHOLD_DIRECTION channelA;
	PS2000A_THRESHOLD_DIRECTION channelB;
	PS2000A_THRESHOLD_DIRECTION channelC;
	PS2000A_THRESHOLD_DIRECTION channelD;
	PS2000A_THRESHOLD_DIRECTION ext;
	PS2000A_THRESHOLD_DIRECTION aux;
}TRIGGER_DIRECTIONS;

typedef struct tPwq
{
	PS2000A_PWQ_CONDITIONS * conditions;
	int16_t nConditions;
	PS2000A_THRESHOLD_DIRECTION direction;
	uint32_t lower;
	uint32_t upper;
	PS2000A_PULSE_WIDTH_TYPE type;
}PWQ;

typedef struct
{
	int16_t					handle;
	PS2000A_RANGE			firstRange;
	PS2000A_RANGE			lastRange;
	uint8_t					signalGenerator;
	uint8_t					ETS;
	int16_t                 channelCount;
	int16_t					maxValue;
	CHANNEL_SETTINGS		channelSettings [PS2000A_MAX_CHANNELS];
	int16_t					digitalPorts;
	int16_t					awgBufferSize;
	double					awgDACFrequency;
}UNIT;

// Глобальные переменные
uint32_t	timebase = 8;
int16_t     oversample = 1;
BOOL		scaleVoltages = TRUE;

uint16_t inputRanges [PS2000A_MAX_RANGES] = {	10,
	20,
	50,
	100,
	200,
	500,
	1000,
	2000,
	5000,
	10000,
	20000,
	50000};

BOOL     		g_ready = FALSE;
int32_t 		g_times [PS2000A_MAX_CHANNELS];
int16_t     	g_timeUnit;
int32_t      	g_sampleCount;
uint32_t		g_startIndex;
int16_t			g_autoStopped;
int16_t			g_trig = 0;
uint32_t		g_trigAt = 0;
int16_t			g_overflow = 0;

char BlockFile[20]		= "block.txt";
char DigiBlockFile[20]	= "digiblock.txt";
char StreamFile[20]		= "stream.txt";

// Используйте эту структуру, чтобы помочь в сборе потоковых данных
typedef struct tBufferInfo
{
	UNIT * unit;
	MODE mode;
	int16_t **driverBuffers;
	int16_t **appBuffers;
	int16_t **driverDigBuffers;
	int16_t **appDigBuffers;

} BUFFER_INFO;


/****************************************************************************
* CallBackStreaming
* используется при вызовах потокового сбора данных ps2000a при получении данных.
* используется для установки глобальных флагов и т.д., проверяемых пользовательскими процедурами
****************************************************************************/
void PREF4 CallBackStreaming(	int16_t handle,
	int32_t noOfSamples,
	uint32_t	startIndex,
	int16_t overflow,
	uint32_t triggerAt,
	int16_t triggered,
	int16_t autoStop,
	void	*pParameter)
{
	int32_t channel;
	int32_t digiPort;
	BUFFER_INFO * bufferInfo = NULL;

	if (pParameter != NULL)
	{
		bufferInfo = (BUFFER_INFO *) pParameter;
	}

	// используется для потоковой передачи
	g_sampleCount	= noOfSamples;
	g_startIndex	= startIndex;
	g_autoStopped	= autoStop;
	g_overflow		= overflow;

	// отметьте, что чтение данных завершено
	g_ready = TRUE;

	// флаги, показывающие, произошел ли триггер и где именно
	g_trig = triggered;
	g_trigAt = triggerAt;

	if (bufferInfo != NULL && noOfSamples)
	{
		if (bufferInfo->mode == ANALOGUE)
		{
			for (channel = 0; channel < bufferInfo->unit->channelCount; channel++)
			{
				if (bufferInfo->unit->channelSettings[channel].enabled)
				{
					if (bufferInfo->appBuffers && bufferInfo->driverBuffers)
					{
						if (bufferInfo->appBuffers[channel * 2]  && bufferInfo->driverBuffers[channel * 2])
						{
							memcpy_s (&bufferInfo->appBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t),
								&bufferInfo->driverBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t));
						}
						if (bufferInfo->appBuffers[channel * 2 + 1] && bufferInfo->driverBuffers[channel * 2 + 1])
						{
							memcpy_s (&bufferInfo->appBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t),
								&bufferInfo->driverBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t));
						}
					}
				}
			}
		}
		else if (bufferInfo->mode == AGGREGATED)
		{
			for (channel = 0; channel < bufferInfo->unit->digitalPorts; channel++)
			{
				if (bufferInfo->appDigBuffers && bufferInfo->driverDigBuffers)
				{
					if (bufferInfo->appDigBuffers[channel * 2] && bufferInfo->driverDigBuffers[channel * 2])
					{
						memcpy_s (&bufferInfo->appDigBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t),
							&bufferInfo->driverDigBuffers[channel * 2][startIndex], noOfSamples * sizeof(int16_t));
					}
					if (bufferInfo->appDigBuffers[channel * 2 + 1] && bufferInfo->driverDigBuffers[channel * 2 + 1])
					{
						memcpy_s (&bufferInfo->appDigBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t),
							&bufferInfo->driverDigBuffers[channel * 2 + 1][startIndex], noOfSamples * sizeof(int16_t));
					}
				}
			}
		}
		else if (bufferInfo->mode == DIGITAL)
		{
			for (digiPort = 0; digiPort < bufferInfo->unit->digitalPorts; digiPort++)
			{
				if (bufferInfo->appDigBuffers && bufferInfo->driverDigBuffers)
				{
					if (bufferInfo->appDigBuffers[digiPort] && bufferInfo->driverDigBuffers[digiPort])
					{
						memcpy_s (&bufferInfo->appDigBuffers[digiPort][startIndex], noOfSamples * sizeof(int16_t),
							&bufferInfo->driverDigBuffers[digiPort][startIndex], noOfSamples * sizeof(int16_t));
					}
				}
			}
		}
	}
}

/****************************************************************************
* CallBackBlock
* используется для сбора блоков данных ps2000a при получении данных.
* используется для установки глобальных флагов и т.д., проверяемых пользовательскими процедурами
****************************************************************************/
void PREF4 CallBackBlock(	int16_t handle, PICO_STATUS status, void * pParameter)
{
	if (status != PICO_CANCELLED)
	{
		g_ready = TRUE;
	}
}

/****************************************************************************
* Закрывающее устройство
****************************************************************************/
void CloseDevice(UNIT *unit)
{
	ps2000aCloseUnit(unit->handle);
}


/****************************************************************************
* Установить значения по умолчанию - восстановить настройки по умолчанию
****************************************************************************/
void SetDefaults(UNIT * unit)
{
	PICO_STATUS status;
	int32_t i;

	status = ps2000aSetEts(unit->handle, PS2000A_ETS_OFF, 0, 0, NULL); // Выключить ETS

	for (i = 0; i < unit->channelCount; i++) //сброс настроек каналов до самых последних значений
	{
		status = ps2000aSetChannel(unit->handle, (PS2000A_CHANNEL) (PS2000A_CHANNEL_A + i),
			unit->channelSettings[PS2000A_CHANNEL_A + i].enabled,
			(PS2000A_COUPLING) unit->channelSettings[PS2000A_CHANNEL_A + i].DCcoupled,
			(PS2000A_RANGE) unit->channelSettings[PS2000A_CHANNEL_A + i].range, 0);
	}
}

/****************************************************************************
* Setdigital - включение или отключение цифровых каналов
****************************************************************************/
PICO_STATUS SetDigitals(UNIT *unit, int16_t state)
{
	PICO_STATUS status;

	int16_t logicLevel;
	float logicVoltage = 1.5;
	int16_t maxLogicVoltage = 5;

	int16_t timebase = 1;
	int16_t port;


	// Установите логический порог
	logicLevel =  (int16_t) ((logicVoltage / maxLogicVoltage) * PS2000A_MAX_LOGIC_LEVEL);

	// Enable or Disable Digital ports
	for (port = PS2000A_DIGITAL_PORT0; port <= PS2000A_DIGITAL_PORT1; port++)
	{
		status = ps2000aSetDigitalPort(unit->handle, (PS2000A_DIGITAL_PORT)port, state, logicLevel);
		printf(status?"SetDigitals:ps2000aSetDigitalPort(Port 0x%X) ------ 0x%08lx \n":"", port, status);
	}
	return status;
}

/****************************************************************************
*DisableAnalog - Отключить аналоговые каналы
****************************************************************************/
PICO_STATUS DisableAnalogue(UNIT *unit)
{
	PICO_STATUS status;
	int16_t ch;

	// Отключите аналоговые каналы, сохранив настройки
	for (ch = 0; ch < unit->channelCount; ch++)
	{

		status = ps2000aSetChannel(unit->handle, (PS2000A_CHANNEL) ch, 0, (PS2000A_COUPLING) unit->channelSettings[ch].DCcoupled, 
			(PS2000A_RANGE) unit->channelSettings[ch].range, 0);

		if (status != PICO_OK)
		{
			printf("DisableAnalogue:ps2000aSetChannel(channel %d) ------ 0x%08lx \n", ch, status);
		}
	}
	return status;
}

/****************************************************************************
* RestoreAnalogueSettings - Восстанавливает настройки аналогового канала
****************************************************************************/
PICO_STATUS RestoreAnalogueSettings(UNIT *unit)
{
	PICO_STATUS status;
	int16_t ch;

	// Включите аналоговые каналы, используя предыдущие настройки
	for (ch = 0; ch < unit->channelCount; ch++)
	{

		status = ps2000aSetChannel(unit->handle, (PS2000A_CHANNEL) ch, unit->channelSettings[ch].enabled, (PS2000A_COUPLING) unit->channelSettings[ch].DCcoupled, 
			(PS2000A_RANGE) unit->channelSettings[ch].range, 0);

		if (status != PICO_OK)
		{
			printf("RestoreAnalogueSettings:ps2000aSetChannel(channel %d) ------ 0x%08lx \n", ch, status);
		}
	}
	return status;
}

/****************************************************************************
* adc_to_mv
*
* Преобразуйте значение 16-разрядного АЦП в милливольты
****************************************************************************/
int32_t adc_to_mv(int32_t raw, int32_t ch, UNIT * unit)
{
	return (raw * inputRanges[ch]) / unit->maxValue;
}

/****************************************************************************
* mv_to_adc
*
* Преобразует значение в милливольтах в 16-разрядный счетчик АЦП
*
* (полезно для настройки пороговых значений срабатывания)
****************************************************************************/
int16_t mv_to_adc(int16_t mv, int16_t ch, UNIT * unit)
{
	return (mv * unit->maxValue) / inputRanges[ch];
}

/****************************************************************************
* timeUnitsToString
*
* Преобразует перечисление PS2000A_TIME_UNITS в строку (используется для потокового режима)
*
****************************************************************************/
int8_t* timeUnitsToString(PS2000A_TIME_UNITS timeUnits)
{
	int8_t* timeUnitsStr = (int8_t*) "ns";

	switch(timeUnits)
	{
		case PS2000A_FS:

			timeUnitsStr = (int8_t*) "fs";
			break;

		case PS2000A_PS:

			timeUnitsStr = (int8_t*) "ps";
			break;

		case PS2000A_NS:

			timeUnitsStr = (int8_t*) "ns";
			break;

		case PS2000A_US:

			timeUnitsStr = (int8_t*) "us";
			break;

		case PS2000A_MS:

			timeUnitsStr = (int8_t*) "ms";
			break;

		case PS2000A_S:

			timeUnitsStr = (int8_t*) "s";
			break;

		default:

			timeUnitsStr = (int8_t*) "ns";
	}

	return timeUnitsStr;

}

/****************************************************************************
* ClearDataBuffers
*
* останавливает запись значений getData в память, которая была освобождена
****************************************************************************/
PICO_STATUS ClearDataBuffers(UNIT * unit)
{
	int32_t i;
	PICO_STATUS status;

	for (i = 0; i < unit->channelCount; i++) 
	{
		if ((status = ps2000aSetDataBuffers(unit->handle, (int16_t)i, NULL, NULL, 0, 0, PS2000A_RATIO_MODE_NONE)) != PICO_OK)
		{
			printf("ClearDataBuffers:ps2000aSetDataBuffers(channel %d) ------ 0x%08lx \n", i, status);
		}
	}


	for (i= 0; i < unit->digitalPorts; i++) 
	{
		if ((status = ps2000aSetDataBuffer(unit->handle, (PS2000A_CHANNEL) (i + PS2000A_DIGITAL_PORT0), NULL, 0, 0, PS2000A_RATIO_MODE_NONE))!= PICO_OK)
		{
			printf("ClearDataBuffers:ps2000aSetDataBuffer(port 0x%X) ------ 0x%08lx \n", i + PS2000A_DIGITAL_PORT0, status);
		}
	}

	return status;
}

/****************************************************************************
* BlockDataHandler
* - Используется всеми процедурами обработки данных блока
* - собирает данные (пользователь устанавливает режим запуска перед вызовом), отображает 10 элементов
* и сохраняет все в data.txt
* Input :
* - единица измерения: используемая единица измерения.
* - текст: текст, отображаемый перед отображением фрагмента данных
* - смещение: смещение в буфер данных для начала отображения фрагмента.
****************************************************************************/
void BlockDataHandler(UNIT * unit, const char * text, int32_t offset, MODE mode, int16_t etsModeSet)
{
	uint16_t bit;
	uint16_t bitValue;
	uint16_t digiValue;
	uint32_t segmentIndex = 0;

	int32_t i, j;
	int32_t timeInterval;
	int32_t sampleCount = BUFFER_SIZE;
	int32_t maxSamples;
	int32_t timeIndisposed;

	int16_t * buffers[PS2000A_MAX_CHANNEL_BUFFERS];
	int16_t * digiBuffer[PS2000A_MAX_DIGITAL_PORTS];

	int64_t * etsTime=0; // Буфер для данных о времени ETS

	FILE * fp = NULL;
	FILE * digiFp = NULL;
	
	PICO_STATUS status;
	PS2000A_RATIO_MODE ratioMode = PS2000A_RATIO_MODE_NONE;
	
	if (mode == ANALOGUE || mode == MIXED)		// Аналоговый или (только для MSO) СМЕШАННЫЙ
	{
		for (i = 0; i < unit->channelCount; i++) 
		{
			if (unit->channelSettings[i].enabled)
			{
				buffers[i * 2] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
				buffers[i * 2 + 1] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
				
				status = ps2000aSetDataBuffers(unit->handle, (int32_t) i, buffers[i * 2], buffers[i * 2 + 1], sampleCount, segmentIndex, ratioMode);

				printf(status?"BlockDataHandler:ps2000aSetDataBuffers(channel %d) ------ 0x%08lx \n":"", i, status);
			}
		}
	}

	// Настройте временные буферы, если данные записываются в режиме блокировки ETS (только при включенных аналоговых каналах).
	if (mode == ANALOGUE && etsModeSet == TRUE)
	{
		etsTime = (int64_t *) calloc(sampleCount, sizeof (int64_t));   
		status = ps2000aSetEtsTimeBuffer(unit->handle, etsTime, sampleCount);
	}

	if (mode == DIGITAL || mode == MIXED)		// (Только для MSO) Цифровой или СМЕШАННЫЙ
	{
		for (i= 0; i < unit->digitalPorts; i++) 
		{
			digiBuffer[i] = (int16_t*) malloc(sampleCount* sizeof(int16_t));
			status = ps2000aSetDataBuffer(unit->handle, (int32_t) (i + PS2000A_DIGITAL_PORT0), digiBuffer[i], sampleCount, 0, ratioMode);
			printf(status?"BlockDataHandler:ps2000aSetDataBuffer(port 0x%X) ------ 0x%08lx \n":"", i + PS2000A_DIGITAL_PORT0, status);
		}
	}

	/*  Проверьте текущий базовый временной индекс и найдите максимальное количество выборок и временной интервал (в наносекундах).*/
	while (ps2000aGetTimebase(unit->handle, timebase, sampleCount, &timeInterval, oversample, &maxSamples, 0) != PICO_OK)
	{
		timebase++;
	}

	if (!etsModeSet)
	{
		printf("\nTimebase: %lu  SampleInterval: %ldnS  oversample: %hd\n", timebase, timeInterval, oversample);
	}

	/* Запустите его сбор, затем дождитесь завершения*/
	g_ready = FALSE;
	status = ps2000aRunBlock(unit->handle, 0, sampleCount, timebase, oversample,	&timeIndisposed, 0, CallBackBlock, NULL);
	printf(status?"BlockDataHandler:ps2000aRunBlock ------ 0x%08lx \n":"", status);

	printf("Waiting for trigger...Press a key to abort\n");

	while (!g_ready && !_kbhit())
	{
		Sleep(0);
	}

	if (g_ready) 
	{
		status = ps2000aGetValues(unit->handle, 0, (uint32_t*) &sampleCount, 10, ratioMode, 0, NULL);
		printf(status?"BlockDataHandler:ps2000aGetValues ------ 0x%08lx \n":"", status);

		/* Распечатайте первые 10 показаний, при необходимости преобразовав их в мВ */
		printf("%s\n",text);

		if (mode == ANALOGUE || mode == MIXED)		// если мы делаем аналоговую или СМЕШАННУЮ музыку
		{
			printf("Channels are in (%s)\n\n", ( scaleVoltages ) ? ("mV") : ("ADC Counts"));

			for (j = 0; j < unit->channelCount; j++) 
			{
				if (unit->channelSettings[j].enabled) 
				{
					printf("Channel%c:\t", 'A' + j);
				}
			}

			printf("\n");
		}

		if (mode == DIGITAL || mode == MIXED)	// если мы работаем в цифровом или СМЕШАННОМ формате
		{
			printf("Digital\n");
		}

		printf("\n");

		for (i = offset; i < offset+10; i++) 
		{
			if (mode == ANALOGUE || mode == MIXED)	// если мы делаем аналоговую или СМЕШАННУЮ музыку
			{
				for (j = 0; j < unit->channelCount; j++) 
				{
					if (unit->channelSettings[j].enabled) 
					{
						printf("  %6d        ", scaleVoltages ? 
							adc_to_mv(buffers[j * 2][i], unit->channelSettings[PS2000A_CHANNEL_A + j].range, unit)	// // Если требуется масштабировать напряжения, выведите значение mV
							: buffers[j * 2][i]);																	// в противном случае выведите количество АЦП
					}
				}
			}

			if (mode == DIGITAL || mode == MIXED)	// если мы работаем в цифровом или СМЕШАННОМ формате
			{
				digiValue = 0x00ff & digiBuffer[1][i];
				digiValue <<= 8;
				digiValue |= digiBuffer[0][i];
				printf("0x%04X", digiValue);
			}
			printf("\n");
		}

		if (mode == ANALOGUE || mode == MIXED)		// если мы делаем аналоговую или СМЕШАННУЮ музыку
		{
			sampleCount = min(sampleCount, BUFFER_SIZE);

			fopen_s(&fp, BlockFile, "w");
			
			if (fp != NULL)
			{
				if (etsModeSet)
				{
					fprintf(fp, "ETS Block Data log\n\n");
				}
				else
				{
					fprintf(fp, "Block Data log\n\n");
				}

				fprintf(fp, "Results shown for each of the %d Channels are......\n",unit->channelCount);
				fprintf(fp, "Maximum Aggregated value ADC Count & mV, Minimum Aggregated value ADC Count & mV\n\n");

				if (etsModeSet)
				{
					fprintf(fp, "Time (fs) ");
				}
				else
				{
					fprintf(fp, "Time (ns)  ");
				}

				for (i = 0; i < unit->channelCount; i++) 
				{
					fprintf(fp," Ch   Max ADC  Max mV   Min ADC  Min mV  ");
				}

				fprintf(fp, "\n");

				for (i = 0; i < sampleCount; i++) 
				{
					if (mode == ANALOGUE && etsModeSet == TRUE)
					{
						fprintf(fp, "%d ", etsTime[i]);
					}
					else
					{
						fprintf(fp, "%7d ", g_times[0] + (int32_t)(i * timeInterval));
					}

					for (j = 0; j < unit->channelCount; j++) 
					{
						if (unit->channelSettings[j].enabled) 
						{
							fprintf(	fp,
								"Ch%C  %5d = %+5dmV, %5d = %+5dmV   ",
								(char)('A' + j),
								buffers[j * 2][i],
								adc_to_mv(buffers[j * 2][i], unit->channelSettings[PS2000A_CHANNEL_A + j].range, unit),
								buffers[j * 2 + 1][i],
								adc_to_mv(buffers[j * 2 + 1][i], unit->channelSettings[PS2000A_CHANNEL_A + j].range, unit));
						}
					}
					fprintf(fp, "\n");
				}
			}
			else
			{
				printf(	"Cannot open the file block.txt for writing.\n"
					"Please ensure that you have permission to access.\n");
			}
		}

		if (mode == DIGITAL || mode == MIXED)
		{
			fopen_s(&digiFp, DigiBlockFile, "w");

			if (digiFp != NULL)
			{
				fprintf(digiFp, "Block Digital Data log.\n");
				fprintf(digiFp,"Results shown for D15 - D8 and D7 to D0.\n\n");

				for(i = 0; i < sampleCount; i++)
				{
					digiValue = 0x00ff & digiBuffer[1][i];	// Замаскируйте значения порта 1, чтобы получить меньшие 8 бит
					digiValue <<= 8;						// Сдвинуть на 8 бит, чтобы поместить в верхние 8 бит 16-битного слова
					digiValue |= digiBuffer[0][i];			// Замаскируйте значения порта 0, чтобы получить меньшие 8 бит

					for (bit = 0; bit < 16; bit++)
					{
						// Значение сдвига (32768 - двоичное значение 1000 0000 0000 0000 0000) И значение, позволяющее получить 1 или 0 для канала
                        // Порядок будет от D15 до D8, затем от D7 до D0

						bitValue = (0x8000 >> bit) & digiValue? 1 : 0;
						fprintf(digiFp, "%u ", bitValue);
					}

					fprintf(digiFp, "\n");
				}
			}
			else
			{
				printf(	"Cannot open the file digiblock.txt for writing.\n"
					"Please ensure that you have permission to access.\n");
			}
		}

	} 
	else 
	{
		printf("data collection aborted\n");
		_getch();
	}

	status = ps2000aStop(unit->handle);
	printf(status?"BlockDataHandler:ps2000aStop ------ 0x%08lx \n":"", status);

	if (fp != NULL)
	{
		fclose(fp);
	}

	if (digiFp != NULL)
	{
		fclose(digiFp);
	}

	if (mode == ANALOGUE || mode == MIXED)		// Только в том случае, если мы выделим эти буферы
	{
		for (i = 0; i < unit->channelCount; i++) 
		{
			if (unit->channelSettings[i].enabled)
			{
				free(buffers[i * 2]);
				free(buffers[i * 2 + 1]);
			}
		}
	}

	if (mode == ANALOGUE && etsModeSet == TRUE)	// Только в том случае, если мы выделим эти буферы
	{
		free(etsTime);
	}

	if (mode == DIGITAL || mode == MIXED)		// Только в том случае, если мы выделим эти буферы
	{
		for (i = 0; i < unit->digitalPorts; i++) 
		{
			free(digiBuffer[i]);
		}
	}

	ClearDataBuffers(unit);
}

/****************************************************************************
* StreamDataHandler
* - Используется в двух примерах потоковых данных - запущенный и триггерный
* Входные данные:
* - unit - модуль для выборки
* - предварительный триггер - количество выборок в фазе предварительного запуска
* (0, если триггер не был установлен)
***************************************************************************/
void StreamDataHandler(UNIT * unit, uint32_t preTrigger, MODE mode)
{
	int8_t * timeUnitsStr;
	
	int16_t  autostop;
	uint16_t portValue, portValueOR, portValueAND;
	uint32_t segmentIndex = 0;

	int16_t * buffers[PS2000A_MAX_CHANNEL_BUFFERS];
	int16_t * appBuffers[PS2000A_MAX_CHANNEL_BUFFERS];
	int16_t * digiBuffers[PS2000A_MAX_DIGITAL_PORTS];
	int16_t * appDigiBuffers[PS2000A_MAX_DIGITAL_PORTS];
	
	int32_t index = 0;
	int32_t totalSamples;
	int32_t bit;
	int32_t i, j;

	int32_t sampleCount = 40000; /*убедитесь, что буфер достаточно велик */
	uint32_t postTrigger;
	uint32_t downsampleRatio = 1;
	uint32_t sampleInterval;
	uint32_t triggeredAt = 0;

	clock_t timer_start = clock();
	clock_t timer_now=0;
	double elapsed=0;

	BUFFER_INFO bufferInfo;
	FILE * fp = NULL;

	PICO_STATUS status;
	PS2000A_TIME_UNITS timeUnits;
	PS2000A_RATIO_MODE ratioMode;

	if (mode == ANALOGUE)		// Аналог
	{
		for (i = 0; i < unit->channelCount; i++) 
		{
			if (unit->channelSettings[i].enabled)
			{
				buffers[i * 2] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
				buffers[i * 2 + 1] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
				status = ps2000aSetDataBuffers(unit->handle, (int32_t)i, buffers[i * 2], buffers[i * 2 + 1], sampleCount, segmentIndex, PS2000A_RATIO_MODE_AGGREGATE);

				appBuffers[i * 2] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
				appBuffers[i * 2 + 1] = (int16_t*) malloc(sampleCount * sizeof(int16_t));

				printf(status?"StreamDataHandler:ps2000aSetDataBuffers(channel %ld) ------ 0x%08lx \n":"", i, status);
			}
		}

		downsampleRatio = 20;
		timeUnits = PS2000A_US;
		sampleInterval = 1;
		ratioMode = PS2000A_RATIO_MODE_AGGREGATE;
		postTrigger = 1000000;
		autostop = TRUE;
	}

	bufferInfo.unit = unit;
	bufferInfo.mode = mode;	
	bufferInfo.driverBuffers = buffers;
	bufferInfo.appBuffers = appBuffers;
	bufferInfo.driverDigBuffers = digiBuffers;
	bufferInfo.appDigBuffers = appDigiBuffers;

	if (mode == AGGREGATED)		// (Только для MSO) АГРЕГИРОВАННЫЙ
	{
		for (i= 0; i < unit->digitalPorts; i++) 
		{

			digiBuffers[i * 2] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
			digiBuffers[i * 2 + 1] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
			status = ps2000aSetDataBuffers(unit->handle, (PS2000A_CHANNEL) (i + PS2000A_DIGITAL_PORT0), digiBuffers[i * 2], digiBuffers[i * 2 + 1], sampleCount, 0, PS2000A_RATIO_MODE_AGGREGATE);

			appDigiBuffers[i * 2] = (int16_t*) malloc(sampleCount * sizeof(int16_t));
			appDigiBuffers[i * 2 + 1] = (int16_t*) malloc(sampleCount * sizeof(int16_t)); 

			printf(status?"StreamDataHandler:ps2000aSetDataBuffer(channel %ld) ------ 0x%08lx \n":"", i, status);
		}

		downsampleRatio = 10;
		timeUnits = PS2000A_MS;
		sampleInterval = 10;
		ratioMode = PS2000A_RATIO_MODE_AGGREGATE;
		postTrigger = 10;
		autostop = FALSE;
	}

	if (mode == DIGITAL)		// (Только для MSO) Цифровой
	{
		for (i= 0; i < unit->digitalPorts; i++) 
		{
			digiBuffers[i] = (int16_t*) malloc(sampleCount* sizeof(int16_t));
			status = ps2000aSetDataBuffer(unit->handle, (PS2000A_CHANNEL) (i + PS2000A_DIGITAL_PORT0), digiBuffers[i], sampleCount, 0, PS2000A_RATIO_MODE_NONE);

			appDigiBuffers[i] = (int16_t*) malloc(sampleCount * sizeof(int16_t));

			printf(status?"StreamDataHandler:ps2000aSetDataBuffer(channel %ld) ------ 0x%08lx \n":"", i, status);
		}

		downsampleRatio = 1;
		timeUnits = PS2000A_MS;
		sampleInterval = 10;
		ratioMode = PS2000A_RATIO_MODE_NONE;
		postTrigger = 10;
		autostop = FALSE;
	}

	if (autostop)
	{
		printf("\nStreaming Data for %lu samples", postTrigger / downsampleRatio);

		if (preTrigger)	// мы передаем 0 для предварительного запуска, если мы не настраиваем триггер
		{
			printf(" after the trigger occurs\nNote: %lu Pre Trigger samples before Trigger arms\n\n",preTrigger / downsampleRatio);
		}
		else
		{
			printf("\n\n");
		}
	}
	else
	{
		printf("\nStreaming Data continually\n\n");
	}

	g_autoStopped = FALSE;

	status = ps2000aRunStreaming(unit->handle, &sampleInterval, timeUnits, preTrigger, postTrigger - preTrigger, 
				autostop, downsampleRatio, ratioMode, (uint32_t) sampleCount);

	if (status == PICO_OK)
	{	
		timeUnitsStr = timeUnitsToString(timeUnits);
		printf("Streaming data... (interval: %d %s) Press a key to stop\n", sampleInterval, timeUnitsStr);
	}
	else
	{
		printf("StreamDataHandler:ps2000aRunStreaming ------ 0x%08lx \n", status);
	}

	if (mode == ANALOGUE)
	{
		fopen_s(&fp, StreamFile, "w");

		if (fp != NULL)
		{
			/*fprintf(fp, "For each of the %d Channels, results shown are....\n", unit->channelCount);
			fprintf(fp,"Maximum Aggregated value ADC Count & mV, Minimum Aggregated value ADC Count & mV\n\n");*/

			for (i = 0; i < unit->channelCount; i++) 
			{
				if (unit->channelSettings[i].enabled) 
				{
					fprintf(fp,"Max ADC   Max mV   Min ADC   Min mV");
				}
			}

			fprintf(fp, "\n");
		}
	}

	totalSamples = 0;

	// Захватывать данные, если не нажата клавиша или не установлен флаг g_auto Stopped при обратном вызове потоковой передачи
	while (/*!_kbhit() && */ !g_autoStopped)
	{
		timer_now = clock();
		double elapsed = (double)(timer_now - timer_start) / CLOCKS_PER_SEC;  // Прошедшее время в секундах
		if (elapsed >= 3) g_autoStopped = TRUE;
		/* Опрос до тех пор, пока не будут получены данные. До тех пор функфция получения последних значений потоковой передачи не вызовет обратный вызов */
		g_ready = FALSE;

		status = ps2000aGetStreamingLatestValues(unit->handle, CallBackStreaming, &bufferInfo);
		index ++;

		if (g_ready && g_sampleCount > 0) /* может быть готово и не содержать данных, если сработала автостопировка */
		{
			if (g_trig)
			{
				triggeredAt = totalSamples + g_trigAt;		// вычислить, где произошел срабатывание триггера в общем количестве собранных образцов
			}

			totalSamples += g_sampleCount;
			printf("\nCollected %3li samples, index = %5lu, Total: %6d samples ", g_sampleCount, g_startIndex, totalSamples);

			if (g_trig)
			{
				printf("Trig. at index %lu", triggeredAt);	// показать, где произошел срабатывание
			}

			for (i = g_startIndex; i < (int32_t)(g_startIndex + g_sampleCount); i++) 
			{
				if (mode == ANALOGUE)
				{
					if (fp != NULL)
					{
						for (j = 0; j < unit->channelCount; j++) 
						{
							if (unit->channelSettings[j].enabled) 
							{
								fprintf(	fp,
									"%d, %d, %d, %d, ",
									appBuffers[j * 2][i],
									adc_to_mv(appBuffers[j * 2][i], unit->channelSettings[PS2000A_CHANNEL_A + j].range, unit),
									appBuffers[j * 2 + 1][i],
									adc_to_mv(appBuffers[j * 2 + 1][i], unit->channelSettings[PS2000A_CHANNEL_A + j].range, unit));
							}
						}

						fprintf(fp, "\n");
					}
					else
					{
						printf("Cannot open the file stream.txt for writing.\n");
					}

				}

				if (mode == DIGITAL)
				{
					portValue = 0x00ff & appDigiBuffers[1][i];	// Замаскируйте значения порта 1, чтобы получить меньшие 8 бит
					portValue <<= 8;							// Сдвинуть на 8 бит, чтобы поместить в верхние 8 бит 16-битного слова
					portValue |= 0x00ff & appDigiBuffers[0][i];	// Замаскируйте значения порта 0, чтобы получить меньшие 8 бит

					printf("\nIndex=%04lu: Value = 0x%04X  =  ", i, portValue);

					for (bit = 0; bit < 16; bit++)
					{
						// Значение сдвига (32768 - двоичное значение 1000 0000 0000 0000 0000) И значение, позволяющее получить 1 или 0 для канала
                        // Порядок будет от D15 до D8, затем от D7 до D0
						printf( (0x8000 >> bit) & portValue? "1 " : "0 ");
					}
				}

				if (mode == AGGREGATED)
				{
					portValueOR = 0x00ff & appDigiBuffers[2][i];
					portValueOR <<= 8;
					portValueOR |= 0x00ff & appDigiBuffers[0][i];

					portValueAND = 0x00ff & appDigiBuffers[3][i];
					portValueAND <<= 8;
					portValueAND |= 0x00ff & appDigiBuffers[1][i];

					printf("\nIndex=%04lu: Bitwise  OR of last %ld readings = 0x%04X ",i,  downsampleRatio, portValueOR);
					printf("\nIndex=%04lu: Bitwise AND of last %ld readings = 0x%04X ",i,  downsampleRatio, portValueAND);
				}
			}
		}
	}

	ps2000aStop(unit->handle);

	if (!g_autoStopped) 
	{
		printf("\nData collection aborted.\n");
		_getch();
	}

	if (g_overflow)
	{
		printf("Overflow on voltage range.\n");
	}

	if (fp != NULL) 
	{
		fclose(fp);	
	}

	if (mode == ANALOGUE)		// Только в том случае, если мы выделим эти буферы
	{
		for (i = 0; i < unit->channelCount; i++) 
		{
			if (unit->channelSettings[i].enabled)
			{
				free(buffers[i * 2]);
				free(buffers[i * 2 + 1]);

				free(appBuffers[i * 2]);
				free(appBuffers[i * 2 + 1]);
			}
		}
	}

	if (mode == DIGITAL) 		// Только если мы выделим эти буферы
	{
		for (i = 0; i < unit->digitalPorts; i++) 
		{
			free(digiBuffers[i]);
			free(appDigiBuffers[i]);
		}

	}

	if (mode == AGGREGATED) 		// Только в том случае, если мы выделим эти буферы
	{
		for (i = 0; i < unit->digitalPorts * 2; i++) 
		{
			free(digiBuffers[i]);
			free(appDigiBuffers[i]);
		}
	}

	ClearDataBuffers(unit);
}


/****************************************************************************
* SetTrigger
*
* Параметры
* - *unit - указатель на структуру UNIT
* - *Свойства канала - указатель на структуру PS2000A_TRIGGER_CHANNEL_PROPERTIES
* - nChannelProperties - количество элементов PS2000A_TRIGGER_CHANNEL_PROPERTIES в channelProperties
* - *triggerConditions - указатель на структуру PS2000A_TRIGGER_CONDITIONS
* - nTriggerConditions - количество элементов PS2000A_TRIGGER_CONDITIONS в triggerConditions
* - *directions - указатель на структуру TRIGGER_DIRECTIONS
* - *pwq - указатель на структуру pwq (определитель длительности импульса)
* - delay - время задержки между запуском и первой выборкой
* - Функция auxoutputable - Не используется
* - Автозапуск - период ожидания, если запуск не происходит
* - *digitalDirections - указатель на структуру PS2000A_DIGITAL_CHANNEL_DIRECTIONS
* - nDigitalDirections - количество элементов PS2000A_DIGITAL_CHANNEL_DIRECTIONS в digitalDirections
*
* Возвращает - PICO_STATUS - для отображения успешного выполнения или в случае возникновения ошибки.
*
***************************************************************************/
PICO_STATUS SetTrigger(	UNIT * unit,
	PS2000A_TRIGGER_CHANNEL_PROPERTIES * channelProperties,
	int16_t nChannelProperties,
	PS2000A_TRIGGER_CONDITIONS * triggerConditions,
	int16_t nTriggerConditions,
	TRIGGER_DIRECTIONS * directions,
	PWQ * pwq,
	uint32_t delay,
	int16_t auxOutputEnabled,
	int32_t autoTriggerMs,
	PS2000A_DIGITAL_CHANNEL_DIRECTIONS * digitalDirections,
	int16_t nDigitalDirections)
{
	PICO_STATUS status;

	if ((status = ps2000aSetTriggerChannelProperties(unit->handle,
		channelProperties,
		nChannelProperties,
		auxOutputEnabled,
		autoTriggerMs)) != PICO_OK) 
	{
		printf("SetTrigger:ps2000aSetTriggerChannelProperties ------ Ox%8lx \n", status);
		return status;
	}

	if ((status = ps2000aSetTriggerChannelConditions(unit->handle,	triggerConditions, nTriggerConditions)) != PICO_OK) 
	{
		printf("SetTrigger:ps2000aSetTriggerChannelConditions ------ 0x%8lx \n", status);
		return status;
	}

	if ((status = ps2000aSetTriggerChannelDirections(unit->handle,
		directions->channelA,
		directions->channelB,
		directions->channelC,
		directions->channelD,
		directions->ext,
		directions->aux)) != PICO_OK) 
	{
		printf("SetTrigger:ps2000aSetTriggerChannelDirections ------ 0x%08lx \n", status);
		return status;
	}

	if ((status = ps2000aSetTriggerDelay(unit->handle, delay)) != PICO_OK) 
	{
		printf("SetTrigger:ps2000aSetTriggerDelay ------ 0x%08lx \n", status);
		return status;
	}

	if ((status = ps2000aSetPulseWidthQualifier(unit->handle,
		pwq->conditions,
		pwq->nConditions, 
		pwq->direction,
		pwq->lower, 
		pwq->upper, 
		pwq->type)) != PICO_OK)
	{
		printf("SetTrigger:ps2000aSetPulseWidthQualifier ------ 0x%08lx \n", status);
		return status;
	}

	if (unit->digitalPorts)					// Функция ps2000aSetTriggerDigitalPortProperties применима только к MSO
{
		if ((status = ps2000aSetTriggerDigitalPortProperties(unit->handle,
			digitalDirections, 
			nDigitalDirections)) != PICO_OK) 
		{
			printf("SetTrigger:ps2000aSetTriggerDigitalPortProperties ------ 0x%08lx \n", status);
			return status;
		}
	}
	return status;
}

/****************************************************************************
* Немедленный сбор блока
* эта функция демонстрирует, как собирать отдельный блок данных
* из модуля (начать сбор немедленно)
****************************************************************************/
void CollectBlockImmediate(UNIT * unit)
{
	PWQ pulseWidth;
	TRIGGER_DIRECTIONS directions;

	memset(&directions, 0, sizeof(TRIGGER_DIRECTIONS));
	memset(&pulseWidth, 0, sizeof(PWQ));

	printf("Collect block immediate\n");
	printf("Data is written to disk file (%s)\n", BlockFile);
	printf("Press a key to start...\n");
	_getch();

	SetDefaults(unit);

	/* Триггер отключен	*/
	SetTrigger(unit, NULL, 0, NULL, 0, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	BlockDataHandler(unit, "\nFirst 10 readings:\n", 0, ANALOGUE, FALSE);
}

/****************************************************************************
* Собирать блоки данных
* эта функция демонстрирует, как собирать блок
данных с использованием эквивалентной временной выборки (ETS).
****************************************************************************/
void CollectBlockEts(UNIT * unit)
{
	PICO_STATUS status;
	int32_t		ets_sampletime;
	int16_t		triggerVoltage = mv_to_adc(1000, unit->channelSettings[PS2000A_CHANNEL_A].range, unit);
	uint32_t	delay = 0;
	int16_t		etsModeSet = FALSE;

	struct tPwq pulseWidth;
	struct tTriggerDirections directions;

	PS2000A_TRIGGER_CHANNEL_PROPERTIES sourceDetails = {	triggerVoltage,
		256 * 10,
		triggerVoltage,
		256 * 10,
		PS2000A_CHANNEL_A,
		PS2000A_LEVEL};

	PS2000A_TRIGGER_CONDITIONS conditions = {	PS2000A_CONDITION_TRUE,
		PS2000A_CONDITION_DONT_CARE,
		PS2000A_CONDITION_DONT_CARE,
		PS2000A_CONDITION_DONT_CARE,
		PS2000A_CONDITION_DONT_CARE,
		PS2000A_CONDITION_DONT_CARE,
		PS2000A_CONDITION_DONT_CARE,
		PS2000A_CONDITION_DONT_CARE};


	memset(&pulseWidth, 0, sizeof(struct tPwq));
	memset(&directions, 0, sizeof(struct tTriggerDirections));
	directions.channelA = PS2000A_RISING;

	printf("Collect ETS block...\n");
	printf("Collects when value rises past %d", scaleVoltages? 
		adc_to_mv(sourceDetails.thresholdUpper,	unit->channelSettings[PS2000A_CHANNEL_A].range, unit)	// Если требуется масштабировать напряжения, выведите значение mV
		: sourceDetails.thresholdUpper);																	// в противном случае выведите количество АЦП
	
	printf(scaleVoltages? "mV\n" : "ADC Counts\n");
	printf("Press a key to start...\n");
	_getch();

	SetDefaults(unit);

	// Активирован триггер
	// Нарастающий фронт
	// Порог = 1000 мВ
	status = SetTrigger(unit, &sourceDetails, 1, &conditions, 1, &directions, &pulseWidth, delay, 0, 0, 0, 0);

	status = ps2000aSetEts(unit->handle, PS2000A_ETS_FAST, 20, 4, &ets_sampletime);

	if (status == PICO_OK)
	{
		etsModeSet = TRUE;
	}
	else
	{
		printf("CollectBlockEts:ps2000aSetEts ------ 0x%08lx \n", status);
	}

	printf("ETS Sample Time is: %ld picoseconds\n", ets_sampletime);

	BlockDataHandler(unit, "Ten readings after trigger\n", BUFFER_SIZE / 10 - 5, ANALOGUE, etsModeSet); // 10% данных предварительно обработаны

	status = ps2000aSetEts(unit->handle, PS2000A_ETS_OFF, 20, 4, &ets_sampletime);

	etsModeSet = FALSE;
}

/****************************************************************************
* CollectBlockTriggered
* эта функция демонстрирует, как собирать отдельный блок данных из блока
* когда происходит запускающее событие.
****************************************************************************/
void CollectBlockTriggered(UNIT * unit)
{
	int16_t	triggerVoltage = mv_to_adc(1000, unit->channelSettings[PS2000A_CHANNEL_A].range, unit);

	PS2000A_TRIGGER_CHANNEL_PROPERTIES sourceDetails = {	triggerVoltage,
		256 * 10,
		triggerVoltage,
		256 * 10,
		PS2000A_CHANNEL_A,
		PS2000A_LEVEL};

	PS2000A_TRIGGER_CONDITIONS conditions = {	PS2000A_CONDITION_TRUE,				// Канал А
		PS2000A_CONDITION_DONT_CARE,		// Канал B 
		PS2000A_CONDITION_DONT_CARE,		// Канал C
		PS2000A_CONDITION_DONT_CARE,		// Канал D
		PS2000A_CONDITION_DONT_CARE,		// Внешний
		PS2000A_CONDITION_DONT_CARE,		// Вспомогательный
		PS2000A_CONDITION_DONT_CARE,		// PWQ
		PS2000A_CONDITION_DONT_CARE};		// Цифровой



	TRIGGER_DIRECTIONS directions = {	PS2000A_RISING,			// Канал А
		PS2000A_NONE,			// Канал B
		PS2000A_NONE,			// Канал C
		PS2000A_NONE,			// Канал D
		PS2000A_NONE,			// Внутренний
		PS2000A_NONE };			// Вспомогательный

	PWQ pulseWidth;
	memset(&pulseWidth, 0, sizeof(PWQ));

	printf("Collect block triggered\n");
	printf("Data is written to disk file (%s)\n", BlockFile);
	printf("Collects when value rises past %d", scaleVoltages?
		adc_to_mv(sourceDetails.thresholdUpper, unit->channelSettings[PS2000A_CHANNEL_A].range, unit)	// При масштабировании напряжений выведите значение в мВ
		: sourceDetails.thresholdUpper);																// в противном случае выведите количество АЦП
	printf(scaleVoltages?"mV\n" : "ADC Counts\n");

	printf("Press a key to start...\n");
	_getch();

	SetDefaults(unit);

	/* Активирован триггер
	 * Нарастающий фронт
	 * Порог = 1000 мВ */
	SetTrigger(unit, &sourceDetails, 1, &conditions, 1, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	BlockDataHandler(unit, "Ten readings after trigger\n", 0, ANALOGUE, FALSE);
}

/****************************************************************************
* CollectRapidBlock
* эта функция демонстрирует, как собирать набор снимков, используя
* режим быстрого блока.
****************************************************************************/
void CollectRapidBlock(UNIT * unit)
{
	int16_t i;
	int16_t channel;
	int16_t ***rapidBuffers;
	int16_t *overflow;
	
	uint32_t nCaptures;
	uint32_t capture;
	uint32_t maxSegments = 0;
	
	int32_t nMaxSamples;
	int32_t timeIndisposed;
	
	uint32_t nSamples = 1000;
	uint32_t nCompletedCaptures;

	PICO_STATUS status;

	// Преобразовать пороговое значение в значения АЦП
	int16_t	triggerVoltage = mv_to_adc(100, unit->channelSettings[PS2000A_CHANNEL_A].range, unit);

	struct tPS2000ATriggerChannelProperties sourceDetails = {	triggerVoltage,
																256,
																triggerVoltage,
																256,
																PS2000A_CHANNEL_A,
																PS2000A_LEVEL};

	struct tPS2000ATriggerConditions conditions = {	PS2000A_CONDITION_TRUE,				// Канал А
													PS2000A_CONDITION_DONT_CARE,		// Канал B
													PS2000A_CONDITION_DONT_CARE,		// Канал C
													PS2000A_CONDITION_DONT_CARE,		// Канал D
													PS2000A_CONDITION_DONT_CARE,		// Внешний
													PS2000A_CONDITION_DONT_CARE,		// Вспомогательный
													PS2000A_CONDITION_DONT_CARE,		// PWQ
													PS2000A_CONDITION_DONT_CARE};		// Цифровой

	struct tPwq pulseWidth;

	struct tTriggerDirections directions = {	PS2000A_RISING,			// Канал А
												PS2000A_NONE,			// Канал B
												PS2000A_NONE,			// Канал C
												PS2000A_NONE,			// Канал D
												PS2000A_NONE,			// Внутренний
												PS2000A_NONE };			// Вспомогательный

	memset(&pulseWidth, 0, sizeof(struct tPwq));

	printf("Collect rapid block triggered...\n");
	printf("Collects when value rises past %d",	scaleVoltages?
		adc_to_mv(sourceDetails.thresholdUpper, unit->channelSettings[PS2000A_CHANNEL_A].range, unit)	// При масштабировании напряжений выведите значение в мВ
		: sourceDetails.thresholdUpper);																    // в противном случае выведите количество АЦП
	
	printf(scaleVoltages?"mV\n" : "ADC Counts\n");
	printf("Press any key to abort\n");

	SetDefaults(unit);

	// Активирован триггер
	SetTrigger(unit, &sourceDetails, 1, &conditions, 1, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	// Установите количество снимков
	nCaptures = 10;

	// Найдите максимальное количество сегментов для устройства
	status = ps2000aGetMaxSegments(unit->handle, &maxSegments);

	if (nCaptures > maxSegments)
	{
		nCaptures = maxSegments;
	}

	// Сегментировать память
	status = ps2000aMemorySegments(unit->handle, nCaptures, &nMaxSamples);

	// Установите количество снимков
	status = ps2000aSetNoOfCaptures(unit->handle, nCaptures);

	// Запустить
	timebase = 160;		// Обратитесь к разделу Временных баз Руководства программиста
	status = ps2000aRunBlock(unit->handle, 0, nSamples, timebase, 1, &timeIndisposed, 0, CallBackBlock, NULL);

	// Подождите, пока данные не будут готовы
	g_ready = 0;

	while(!g_ready && !_kbhit())
	{
		Sleep(0);
	}

	if (!g_ready)
	{
		_getch();
		status = ps2000aStop(unit->handle);
		status = ps2000aGetNoOfCaptures(unit->handle, &nCompletedCaptures);
		
		printf("Rapid capture aborted. %lu complete blocks were captured\n", nCompletedCaptures);
		printf("\nPress any key...\n\n");
		_getch();

		if (nCompletedCaptures == 0)
		{
			return;
		}

		// Отображать только те блоки, которые были захвачены
		nCaptures = (uint16_t) nCompletedCaptures;
	}

	// Выделить память
	rapidBuffers = (int16_t ***) calloc(unit->channelCount, sizeof(int16_t*));
	overflow = (int16_t *) calloc(unit->channelCount * nCaptures, sizeof(int16_t));

	for (channel = 0; channel < unit->channelCount; channel++) 
	{
		rapidBuffers[channel] = (int16_t**) calloc(nCaptures, sizeof(int16_t*));
	}

	for (channel = 0; channel < unit->channelCount; channel++) 
	{	
		if (unit->channelSettings[channel].enabled)
		{
			for (capture = 0; capture < nCaptures; capture++) 
			{
				rapidBuffers[channel][capture] = (int16_t *) calloc(nSamples, sizeof(int16_t));
			}
		}
	}

	for (channel = 0; channel < unit->channelCount; channel++) 
	{
		if (unit->channelSettings[channel].enabled)
		{
			for (capture = 0; capture < nCaptures; capture++) 
			{
				status = ps2000aSetDataBuffer(unit->handle, channel, rapidBuffers[channel][capture], nSamples, capture, PS2000A_RATIO_MODE_NONE);
			}
		}
	}

	// Получить данные
	status = ps2000aGetValuesBulk(unit->handle, &nSamples, 0, nCaptures - 1, 1, PS2000A_RATIO_MODE_NONE, overflow);

	// Остановить
	status = ps2000aStop(unit->handle);

	// Распечатайте первые 10 образцов из каждого снимка
	for (capture = 0; capture < nCaptures; capture++)
	{
		printf("\nCapture %d:\n\n", capture + 1);

		for (channel = 0; channel < unit->channelCount; channel++) 
		{
			printf("Channel %c\t", 'A' + channel);
		}

		printf("\n");

		for(i = 0; i < 10; i++)
		{
			for (channel = 0; channel < unit->channelCount; channel++) 
			{
				if (unit->channelSettings[channel].enabled)
				{
					printf("%d\t\t", rapidBuffers[channel][capture][i]);
				}
			}

			printf("\n");
		}
	}

	// Свободная память
	free(overflow);

	for (channel = 0; channel < unit->channelCount; channel++) 
	{	
		if (unit->channelSettings[channel].enabled)
		{
			for (capture = 0; capture < nCaptures; capture++) 
			{
				free(rapidBuffers[channel][capture]);
			}
		}
	}

	for (channel = 0; channel < unit->channelCount; channel++) 
	{
		free(rapidBuffers[channel]);
	}

	free(rapidBuffers);

	// Установите количество сегментов и сохраните значение 1
    // status = ps2000aMemorySegments(единица измерения->дескриптор, 1, &nMaxSamples);
	// статус = ps2000aSetNoOfCaptures(модуль->ручка, 1);

}

/****************************************************************************
* Инициализируйте структуру модуля с помощью настроек по умолчанию для конкретных вариантов
****************************************************************************/
void get_info(UNIT * unit)
{
	int8_t description [11][25]= { "Driver Version",
									"USB Version",
									"Hardware Version",
									"Variant Info",
									"Serial",
									"Cal Date",
									"Kernel",
									"Digital H/W",
									"Analogue H/W",
									"Firmware 1",
									"Firmware 2"};

	int16_t i, r = 0;
	int8_t line [80];
	PICO_STATUS status = PICO_OK;
	int16_t numChannels = DUAL_SCOPE;
	int8_t channelNum = 0; 
	int8_t character = 'A';

	unit->signalGenerator	= TRUE;
	unit->ETS				= FALSE;
	unit->firstRange		= PS2000A_20MV; // Это для новых моделей PicoScope 220X B, B MSO, 2405A и 2205A MSO, более старые устройства будут иметь первый диапазон 50 мВ
	unit->lastRange			= PS2000A_20V; 
	unit->channelCount		= DUAL_SCOPE;
	unit->digitalPorts      = 0;
	unit->awgBufferSize		= PS2000A_MAX_SIG_GEN_BUFFER_SIZE;

	if (unit->handle) 
	{
		for (i = 0; i < 11; i++) 
		{
			status = ps2000aGetUnitInfo(unit->handle, (int8_t *) line, sizeof (line), &r, i);
			
			if (i == PICO_VARIANT_INFO) 
			{
				// Проверьте, имеет ли устройство четыре канала

				channelNum = line[1];
				numChannels = atoi((char*)  & channelNum);

				if (numChannels == QUAD_SCOPE)
				{
					unit->channelCount = QUAD_SCOPE;
				}

				// Установите первый диапазон напряжения, если устройство представляет собой 2206/7/8, 2206/7/8A или 2205 MSO
				if (numChannels == DUAL_SCOPE)
				{
					if (strlen((char*)line) == 4 || (strlen((char*)line) == 5 && _strcmpi((char*)&line[4], "A") == 0) || (_strcmpi((char*)line, "2205MSO")) == 0)
					{
						unit->firstRange = PS2000A_50MV;
					}
				}

				// Проверьте, является ли устройство MSO 
				if (strstr((char*)line, "MSO"))
				{
					unit->digitalPorts = 2;
				}

			}
			printf("%s: %s\n", description[i], line);
		}
	}
}

/****************************************************************************
* Выберите диапазоны входного напряжения для каналов
****************************************************************************/
void SetVoltages(UNIT * unit)
{
	int32_t i, ch;
	int32_t count = 0;

	/* See what ranges are available... */
	for (i = unit->firstRange; i <= unit->lastRange; i++) 
	{
		printf("%d -> %d mV\n", i, inputRanges[i]);
	}

	do
	{
		/* Попросите пользователя выбрать диапазон */
		printf(u8"Укажите диапазон напряжений (%d..%d)\n", unit->firstRange, unit->lastRange);
		printf(u8"99 - выключает канал\n");
		for (ch = 0; ch < unit->channelCount; ch++) 
		{
			printf("\n");
			do 
			{
				printf(u8"Канал %c: ", 'A' + ch);
				fflush(stdin);
				scanf_s("%hd", &unit->channelSettings[ch].range);
			} while (unit->channelSettings[ch].range != 99 && (unit->channelSettings[ch].range < unit->firstRange || unit->channelSettings[ch].range > unit->lastRange));

			if (unit->channelSettings[ch].range != 99) 
			{
				printf(u8" - %d мВ\n", inputRanges[unit->channelSettings[ch].range]);
				unit->channelSettings[ch].enabled = TRUE;
				count++;
			} 
			else 
			{
				printf(u8"Канал выключен\n");
				unit->channelSettings[ch].enabled = FALSE;
				unit->channelSettings[ch].range = PS2000A_MAX_RANGES-1;
			}
		}
		printf(count == 0? u8"\n** Должен быть включен как минимум 1 канал **\n\n":"");
	}
	while(count == 0);	// должен быть включен хотя бы один канал

	SetDefaults(unit);	// Ввести эти изменения в действие
}

/****************************************************************************
*
* Выберите временную базу, установите для избыточной выборки значение вкл., а единицы измерения времени - наносекунды
*
****************************************************************************/
void SetTimebase(UNIT unit)
{
	int32_t timeInterval;
	int32_t maxSamples;

	printf(u8"Укажите желаемый временной интервал: ");
	fflush(stdin);
	scanf_s("%lud", &timebase);

	while (ps2000aGetTimebase(unit.handle, timebase, BUFFER_SIZE, &timeInterval, 1, &maxSamples, 0))
	{
		timebase++;  // Увеличьте временную базу, если указанная не может быть использована.
	}

	printf(u8"Базовый показатель времени, %lu использованный  = %ld ns\n", timebase, timeInterval);
	oversample = TRUE;
}



/****************************************************************************
* SetSignalGenerator
* - позволяет пользователю задавать частоту и форму сигнала
* - позволяет настраивать форму сигнала (значения -32768..32767) до 8192 выборок int32_t
***************************************************************************/
void SetSignalGenerator(UNIT unit)
{
	PICO_STATUS status;
	int16_t waveform;
	int32_t frequency;
	char fileName [128];
	FILE * fp = NULL;
	int16_t arbitraryWaveform [PS2000A_MAX_SIG_GEN_BUFFER_SIZE];
	int16_t waveformSize = 0;
	uint32_t pkpk = 2000000;
	int32_t offset = 0;
	uint32_t delta =0;
	char ch;
	int16_t choice;

	memset(&arbitraryWaveform, 0, PS2000A_MAX_SIG_GEN_BUFFER_SIZE);

	while (_kbhit())			// используйте максимальное нажатие клавиши
	{
		_getch();
	}

	do
	{
		printf("\nSignal Generator\n================\n");
		printf("0 - SINE         1 - SQUARE\n");
		printf("2 - TRIANGLE     3 - DC VOLTAGE\n");
		printf("4 - RAMP UP      5 - RAMP DOWN\n");
		printf("6 - SINC         7 - GAUSSIAN\n");
		printf("8 - HALF SINE    A - AWG WAVEFORM\n");
		printf("F - SigGen Off\n\n");

		ch = _getch();

		if (ch >= '0' && ch <='9')
			choice = ch -'0';
		else
			ch = toupper(ch);
	}
	while(ch != 'A' && ch != 'F' && (ch < '0' || ch > '8')  );




	if (ch == 'F')				// Если мы собираемся отключить siggen
	{
		printf("Signal generator Off\n");
		waveform = 8;		// Напряжение постоянного тока
		pkpk = 0;			// 0В
		waveformSize = 0;
	}
	else
		if (ch == 'A' )		// Установите AWG
		{
			waveformSize = 0;

			printf("Select a waveform file to load: ");
			scanf_s("%s", fileName, 128);
			if (fopen_s(&fp, fileName, "r") == 0) 
			{ 
				// Открыв файл, введите данные - по одному числу в строке (максимум 8192 строки), со значениями от (-32768 до 32767)
				while (EOF != fscanf_s(fp, "%hi", (arbitraryWaveform + waveformSize))&& waveformSize++ < (PS2000A_MAX_SIG_GEN_BUFFER_SIZE - 1));
				fclose(fp);
				printf("File successfully loaded\n");
			} 
			else 
			{
				printf("Invalid filename\n");
				return;
			}
		}
		else			// Установите одну из встроенных форм сигнала
		{
			switch (choice)
			{
				case 0:
					waveform = PS2000A_SINE;
					break;

				case 1:
					waveform = PS2000A_SQUARE;
					break;

				case 2:
					waveform = PS2000A_TRIANGLE;
					break;

				case 3:
					waveform = PS2000A_DC_VOLTAGE;
					do 
					{
						printf("\nEnter offset in uV: (0 to 2500000)\n"); // Попросите пользователя ввести уровень смещения по постоянному току
						scanf_s("%lu", &offset);
					} while (offset < 0 || offset > 10000000);
					break;

				case 4:
					waveform = PS2000A_RAMP_UP;
					break;

				case 5:
					waveform = PS2000A_RAMP_DOWN;
					break;

				case 6:
					waveform = PS2000A_SINC;
					break;

				case 7:
					waveform = PS2000A_GAUSSIAN;
					break;

				case 8:
					waveform = PS2000A_HALF_SINE;
					break;

				default:
					waveform = PS2000A_SINE;
					break;
			}
		}

		if (waveform < 8 || ch == 'A' )				// При необходимости уточните частоту
		{
			do 
			{
				printf("\nEnter frequency in Hz: (1 to 1000000)\n"); // Попросите пользователя ввести частоту сигнала
				scanf_s("%lu", &frequency);
			} while (frequency <= 0 || frequency > 1000000);
		}

		if (waveformSize > 0)		
		{
			ps2000aSigGenFrequencyToPhase(unit.handle, frequency, PS2000A_SINGLE, waveformSize, &delta);

			status = ps2000aSetSigGenArbitrary(	unit.handle, 
				0, 
				pkpk, 
				(uint32_t) delta, 
				(uint32_t) delta, 
				0, 
				0, 
				arbitraryWaveform, 
				waveformSize, 
				(PS2000A_SWEEP_TYPE) 0,
				(PS2000A_EXTRA_OPERATIONS) 0, 
				PS2000A_SINGLE, 
				0, 
				0, 
				PS2000A_SIGGEN_RISING,
				PS2000A_SIGGEN_NONE, 
				0);

			printf(status?"\nps2000aSetSigGenArbitrary: Status Error 0x%x \n":"", (uint32_t)status);		// Если status != 0, выводится сообщение об ошибке
		} 
		else 
		{
			status = ps2000aSetSigGenBuiltIn(unit.handle, offset, pkpk, waveform, (float)frequency, (float)frequency, 0, 0, 
				(PS2000A_SWEEP_TYPE) 0, (PS2000A_EXTRA_OPERATIONS) 0, 0, 0, (PS2000A_SIGGEN_TRIG_TYPE) 0, (PS2000A_SIGGEN_TRIG_SOURCE) 0, 0);

			printf(status?"\nps2000aSetSigGenBuiltIn: Status Error 0x%x \n":"", (uint32_t)status);		// Если status != 0, выводится сообщение об ошибке
		}
}

/****************************************************************************
* CollectStreamingImmediate
* эта функция демонстрирует, как собирать поток данных
* с устройства (начать сбор немедленно)
***************************************************************************/
void CollectStreamingImmediate(UNIT * unit)
{
	struct tPwq pulseWidth;
	struct tTriggerDirections directions;

	memset(&pulseWidth, 0, sizeof(struct tPwq));
	memset(&directions, 0, sizeof(struct tTriggerDirections));

	SetDefaults(unit);

	printf("Collect streaming...\n");
	printf("Data is written to disk file (%s)\n", StreamFile);
	printf("Press a key to start...\n");
	_getch();

	/* Триггер отключен	*/
	SetTrigger(unit, NULL, 0, NULL, 0, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	StreamDataHandler(unit, 0, ANALOGUE);
}

/****************************************************************************
* CollectStreamingTriggered
* эта функция демонстрирует, как собирать поток данных
* с устройства (начать сбор при запуске)
***************************************************************************/
void CollectStreamingTriggered(UNIT * unit)
{
	int16_t triggerVoltage = mv_to_adc(1000,	unit->channelSettings[PS2000A_CHANNEL_A].range, unit); // КаналInfo хранит количество АЦП
	struct tPwq pulseWidth;

	struct tPS2000ATriggerChannelProperties sourceDetails = {	triggerVoltage,
		256 * 10,
		triggerVoltage,
		256 * 10,
		PS2000A_CHANNEL_A,
		PS2000A_LEVEL };

	struct tPS2000ATriggerConditions conditions = {	PS2000A_CONDITION_TRUE,				// Канал А
		PS2000A_CONDITION_DONT_CARE,		// Канал B
		PS2000A_CONDITION_DONT_CARE,		// Канал C
		PS2000A_CONDITION_DONT_CARE,		// Канал D
		PS2000A_CONDITION_DONT_CARE,		// Внешний
		PS2000A_CONDITION_DONT_CARE,		// Вспомогательный
		PS2000A_CONDITION_DONT_CARE,		// PWQ
		PS2000A_CONDITION_DONT_CARE };		// Цифровой

	struct tTriggerDirections directions = {	PS2000A_RISING,			// Канал А
		PS2000A_NONE,			// Канал B
		PS2000A_NONE,			// Канал C
		PS2000A_NONE,			// Канал D
		PS2000A_NONE,			// Внешний
		PS2000A_NONE };			// Вспомогательный

	memset(&pulseWidth, 0, sizeof(struct tPwq));
	/*
	printf("Collect streaming triggered...\n");
	printf("Data is written to disk file (%s)\n", StreamFile);
	printf("Indicates when value rises past %d", scaleVoltages?
		adc_to_mv(sourceDetails.thresholdUpper, unit->channelSettings[PS2000A_CHANNEL_A].range, unit)		// При масштабировании напряжений выведите значение в мВ
		: sourceDetails.thresholdUpper);																        // в противном случае выведите количество АЦП
	printf(scaleVoltages?"mV\n" : "ADC Counts\n");
	printf("Press a key to start...\n");
	_getch();*/

	SetDefaults(unit);

	/* Активирован триггер
	 * Нарастающий фронт
	 * Порог = 1000 мВ */
	SetTrigger(unit, &sourceDetails, 1, &conditions, 1, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	StreamDataHandler(unit, 0, ANALOGUE);
}


/****************************************************************************
* OpenDevice
* Параметры
* - указатель единицы измерения на структуру единицы измерения, в которой будет сохранен дескриптор
*
* Возвращает
* - PICO_STATUS для указания на успешное выполнение или в случае возникновения ошибки
***************************************************************************/
PICO_STATUS OpenDevice(UNIT *unit)
{
	int16_t value = 0;
	int32_t i;
	PWQ pulseWidth;
	TRIGGER_DIRECTIONS directions;

	PICO_STATUS status = ps2000aOpenUnit(&(unit->handle), NULL);

	printf(u8"Ручка: %d\n", unit->handle);

	if (status != PICO_OK) 
	{
		printf(u8"Не удается открыть устройство\n");
		printf(u8"Код ошибки : %d\n", (int32_t)status);
		while(!_kbhit());
		exit(99); // выход из программы
	}

	printf(u8"Устройство успешно открыто, цикл %d\n\n", ++cycles);

	// настройка устройств
	get_info(unit);
	timebase = 1;

	ps2000aMaximumValue(unit->handle, &value);
	unit->maxValue = value;

	for ( i = 0; i < unit->channelCount; i++) {
		unit->channelSettings[i].enabled = TRUE;
		unit->channelSettings[i].DCcoupled = TRUE;
		unit->channelSettings[i].range = PS2000A_5V;
	}

	for (i = 1; i < unit->channelCount; i++) {
		unit->channelSettings[i].enabled = FALSE;
	}

	memset(&directions, 0, sizeof(TRIGGER_DIRECTIONS));
	memset(&pulseWidth, 0, sizeof(PWQ));

	SetDefaults(unit);

	/* Триггер отключен	*/
	SetTrigger(unit, NULL, 0, NULL, 0, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	return status;
}


/****************************************************************************
* DisplaySettings
* В этом примере отображается информация о настраиваемых пользователем параметрах
* Параметры
* - указатель единицы измерения на структуру единицы измерения
* Возвращает значение none
***************************************************************************/
void DisplaySettings(UNIT *unit)
{
	int32_t ch;
	int32_t voltage;

	printf(u8"\n\nПоказания будут масштабироваться в (%s)\n", (scaleVoltages)? (u8"мВ") : ("ADC counts"));

	for (ch = 0; ch < unit->channelCount; ch++)
	{
		if (!(unit->channelSettings[ch].enabled))
		{
			printf(u8"Диапазон напряжений канала %c = Выкл.\n", 'A' + ch);
		}
		else
		{
			voltage = inputRanges[unit->channelSettings[ch].range];
			printf(u8"Диапазон напряжений канала, % c = ", 'A' + ch);

			if (voltage < 1000)
			{
				printf("%dmV\n", voltage);
			}
			else
			{
				printf("%dV\n", voltage / 1000);
			}
		}
	}
	printf("\n");

	if (unit->digitalPorts > 0)
	{
		printf( u8"Цифровые порты отключены.\n\n");
	}
}


/****************************************************************************
* ANDAnalogueDigital
* Эта функция показывает, как собирать блок данных с аналоговых
* и цифровых портов одновременно, срабатывая при выполнении
* цифровых и аналоговых условий
*
* Возвращает значение none
***************************************************************************/
void ANDAnalogueDigitalTriggered(UNIT * unit)
{
	int32_t channel = 0;
	PICO_STATUS status = PICO_OK;

	int16_t	triggerVoltage = mv_to_adc(1000, unit->channelSettings[PS2000A_CHANNEL_A].range, unit);


	PS2000A_TRIGGER_CHANNEL_PROPERTIES sourceDetails = {	triggerVoltage,			// Установщик порогового значения
															256 * 10,				// Установщик порогового значения Гистерезис
															triggerVoltage,			// Понижающий порог
															256 * 10,				// Понижающий порог Гистерезис
															PS2000A_CHANNEL_A,		// Канал
															PS2000A_LEVEL};			// Режим

	PS2000A_TRIGGER_CONDITIONS conditions = {	PS2000A_CONDITION_TRUE,					// Канал А
												PS2000A_CONDITION_DONT_CARE,			// Канал B
												PS2000A_CONDITION_DONT_CARE,			// Канал C
												PS2000A_CONDITION_DONT_CARE,			// Канал D
												PS2000A_CONDITION_DONT_CARE,			// Внешний
												PS2000A_CONDITION_DONT_CARE,			// Вспомогательный 
												PS2000A_CONDITION_DONT_CARE,			// pwq
												PS2000A_CONDITION_TRUE};				// Цифровой

	TRIGGER_DIRECTIONS directions = {	PS2000A_ABOVE,				// Канал А
										PS2000A_NONE,				// Канал B
										PS2000A_NONE,				// Канал C
										PS2000A_NONE,				// Канал D
										PS2000A_NONE,				// Внешний
										PS2000A_NONE };				// Вспомогательный

	PS2000A_DIGITAL_CHANNEL_DIRECTIONS digDirections[2];		// Размер массива может достигать 16, запись для каждого цифрового бита

	PWQ pulseWidth;
	memset(&pulseWidth, 0, sizeof(PWQ));
	
	// Установите цифровой триггер таким образом, чтобы он срабатывал при увеличении разряда 0 и увеличении разряда 4
	// Все необъявленные биты принимаются как PS2000A_DIGITAL_DONT_CARE

	digDirections[0].channel = PS2000A_DIGITAL_CHANNEL_0;
	digDirections[0].direction = PS2000A_DIGITAL_DIRECTION_RISING;

	digDirections[1].channel = PS2000A_DIGITAL_CHANNEL_4;
	digDirections[1].direction = PS2000A_DIGITAL_DIRECTION_HIGH;

	printf("\nCombination Block Triggered\n");
	printf("Collects when value is above %d", scaleVoltages?
		adc_to_mv(sourceDetails.thresholdUpper, unit->channelSettings[PS2000A_CHANNEL_A].range, unit)	// При масштабировании напряжений выведите значение в мВ
		: sourceDetails.thresholdUpper);																    // в противном случае выведите количество АЦП
	
	printf(scaleVoltages?"mV\n" : "ADC Counts\n");

	printf("AND \n");
	printf("Digital Channel  0   --- Rising\n");
	printf("Digital Channel  4   --- High\n");
	printf("Other Digital Channels - Don't Care\n");

	printf("Press a key to start...\n");
	_getch();

	for (channel = 0; channel < unit->channelCount; channel++)
	{
		unit->channelSettings[channel].enabled = TRUE;
	}

	SetDefaults(unit);	// Включить аналоговые каналы.

	/* Активирован триггер
	 * Нарастающий фронт
	 * Порог = 100 мВ */

	status = SetTrigger(unit, &sourceDetails, 1, &conditions, 1, &directions, &pulseWidth, 0, 0, 0, digDirections, 2);

	if (status == PICO_OK)
	{
		BlockDataHandler(unit, "\nFirst 10 readings:\n", 0, MIXED, FALSE);
	}

	DisableAnalogue(unit);			// Отключите аналоговые порты по завершении работы
}


/****************************************************************************
* ORAnalogueDigital
* Эта функция показывает, как собирать блок данных из аналоговых
* и цифровых портов одновременно, срабатывая при выполнении либо цифровых условий
* либо аналоговых условий
*
* Возвращает значение none
***************************************************************************/
void ORAnalogueDigitalTriggered(UNIT * unit)
{
	int32_t channel = 0;

	PICO_STATUS status = PICO_OK;

	int16_t	triggerVoltage = mv_to_adc(1000, unit->channelSettings[PS2000A_CHANNEL_A].range, unit);

	PS2000A_TRIGGER_CHANNEL_PROPERTIES sourceDetails = {	triggerVoltage,		// Установщик порогового значения
															256 * 10,			// Установщик порогового значения Гистерезис
															triggerVoltage,		// Понижающий порог
															256 * 10,			// Понижающий порог Гистерезис
															PS2000A_CHANNEL_A,	// Канал
															PS2000A_LEVEL};		// Режим


	PS2000A_TRIGGER_CONDITIONS conditions[2];


	TRIGGER_DIRECTIONS directions = {	PS2000A_RISING,			// Канал А
										PS2000A_NONE,			// Канал B
										PS2000A_NONE,			// Канал C
										PS2000A_NONE,			// Канал D
										PS2000A_NONE,			// Внешний
										PS2000A_NONE };			// Вспомогательный

	PS2000A_DIGITAL_CHANNEL_DIRECTIONS digDirections[2];		// Размер массива может достигать 16, запись для каждого цифрового бита

	PWQ pulseWidth;


	conditions[0].channelA				= PS2000A_CONDITION_TRUE;					// Канал А
	conditions[0].channelB				= PS2000A_CONDITION_DONT_CARE;				// Канал B
	conditions[0].channelC				= PS2000A_CONDITION_DONT_CARE;				// Канал C
	conditions[0].channelD				= PS2000A_CONDITION_DONT_CARE;				// Канал D
	conditions[0].external				= PS2000A_CONDITION_DONT_CARE;				// Внешний
	conditions[0].aux					= PS2000A_CONDITION_DONT_CARE;				// Вспомогательный
	conditions[0].pulseWidthQualifier	= PS2000A_CONDITION_DONT_CARE;				// pwq
	conditions[0].digital				= PS2000A_CONDITION_DONT_CARE;				// Цифровой


	conditions[1].channelA				= PS2000A_CONDITION_DONT_CARE;				// Канал А
	conditions[1].channelB				= PS2000A_CONDITION_DONT_CARE;				// Канал B
	conditions[1].channelC				= PS2000A_CONDITION_DONT_CARE;				// Канал C
	conditions[1].channelD				= PS2000A_CONDITION_DONT_CARE;				// Канал D
	conditions[1].external				= PS2000A_CONDITION_DONT_CARE;				// Внешний
	conditions[1].aux					= PS2000A_CONDITION_DONT_CARE;				// Вспомогательный
	conditions[1].pulseWidthQualifier	= PS2000A_CONDITION_DONT_CARE;				// pwq
	conditions[1].digital				= PS2000A_CONDITION_TRUE;					// Цифровой

	memset(&pulseWidth, 0, sizeof(PWQ));

	// Установите цифровой триггер таким образом, чтобы он срабатывал при увеличении разряда 0 и увеличении разряда 4
	// Все необъявленные биты принимаются как PS2000A_DIGITAL_DONT_CARE

	digDirections[0].channel = PS2000A_DIGITAL_CHANNEL_0;
	digDirections[0].direction = PS2000A_DIGITAL_DIRECTION_RISING;

	digDirections[1].channel = PS2000A_DIGITAL_CHANNEL_4;
	digDirections[1].direction = PS2000A_DIGITAL_DIRECTION_HIGH;

	printf("\nCombination Block Triggered\n");
	printf("Collects when value rises past %d", scaleVoltages?
		adc_to_mv(sourceDetails.thresholdUpper, unit->channelSettings[PS2000A_CHANNEL_A].range, unit)	// При масштабировании напряжений выведите значение в мВ
		: sourceDetails.thresholdUpper);																	// в противном случае выведите количество АЦП
	
	printf(scaleVoltages?"mV\n" : "ADC Counts\n");

	printf("OR \n");
	printf("Digital Channel  0   --- Rising\n");
	printf("Digital Channel 4   --- High\n");
	printf("Other Digital Channels - Don't Care\n");

	printf("Press a key to start...\n");
	_getch();

	for (channel = 0; channel < unit->channelCount; channel++)
	{
		unit->channelSettings[channel].enabled = TRUE;
	}

	SetDefaults(unit);	// Включить аналоговые порты

	/* Активирован триггер
	 * Нарастающий фронт
	 * Порог = 1000 мВ */

	status = SetTrigger(unit, &sourceDetails, 1, conditions, 2, &directions, &pulseWidth, 0, 0, 0, digDirections, 2);

	if (status == PICO_OK)
	{

		BlockDataHandler(unit, "\nFirst 10 readings:\n", 0, MIXED, FALSE);
	}

	DisableAnalogue(unit);	// Отключите аналоговые порты по завершении работы
}

/****************************************************************************
* DigitalBlockTriggered
* Эта функция показывает, как собирать блок данных с цифровых портов
* при включенном запуске
*
* Возвращает значение none
***************************************************************************/

void DigitalBlockTriggered(UNIT * unit)
{
	PWQ pulseWidth;
	TRIGGER_DIRECTIONS directions;
	PS2000A_DIGITAL_CHANNEL_DIRECTIONS digDirections[2];		// Размер массива может достигать 16, запись для каждого цифрового бита

	PS2000A_TRIGGER_CONDITIONS conditions = {	PS2000A_CONDITION_DONT_CARE,		// Канал А
		PS2000A_CONDITION_DONT_CARE,		// Канал B
		PS2000A_CONDITION_DONT_CARE,		// Канал C
		PS2000A_CONDITION_DONT_CARE,		// Канал D
		PS2000A_CONDITION_DONT_CARE,		// Внешний
		PS2000A_CONDITION_DONT_CARE,		// Вспомогательный
		PS2000A_CONDITION_DONT_CARE,		// pwq
		PS2000A_CONDITION_TRUE};			// Цифровой


	printf("\nDigital Block Triggered\n");

	memset(&directions, 0, sizeof(TRIGGER_DIRECTIONS));
	memset(&pulseWidth, 0, sizeof(PWQ));

	printf("Collect block of data when the trigger occurs...\n");
	printf("Digital Channel  0   --- Rising\n");
	printf("Digital Channel  4   --- High\n");
	printf("Other Digital Channels - Don't Care\n");


	digDirections[0].channel = PS2000A_DIGITAL_CHANNEL_0;
	digDirections[0].direction = PS2000A_DIGITAL_DIRECTION_RISING;

	digDirections[1].channel = PS2000A_DIGITAL_CHANNEL_4;
	digDirections[1].direction = PS2000A_DIGITAL_DIRECTION_HIGH;


	if (SetTrigger(unit, NULL, 0, &conditions, 1, &directions, &pulseWidth, 0, 0, 0, digDirections, 2) == PICO_OK)
	{
		printf("Press a key to start...\n");
		_getch();
		BlockDataHandler(unit, "\nFirst 10 readings:\n", 0, DIGITAL, FALSE);
	}
}


/****************************************************************************
* DigitalBlockImmediate
* Эта функция показывает, как собирать блок данных с цифровых портов
* при отключенном запуске
*
* Возвращает значение none
***************************************************************************/
void DigitalBlockImmediate(UNIT *unit)
{
	PWQ pulseWidth;
	TRIGGER_DIRECTIONS directions;
	PS2000A_DIGITAL_CHANNEL_DIRECTIONS digDirections;

	printf("\nDigital Block Immediate\n");
	memset(&directions, 0, sizeof(TRIGGER_DIRECTIONS));
	memset(&pulseWidth, 0, sizeof(PWQ));
	memset(&digDirections, 0, sizeof(PS2000A_DIGITAL_CHANNEL_DIRECTIONS));

	SetTrigger(unit, NULL, 0, NULL, 0, &directions, &pulseWidth, 0, 0, 0, &digDirections, 0);

	printf("Press a key to start...\n");
	_getch();

	BlockDataHandler(unit, "\nFirst 10 readings:\n", 0, DIGITAL, FALSE);
}


/****************************************************************************
* DigitalStreamingAggregated
* эта функция демонстрирует, как собирать поток агрегированных данных
* с цифровых входов устройства (начните сбор немедленно)
***************************************************************************/
void DigitalStreamingAggregated(UNIT * unit)
{
	struct tPwq pulseWidth;
	struct tTriggerDirections directions;

	memset(&pulseWidth, 0, sizeof(struct tPwq));
	memset(&directions, 0, sizeof(struct tTriggerDirections));


	printf("Digital streaming with Aggregation...\n");
	printf("Press a key to start...\n");
	_getch();

	/* Trigger disabled	*/
	SetTrigger(unit, NULL, 0, NULL, 0, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	StreamDataHandler(unit, 0, AGGREGATED);
}


/****************************************************************************
* DigitalStreamingImmediate
* эта функция демонстрирует, как собирать поток данных
* с цифровых входов устройства (начните сбор немедленно)
***************************************************************************/
void DigitalStreamingImmediate(UNIT * unit)
{
	struct tPwq pulseWidth;
	struct tTriggerDirections directions;

	memset(&pulseWidth, 0, sizeof(struct tPwq));
	memset(&directions, 0, sizeof(struct tTriggerDirections));

	printf("Digital streaming...\n");
	printf("Press a key to start...\n");
	_getch();

	/* Триггер отключен	*/
	SetTrigger(unit, NULL, 0, NULL, 0, &directions, &pulseWidth, 0, 0, 0, 0, 0);

	StreamDataHandler(unit, 0, DIGITAL);
}


/****************************************************************************
* DigitalMenu 
* Отображает доступные цифровые примеры
* Параметры
* - указатель единфицы измерения на структуру единицы измерения
*
* Возвращает значение none
***************************************************************************/
void DigitalMenu(UNIT *unit)
{
	char ch;
	int16_t enabled = TRUE;
	int16_t disabled = !enabled;

	DisableAnalogue(unit);					// Отключить аналоговые порты
	SetDigitals(unit, enabled);		// Включить цифровые порты

	ch = ' ';
	while (ch != 'X')
	{
		printf("\n");
		printf(u8"\nМеню цифрового порта\n\n");
		printf(u8"B - Цифровой блок Немедленно\n");
		printf(u8"T - Сработал цифровой блок\n");
		printf(u8"A - Аналоговый 'И' Цифровой триггерный блок\n");
		printf(u8"O - Аналоговый 'ИЛИ' Цифровой триггерный блок\n");
		printf(u8"S - Режим цифровой потоковой передачи\n");
		printf(u8"V - Агрегированная цифровая потоковая передача\n");
		printf(u8"X - Возврат к предыдущему меню\n\n");
		printf(u8"Операция:");

		ch = toupper(_getch());

		printf("\n\n");
		switch (ch) 
		{
			case 'B':
				DigitalBlockImmediate(unit);
				break;

			case 'T':
				DigitalBlockTriggered(unit);
				break;

			case 'A':
				ANDAnalogueDigitalTriggered(unit);
				break;

			case 'O':
				ORAnalogueDigitalTriggered(unit);
				break;

			case 'S':
				DigitalStreamingImmediate(unit);
				break;

			case 'V':
				DigitalStreamingAggregated(unit);
				break;

			default:

				printf(u8"Недопустимый параметр.\n");
				break;
		}
	}

	SetDigitals(unit, disabled);				// Отключите цифровые порты, когда закончите
	RestoreAnalogueSettings(unit);
}

/****************************************************************************
* main
*
***************************************************************************/

int32_t main(void) {
	SetConsoleOutputCP(CP_UTF8);
	int8_t ch;

	PICO_STATUS status;
	UNIT unit;

	printf(u8"Пример программы-драйвера для PicoScope 2000 Series (A API)\n");
	printf(u8"Версия 2.3\n\n");
	printf(u8"\n\nОткрытие устройства...\n");

	status = OpenDevice(&unit);
	CollectStreamingTriggered(&unit);
	/*
	ch = ' ';

	while (ch != 'X')
	{
		DisplaySettings(&unit);

		printf("\n");
		printf(u8"B - Немедленный блок                          V - Установленные напряжения\n");
		printf(u8"T - Сработавший блок                          I - Установите временной интервал\n");
		printf(u8"E - Соберите блок данных с помощью ETS        A - Количество отсчетов АЦП/мВ\n");
		printf(u8"R - Соберите набор быстрых захватов           G - Генератор сигналов\n");
		printf(u8"S - Немедленная потоковая передача\n");
		printf(u8"W - Запущенная потоковая передача\n");
		printf(unit.digitalPorts? "D - Меню цифровых портов\n":"");
		printf(u8"                                              X - Выход\n\n");
		printf(u8"Операция:");

		ch = toupper(_getch());

		printf("\n\n");

		switch (ch)
		{
			case 'B':
				CollectBlockImmediate(&unit);
				break;

			case 'T':
				CollectBlockTriggered(&unit);
				break;

			case 'R':
				CollectRapidBlock(&unit);
				break;

			case 'S':
				CollectStreamingImmediate(&unit);
				break;

			case 'W':
				CollectStreamingTriggered(&unit);
				break;

			case 'E':
				CollectBlockEts(&unit);
				break;

			case 'G':
				SetSignalGenerator(unit);
				break;

			case 'V':
				SetVoltages(&unit);
				break;

			case 'I':
				SetTimebase(unit);
				break;

			case 'A':
				scaleVoltages = !scaleVoltages;
				break;

			case 'D':
				if (unit.digitalPorts)
					DigitalMenu(&unit);
				break;

			case 'X':
				break;

			default:
				printf(u8"Недопустимый параметр.\n");
				break;
		}
	}
	*/
	CloseDevice(&unit);

	return 0;
}
