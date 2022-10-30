#ifndef LORA_LORA_H_
#define LORA_LORA_H_

#include "main.h"

// Límite entre transferencias
#define TRANSFER_TIMEOUT 2000

/* ------------- REGISTROS --------------- */
/*
 * La página 108 del datasheet del SX1276 mapea los registros
 * que necesitamos para transmitir/recibir por LoRa, además de la
 * descripción.
 */
#define REG_FIFO					0x00

// Comunes
#define REG_OP_MODE					0x01
#define REG_FR_MSB					0x06
#define REG_FR_MID					0x07
#define REG_FR_LSB					0x08

// Transmisor
#define REG_PA_CFG					0x09
#define REG_OCP						0x0B

// Receptor
#define REG_LNA						0x0C
#define REG_FIFO_ADDR_PTR			0x0D
#define REG_FIFO_TX_BASE_ADDR		0x0E
#define REG_FIFO_RX_BASE_ADDR		0x0F
#define REG_FIFO_RX_CURR_ADDR		0x10
#define REG_IRQ_FLAGS				0x12
#define REG_RX_NB_BYTES				0x13
#define REG_PKT_RSSI_VALUE			0x1A
#define	REG_MODEM_CFG1				0x1D
#define REG_MODEM_CFG2				0x1E
#define REG_SYMB_TIMEOUT_L			0x1F
#define REG_PREAMBLE_MSB			0x20
#define REG_PREAMBLE_LSB			0x21
#define REG_PAYLOAD_LENGTH			0x22

// IO Control
#define REG_DIO_MAP1				0x40
#define REG_DIO_MAP2				0x41

// Version
#define REG_VERSION					0x42

/* ------ Estados ----------- */

typedef enum {
	LoRa_SleepMode = 0u,
	LoRa_StandbyMode = 1u,
	LoRa_TxMode = 3u,
	LoRa_RxContinuousMode = 5u,
	LoRa_RxSingleMode = 6u
} LoRa_Mode;

/* ------ Ganancia ---------- */

// Estos valores salen del siguiente repo: https://github.com/SMotlaq/LoRa/blob/master/LoRa/LoRa.h
// Habría que ver bien el cálculo, que se encuentra en el datasheet pag 94, así usamos el registro como se debe.
// Es cuestión de ver si uso (o no) el PA output pin (que es para la transmisión).
typedef enum {
	Power_11DB = 0xF6,
	Power_14DB = 0xF9,
	Power_17DB = 0xFC,
	Power_20DB = 0xFF
} LoRa_Power_Gain;

/* ------- Spreading Factor ------- */
typedef enum {
	SF_64 = 6u,
	SF_128,
	SF_256,
	SF_512,
	SF_1024,
	SF_2048,
	SF_4096
} LoRa_SpreadingFactor;

typedef struct LoRa_Settings
{

	// Hardware Definitions
	GPIO_TypeDef*      NSS_port;
	uint16_t		   NSS_pin;

	GPIO_TypeDef*      RST_port;
	uint16_t		   RST_pin;

	GPIO_TypeDef*      DIO0_port;
	uint16_t		   DIO0_pin;

	// SPI
	SPI_HandleTypeDef* SPI_Handler;

	// Current Mode
	LoRa_Mode		   currentMode;

	// Settings
	long frequency;
	LoRa_Power_Gain powerGain;
	uint8_t OCPmilliamps; // OverCurrentProtection
	LoRa_SpreadingFactor spreadingFactor;

} LoRa;



LoRa LoRa_getDefault();
void LoRa_init(LoRa* _LoRa);
void LoRa_reset(LoRa* _LoRa);
uint8_t LoRa_readRegBlocking(LoRa* loRa, uint8_t addr);
void LoRa_setSpreadingFactor(LoRa* loRa, LoRa_SpreadingFactor spreadingFactor);
void LoRa_enableLNABoost(LoRa* loRa, uint8_t enabled);
void LoRa_enableCRC(LoRa* loRa, uint8_t enabled);
void LoRa_setOCP(LoRa* loRa, uint8_t current);
void LoRa_setPower(LoRa* loRa, LoRa_Power_Gain gain);
void LoRa_writeRegBlocking(LoRa* loRa, uint8_t addr, uint8_t data);
void LoRa_burstWriteBlocking(LoRa* loRa, uint8_t addr, uint8_t* data, uint8_t length);
uint8_t LoRa_transmitBlocking(LoRa* loRa, uint8_t* data, uint8_t length, uint16_t timeout);

#endif /* LORA_LORA_H_ */
