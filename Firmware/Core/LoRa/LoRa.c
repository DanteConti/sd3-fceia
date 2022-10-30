#include "LoRa.h"

/* ====================== [DEFINICION DE REGISTROS] ============================ */
// Por ahora estoy haciendo las estructuras de los registros que pueden llegarse a leer/escribirse
// via SPI al módulo, pero no necesariamente usemos todas. Por lo pronto, servirá para el IntelliSense.

// RegOpMode
typedef union {
	struct {
		unsigned LongRangeMode :1;
		unsigned AccessSharedReg:1;
		unsigned :2;
		unsigned LowFrequencyModeOn:1;
		unsigned Mode:3;
	};
	uint8_t data;
} RegOpMode_t;

// RegPaConfig
typedef union {
	struct {
		unsigned PaSelect :1;
		unsigned MaxPower :3;
		unsigned OutputPower:4;
	};
	uint8_t data;
} RegPaConfig_t;

// RegPaRamp
typedef union {
	struct {
		unsigned :3;
		unsigned :1;
		unsigned PaRamp:4;
	};
	uint8_t data;
} RegPaRamp_t;

// RegOcp
typedef union {
	struct {
		unsigned :2;
		unsigned OcpOn:1;
		unsigned OcpTrim:5;
	};
	uint8_t data;
} RegOcp_t;

// RegLna
typedef union {
	struct {
		unsigned LnaGain:3;
		unsigned LnaBoostLf:2;
		unsigned :1;
		unsigned LnaBoostHf:2;
	};
	uint8_t data;
} RegLna_t;

typedef union {
	struct {
		unsigned RxTimeoutMask:1;
		unsigned RxDoneMask:1;
		unsigned PayloadCrcErrorMask:1;
		unsigned ValidHeaderMask:1;
		unsigned TxDoneMask:1;
		unsigned CadDoneMask:1;
		unsigned FhssChangeChannelMask:1;
		unsigned CadDetectedMask:1;
	};
	uint8_t data;
} RegIrqFlagsMask_t;

typedef union {
	struct {
		unsigned RxCodingRate :3;
		struct {
			unsigned ModemClear:1;
			unsigned HeaderInfoValid:1;
			unsigned RxOnGoing:1;
			unsigned SignalSynchronized:1;
			unsigned SignalDetected:1;
		}ModemStatus;
	};
	uint8_t data;
} RegModemStat_t;

typedef union {
	struct {
		unsigned PllTimeout:1;
		unsigned CrcOnPayload:1;
		unsigned FhssPresentChannel:5;
	};
	uint8_t data;
} RegHopChannel_t;

typedef union {
	struct {
		unsigned BW:4;
		unsigned CodingRate:3;
		unsigned InplicitHeaderOn:1;
	};
	uint8_t data;
} RegModemCfg1_t;

typedef union {
	struct {
		unsigned SpreadingFactor:4;
		unsigned TxContiunousMode:1;
		unsigned RxPayloadCrcOn:1;
		unsigned SymbTimeout:2;
	};
	uint8_t data;
} RegModemCfg2_t;

typedef union {
	struct {
		unsigned :4;
		unsigned LowDataRateOptimize:1;
		unsigned AgcAutoOn:1;
		unsigned :2;
	};
	uint8_t data;
} RegModemCfg3_t;

typedef union {
	struct {
		unsigned :4;
		unsigned FreqErrorMSB:4;
	};
	uint8_t data;
}RegFeiMSB_t;

typedef union {
	struct {
		unsigned :5;
		unsigned DetectionOptimize:3;
	};
	uint8_t data;
}RegDetectOptimize_t;

typedef union {
	struct {
		unsigned :1;
		unsigned InvertIQ:1;
		unsigned :6;
	};
	uint8_t data;
}RegInvertIQ_t;

/* ========================= [FIN DEFINICION REGISTROS] ========================== */

/* ========================= [FUNCIONES DRIVER] ================================= */

/*
 * Lee el registro ubicado en la dirección "addr" (un registro a la vez, 1 byte de longitud)
 *
 * Bloquea el hilo! Ojo con uso de interrupciones. En el mejor de los casos implementamos DMA
 * más adelante.
 *
 */
uint8_t LoRa_readRegBlocking(LoRa* loRa, uint8_t addr)
{
	uint8_t out;

	// Hacemos NSS bajo para seleccionar el chip asignado al LoRa
	HAL_GPIO_WritePin(loRa->NSS_port, loRa->NSS_pin, GPIO_PIN_RESET);

	/*
	 * Según el Datasheet SX1276-7-8, pag. 80: Interfaz SPI, tenemos que especificar si
	 * vamos a leer o escribir mediante el MSB del address. Si el primer bit que se manda es un 0,
	 * estamos leyendo. Sino, es escritura.
	 */

	// Asignamos 0 al MSB del address para indicar lectura
	addr &= 0x7F; // 01111111

	// Comenzamos transmisión del address
	HAL_SPI_Transmit(loRa->SPI_Handler, &addr, 1,TRANSFER_TIMEOUT);

	// Esperamos a que termine
	while(HAL_SPI_GetState(loRa->SPI_Handler) != HAL_SPI_STATE_READY);

	// Leemos el dato en el registro.
	HAL_SPI_Receive(loRa->SPI_Handler, &out, 1, TRANSFER_TIMEOUT);

	// Volvemos a esperar que termine
	while(HAL_SPI_GetState(loRa->SPI_Handler) != HAL_SPI_STATE_READY);

	// Finalizamos transferencia levantando el NSS
	HAL_GPIO_WritePin(loRa->NSS_port, loRa->NSS_pin, GPIO_PIN_SET);

	return out;
}

/*
 * Escribe el registro ubicado en la dirección "addr" (un registro a la vez, 1 byte de longitud)
 *
 * Bloquea el hilo! Ojo con uso de interrupciones. En el mejor de los casos implementamos DMA
 * más adelante.
 *
 */
void LoRa_writeRegBlocking(LoRa* loRa, uint8_t addr, uint8_t data)
{
	// Hacemos NSS bajo para seleccionar el chip asignado al LoRa
	HAL_GPIO_WritePin(loRa->NSS_port, loRa->NSS_pin, GPIO_PIN_RESET);

	/*
	 * Según el Datasheet SX1276-7-8, pag. 80: Interfaz SPI, tenemos que especificar si
	 * vamos a leer o escribir mediante el MSB del address. Si el primer bit que se manda es un 0,
	 * estamos leyendo. Sino, es escritura.
	 */

	// Asignamos 1 al MSB del address para indicar escritura
	addr |= 0x80; // 10000000

	// Comenzamos transmisión del address
	HAL_SPI_Transmit(loRa->SPI_Handler, &addr, 1,TRANSFER_TIMEOUT);

	// Esperamos a que termine
	while(HAL_SPI_GetState(loRa->SPI_Handler) != HAL_SPI_STATE_READY);

	// Escribimos el dato en el registro.
	HAL_SPI_Transmit(loRa->SPI_Handler, &data, 1, TRANSFER_TIMEOUT);

	// Volvemos a esperar que termine
	while(HAL_SPI_GetState(loRa->SPI_Handler) != HAL_SPI_STATE_READY);

	// Finalizamos transferencia levantando el NSS
	HAL_GPIO_WritePin(loRa->NSS_port, loRa->NSS_pin, GPIO_PIN_SET);
}


/*
 * Escribe el registro ubicado en la dirección "addr" ("length" bytes de longitud)
 *
 * Útil para escribir en la cola "FIFO" del módulo para la transmisión de datos con
 * mayor longitud.
 */
void LoRa_burstWriteBlocking(LoRa* loRa, uint8_t addr, uint8_t* data, uint8_t length)
{
	// Hacemos NSS bajo para seleccionar el chip asignado al LoRa
	HAL_GPIO_WritePin(loRa->NSS_port, loRa->NSS_pin, GPIO_PIN_RESET);

	// Asignamos 1 al MSB del address para indicar escritura
	addr |= 0x80; // 10000000

	// Comenzamos transmisión del address
	HAL_SPI_Transmit(loRa->SPI_Handler, &addr, 1,TRANSFER_TIMEOUT);

	// Esperamos a que termine
	while(HAL_SPI_GetState(loRa->SPI_Handler) != HAL_SPI_STATE_READY);

	// Escribimos los datos en el registro.
	HAL_SPI_Transmit(loRa->SPI_Handler, data, length, TRANSFER_TIMEOUT);

	// Volvemos a esperar que termine
	while(HAL_SPI_GetState(loRa->SPI_Handler) != HAL_SPI_STATE_READY);

	// Finalizamos transferencia levantando el NSS
	HAL_GPIO_WritePin(loRa->NSS_port, loRa->NSS_pin, GPIO_PIN_SET);
}


void LoRa_setMode(LoRa* loRa, LoRa_Mode mode)
{
	RegOpMode_t currentMode;
	currentMode.data = LoRa_readRegBlocking(loRa, REG_OP_MODE);

	currentMode.Mode = mode;

	LoRa_writeRegBlocking(loRa, REG_OP_MODE, currentMode.data);

	loRa->currentMode = mode;
}


void LoRa_enableLongRange(LoRa* loRa){
	RegOpMode_t currentMode;
	currentMode.data = LoRa_readRegBlocking(loRa, REG_OP_MODE);

	currentMode.LongRangeMode = 1u;
	LoRa_writeRegBlocking(loRa, REG_OP_MODE, currentMode.data);
}

/*
 * Setea la frecuencia (EN HERTZ) de trabajo para el módulo LoRa.
 * Ejemplo de llamado para 433MHz:
 * 	LoRa_setFrequency(loRa, 433e6);
 */
void LoRa_setFrequency(LoRa* loRa, long freq)
{
	// Según pág 109 del datasheet, Frf = freq * 2^19 / F(XOSC).
	// Multiplicar por 2^19 es correr 19 bits a la izq, F(XOSC) es = 32MHz x defecto.
	uint64_t F = ((uint64_t)freq << 19) / 32E6;

	LoRa_writeRegBlocking(loRa, REG_FR_MSB, (uint8_t)(F>>16));
	LoRa_writeRegBlocking(loRa, REG_FR_MID, (uint8_t)(F>>8));
	LoRa_writeRegBlocking(loRa, REG_FR_LSB, (uint8_t)(F>>0));
}

void LoRa_setPower(LoRa* loRa, LoRa_Power_Gain gain)
{
	LoRa_writeRegBlocking(loRa, REG_PA_CFG, (uint8_t)gain);
	HAL_Delay(10);
	loRa->powerGain = gain;
}

/*
 * Setea protección a sobrecorriente en la cantidad de milliamperes especificado en "current"
 */
void LoRa_setOCP(LoRa* loRa, uint8_t current)
{
	uint8_t OcpTrim = 0;

	// Aseguro que "current" esté entre los valores posibles.
	if(current <45) current = 45;
	if(current > 240) current = 240;

	// Calculo según fórmula en página 85 del datasheet
	if(current<=120)
	{
		OcpTrim = (current-45)/5;
	}
	else
	{
		OcpTrim = (current + 30)/10;
	}

	RegOcp_t OCPReg;
	OCPReg.OcpOn = 1;
	OCPReg.OcpTrim = OcpTrim;

	LoRa_writeRegBlocking(loRa, REG_OCP, OCPReg.data);
	HAL_Delay(10);
	loRa->OCPmilliamps = current;
}

/*
 * Habilita (o no) el boost del amplificador interno del módulo para la recepción.
 */
void LoRa_enableLNABoost(LoRa* loRa, uint8_t enabled){
	RegLna_t RegLna;
	RegLna.data = LoRa_readRegBlocking(loRa, REG_LNA);

	if(enabled == 1)
		RegLna.LnaBoostHf = 3u; // 11 = boost on, 150% LNA current
	else
		RegLna.LnaBoostHf = 0;

	// Maximum Gain
	RegLna.LnaGain = 1u;
	LoRa_writeRegBlocking(loRa, REG_LNA, RegLna.data);
	HAL_Delay(10);
}

void LoRa_setSpreadingFactor(LoRa* loRa, LoRa_SpreadingFactor spreadingFactor)
{
	RegModemCfg2_t RegModemCfg2;
	RegModemCfg2.data = LoRa_readRegBlocking(loRa, REG_MODEM_CFG2);

	RegModemCfg2.SpreadingFactor = spreadingFactor;
	LoRa_writeRegBlocking(loRa, REG_MODEM_CFG2, RegModemCfg2.data);
	loRa->spreadingFactor = spreadingFactor;
}

void LoRa_enableCRC(LoRa* loRa, uint8_t enabled){
	RegModemCfg2_t RegModemCfg2;
	RegModemCfg2.data = LoRa_readRegBlocking(loRa, REG_MODEM_CFG2);

	if(enabled == 1){
		RegModemCfg2.RxPayloadCrcOn = 1;
	}else{
		RegModemCfg2.RxPayloadCrcOn = 0;
	}
	LoRa_writeRegBlocking(loRa, REG_MODEM_CFG2, RegModemCfg2.data);
}

/*
 * Para evitar leer/escribir multiples veces el mismo registro, se usa
 * este método para inicializar el registro Modem_CFG2.
 *
 * Acá se setean el CRC, Spreading Factor, y el los MSB del timeout.
 *
 * Por defecto, se habilita la generación de CRC, y se da el máximo TimeOut.
 */
void LoRa_initializeModemCFG2(LoRa* loRa){
	RegModemCfg2_t RegModemCfg2;
	RegModemCfg2.data = LoRa_readRegBlocking(loRa, REG_MODEM_CFG2);

	RegModemCfg2.SpreadingFactor = loRa->spreadingFactor;
	RegModemCfg2.RxPayloadCrcOn = 1;

	// Máximo timeout.
	RegModemCfg2.SymbTimeout = 3;

	LoRa_writeRegBlocking(loRa, REG_MODEM_CFG2, RegModemCfg2.data);
}

void LoRa_initializeTimeoutLSB(LoRa* loRa)
{
	// Máximo timeout.
	uint8_t RegSymbTimeoutLsb = 0xFF;
	LoRa_writeRegBlocking(loRa, REG_SYMB_TIMEOUT_L, RegSymbTimeoutLsb);
}

/*
 * Inicializa el módulo LoRa con las configuraciones de la estructura.
 * Para obtener las configuraciones por defecto, use LoRa_getDefault().
 *
 * Más allá de las configuraciones por defecto, esta función habilita máximo
 * timeout de paquetes, BoostLNA y generación de CRC.
 */
void LoRa_Init(LoRa* loRa){
	// Antes de cambiar cualquier configuración inicial, hemos de poner el módulo en modo sleep
	LoRa_setMode(loRa, LoRa_SleepMode);
	HAL_Delay(10);

	LoRa_setFrequency(loRa, loRa->frequency);
	LoRa_setPower(loRa, loRa->powerGain);
	LoRa_setOCP(loRa, loRa->OCPmilliamps);

	// En teoría, en Reset ya está con ganancia más alta,
	// pero el boost no está habilitado. En el caso que sea de un consumo muy
	// alto, deshabilitamos el boost.
	LoRa_enableLNABoost(loRa, 1);

	// Inicializamos registros que necesitemos que tengan otros valores
	// que no son por defecto
	LoRa_initializeModemCFG2(loRa);
	LoRa_initializeTimeoutLSB(loRa);

	// Mandar a modo standby
	LoRa_setMode(loRa, LoRa_StandbyMode);

	// TODO: Chequear que esté todo OK haciendo un llamado al registro "RegVersion" y comparar con 0x12.
}

/*
 * Devuelve la configuración por defecto del módulo LoRa.
 * NO CONFIGURA HANDLER SPI NI PINES POR DEFECTO
 * (Esos se deben setear antes de llamar a LoRa_Init())
 *
 * Configura:
 * 	- Frecuencia: 433MHz
 * 	- SpreadingFactor: 128 chips/s
 * 	- Ganancia : 20dB
 * 	- Overcurrent: 100 mA
 */
LoRa LoRa_getDefault()
{
	LoRa loRa;

	loRa.frequency = 433E6;
	loRa.powerGain = Power_20DB;
	loRa.spreadingFactor = SF_128;
	loRa.OCPmilliamps = 100;

	return loRa;
}

/*
 * Manda el dato a la cola FIFO del módulo para su transmisión.
 * Bloquea la ejecución hasta "timeout" millisegundos, debido a que se fija (por polling)
 * la interrupción de TxDone (lo cual es malo, pero sirve para probar).
 * Devuelve 1 si pudo mandarlo, 0 si llegó al timeout.
 *
 * Al finalizar, vuelve al modo standby.
 */
uint8_t LoRa_transmitBlocking(LoRa* loRa, uint8_t* data, uint8_t length, uint16_t timeout)
{
	LoRa_setMode(loRa, LoRa_StandbyMode);

	uint8_t RegFiFoTxBaseAddr = LoRa_readRegBlocking(loRa, REG_FIFO_TX_BASE_ADDR);

	// Muevo el puntero de la cola al asignado para transmisión
	LoRa_writeRegBlocking(loRa, REG_FIFO_ADDR_PTR, RegFiFoTxBaseAddr);

	// Asigno la longitud del paquete
	LoRa_writeRegBlocking(loRa, REG_PAYLOAD_LENGTH, length);

	// Escribo el paquete en la cola
	LoRa_burstWriteBlocking(loRa, REG_FIFO, data,length);

	LoRa_setMode(loRa, LoRa_TxMode);
	// Esto se puede optimizar reusando el RegFiFoTxBaseAddr,
	// pero de esta forma es más legible. En caso de necesidad,
	// podemos sacar 1 byte más de memoria.
	uint8_t RegIRQFlags;

	// POLLING DE INTERRUPCIÓN = MALO
	// Más adelante habría que setear la máscara de TxDoneMask en el registro
	// de RegIRQFlagsMask y que DIO0 sea TxDone para que levante la interrupción en ese
	// pin, en vez de estar haciendo polling (no interrumpe al procesador).
	while(1){
		RegIRQFlags = LoRa_readRegBlocking(loRa, REG_IRQ_FLAGS);
		if((RegIRQFlags & 0x08) != 0) { // 0x08 : TxDoneMask
			// Limpio la interrupción
			LoRa_writeRegBlocking(loRa, REG_IRQ_FLAGS, 0xFF);
			// Vuelvo a standby
			LoRa_setMode(loRa, LoRa_StandbyMode);
			return 1;
		}else{
			if(--timeout == 0){
				LoRa_setMode(loRa, LoRa_StandbyMode);
				return 0;
			}
		}
		HAL_Delay(1);
	}
}
