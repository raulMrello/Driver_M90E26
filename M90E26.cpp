/*
 * M90E26.cpp
 *
 *  Created on: Feb 2018
 *      Author: raulMrello
 *
 */
 
#include "M90E26.h"


//------------------------------------------------------------------------------------
//-- PRIVATE TYPEDEFS ----------------------------------------------------------------
//------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------
/** Comandos */
#define SoftReset 	0x00 	//Software Reset
#define SysStatus 	0x01 	//System Status
#define FuncEn 		0x02 	//Function Enable
#define SagTh 		0x03 	//Voltage Sag Threshold
#define SmallPMod 	0x04 	//Small-Power Mode
#define LastData 	0x06 	//Last Read/Write SPI/UART Value
#define LSB 		0x08 	//RMS/Power 16-bit LSB
#define CalStart 	0x20 	//Calibration Start Command
#define PLconstH 	0x21 	//High Word of PL_Constant
#define PLconstL 	0x22 	//Low Word of PL_Constant
#define Lgain 		0x23 	//L Line Calibration Gain
#define Lphi 		0x24 	//L Line Calibration Angle
#define Ngain 		0x25 	//N Line Calibration Gain
#define Nphi 		0x26 	//N Line Calibration Angle
#define PStartTh 	0x27 	//Active Startup Power Threshold
#define PNolTh 		0x28 	//Active No-Load Power Threshold
#define QStartTh 	0x29 	//Reactive Startup Power Threshold
#define QNolTh 		0x2A 	//Reactive No-Load Power Threshold
#define MMode 		0x2B 	//Metering Mode Configuration
#define CSOne 		0x2C 	//Checksum 1
#define AdjStart 	0x30 	//Measurement Calibration Start Command
#define Ugain 		0x31 	//Voltage rms Gain
#define IgainL 		0x32 	//L Line Current rms Gain
#define IgainN 		0x33 	//N Line Current rms Gain
#define Uoffset 	0x34 	//Voltage Offset
#define IoffsetL 	0x35 	//L Line Current Offset
#define IoffsetN 	0x36 	//N Line Current Offse
#define PoffsetL 	0x37 	//L Line Active Power Offset
#define QoffsetL 	0x38 	//L Line Reactive Power Offset
#define PoffsetN 	0x39 	//N Line Active Power Offset
#define QoffsetN 	0x3A 	//N Line Reactive Power Offset
#define CSTwo 		0x3B 	//Checksum 2
#define APenergy 	0x40 	//Forward Active Energy
#define ANenergy 	0x41 	//Reverse Active Energy
#define ATenergy 	0x42 	//Absolute Active Energy
#define RPenergy 	0x43 	//Forward (Inductive) Reactive Energy
#define Rnenerg 	0x44 	//Reverse (Capacitive) Reactive Energy
#define Rtenergy 	0x45 	//Absolute Reactive Energy
#define EnStatus 	0x46 	//Metering Status
#define Irms 		0x48 	//L Line Current rms
#define Urms 		0x49 	//Voltage rms
#define Pmean 		0x4A 	//L Line Mean Active Power
#define Qmean 		0x4B 	//L Line Mean Reactive Power
#define Freq 		0x4C 	//Voltage Frequency
#define PowerF 		0x4D 	//L Line Power Factor
#define Pangle 		0x4E 	//Phase Angle between Voltage and L Line Current
#define Smean 		0x4F 	//L Line Mean Apparent Power
#define IrmsTwo 	0x68 	//N Line Current rms
#define PmeanTwo 	0x6A 	//N Line Mean Active Power
#define QmeanTwo 	0x6B 	//N Line Mean Reactive Power
#define PowerFTwo 	0x6D 	//N Line Power Factor
#define PangleTwo 	0x6E 	//Phase Angle between Voltage and N Line Current
#define SmeanTwo 	0x6F 	//N Line Mean Apparent Power

//------------------------------------------------------------------------------------
/** Errores */
#define SYS_CAL_ERR		0xC0
#define SYS_ADJ_ERR		0x30
//------------------------------------------------------------------------------------
static const uint8_t energy_regs[]  = {
	APenergy,
	ANenergy,
	ATenergy,
	RPenergy,
	Rnenerg,
	Rtenergy,
	EnStatus
};

//------------------------------------------------------------------------------------
static const uint8_t measure_regs[] = {
	Irms,
	Urms,
	Pmean,
	Qmean,
	Freq,
	PowerF,
	Pangle,
	Smean,
	IrmsTwo,
	PmeanTwo,
	QmeanTwo,
	PowerFTwo,
	PangleTwo,
	SmeanTwo
};

//------------------------------------------------------------------------------------
static const uint8_t meter_cal[]  = {
	CalStart,
	//LSB,
	PLconstH,
	PLconstL,
	Lgain,
	Lphi,
	Ngain,
	Nphi,
	PStartTh,
	PNolTh,
	QStartTh,
	QNolTh,
	MMode,
	CSOne
};

//------------------------------------------------------------------------------------
static const uint8_t meas_cal[]  = {
	AdjStart,
	Ugain,
	IgainL,
	IgainN,
	Uoffset,
	IoffsetL,
	IoffsetN,
	PoffsetL,
	QoffsetL,
	PoffsetN,
	QoffsetN,
	CSTwo
};


//------------------------------------------------------------------------------------
#define ENERGY_REG_COUNT		sizeof(energy_regs)/sizeof(energy_regs[0])
#define MEASUREMENT_REG_COUNT	sizeof(measure_regs)/sizeof(measure_regs[0])
#define METERCAL_REG_COUNT		sizeof(meter_cal)/sizeof(meter_cal[0])
#define MEASCAL_REG_COUNT		sizeof(meas_cal)/sizeof(meas_cal[0])

//------------------------------------------------------------------------------------
/** Macro para imprimir trazas de depuración, siempre que se haya configurado un objeto
 *	Logger válido (ej: _debug)
 */

static const char* _MODULE_ = "[M90E26]........";
#define _EXPR_	(_defdbg && !IS_ISR())


//------------------------------------------------------------------------------------
//-- STATIC METHODS IMPLEMENTATION ---------------------------------------------------
//------------------------------------------------------------------------------------

/** Calcula el checksum de un array de N valores */
static uint16_t checksum(uint16_t *data, int size){
	uint8_t lcrc = 0;
	uint8_t hcrc = 0;
	for(int i=0; i<size; i++){
		uint8_t lb = (uint8_t)(data[i] & 0xff);
		uint8_t hb = (uint8_t)((data[i] >> 8) & 0xff);
		lcrc += hb;
		lcrc += lb;
		hcrc ^= hb;
		hcrc ^= lb;
	}
	uint16_t crc = (((uint16_t)hcrc) << 8) + lcrc;
	return crc;
}


//------------------------------------------------------------------------------------
//-- PUBLIC METHODS IMPLEMENTATION ---------------------------------------------------
//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------
M90E26::M90E26(PinName p_tx, PinName p_rx, uint32_t hz, PinName p_rst, PinName p_irq, uint32_t recv_timed_ms, bool defdbg) : AMDriver("M90E26", 1) {
	_defdbg = defdbg;
	_spi = NULL;
	_out_cs = NULL;
	_out_rst = NULL;
	_out_clk = NULL;
	_iin_irq = NULL;
	_ready = false;
	_recv_timed_ms = recv_timed_ms;

	DEBUG_TRACE_I(_EXPR_, _MODULE_, "Iniciando driver M90E26");
	_serial = new Serial(p_tx, p_rx, 256, 9600, 0, UART_NUM_1, false, UART_HW_FLOWCTRL_DISABLE, NC, NC, UART_DATA_8_BITS, UART_PARITY_DISABLE, UART_STOP_BITS_1);
	MBED_ASSERT(_serial);
	if(gpio_pullup_en(p_rx) != ESP_OK){
		DEBUG_TRACE_E(_EXPR_, _MODULE_, "ERR_RX no se puede activar pullup");
	}


	_out_cs = NULL;
	_out_clk = NULL;

	if(p_irq != NC){
		_iin_irq = new InterruptIn(p_irq);
		MBED_ASSERT(_iin_irq);
		_iin_irq->rise(NULL);
		_iin_irq->fall(NULL);
	}

	if(p_rst != NC){
		_out_rst = new DigitalOut(p_rst);
		MBED_ASSERT(_out_rst);
		_out_rst->write(1);
		wait_us(500);
		_out_rst->write(1);
	}
}


//------------------------------------------------------------------------------------
M90E26::M90E26(Serial* serial, PinName p_rst, PinName p_irq, uint32_t recv_timed_ms, bool defdbg) : AMDriver("M90E26", 1) {
	_defdbg = defdbg;
	_spi = NULL;
	_out_cs = NULL;
	_out_rst = NULL;
	_out_clk = NULL;
	_iin_irq = NULL;
	_ready = false;
	_recv_timed_ms = recv_timed_ms;

	DEBUG_TRACE_I(_EXPR_, _MODULE_, "Iniciando driver M90E26");
	_serial = serial;
	MBED_ASSERT(_serial);

	_out_cs = NULL;
	_out_clk = NULL;

	if(p_irq != NC){
		_iin_irq = new InterruptIn(p_irq);
		MBED_ASSERT(_iin_irq);
		_iin_irq->rise(NULL);
		_iin_irq->fall(NULL);
	}

	if(p_rst != NC){
		_out_rst = new DigitalOut(p_rst);
		MBED_ASSERT(_out_rst);
		_out_rst->write(1);
	}
}


//------------------------------------------------------------------------------------
M90E26::M90E26(SPI* spi, PinName p_rst, PinName p_irq, bool defdbg) : AMDriver("M90E26", 1) {
	_defdbg = defdbg;
	_serial = NULL;
	_out_cs = NULL;
	_out_clk = NULL;
	_out_rst = NULL;
	_iin_irq = NULL;
	_ready = false;
	_recv_timed_ms = DefaultReceptionTimeout;

	DEBUG_TRACE_I(_EXPR_, _MODULE_, "Iniciando driver M90E26");
	_spi = spi;
	MBED_ASSERT(_spi);

	if(p_irq != NC){
		_iin_irq = new InterruptIn(p_irq);
		MBED_ASSERT(_iin_irq);
		_iin_irq->rise(NULL);
		_iin_irq->fall(NULL);
	}

	if(p_rst != NC){
		_out_rst = new DigitalOut(p_rst);
		MBED_ASSERT(_out_rst);
		_out_rst->write(1);
	}
}

//------------------------------------------------------------------------------------
void M90E26::initEnergyIC(uint16_t* meter_cal, uint8_t meter_cal_count, uint16_t* meas_cal, uint8_t meas_cal_count){
	uint16_t sys_stat;
	int32_t rc;
	bool rdy = true;

	if(_out_rst){
		_out_rst->write(0);
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "RST hardware para habilitar ZX");
		Thread::wait(10);
		_out_rst->write(1);
	}

	// espera de 20ms para iniciar transacción
	Thread::wait(20);

	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Enviando RST");
 	//Perform soft reset
	if((rc = write(SoftReset, 0x789A)) < 0){
		return;
	}

	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Enviando Ajuste warnout, irq, thres. De momento sin uso.");
	//Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
	if((rc = write(FuncEn, 0/*0x0030*/)) < 0){
		return;
	}
//	//Voltage sag threshhold
//	if((rc = write(SagTh, 0x1F2F)) < 0){
//		return;
//	}

	// chequea si hay parámetros de calibración del medidor
	if((meter_cal && meter_cal_count)){
		//Set metering calibration values
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Enviando Calibración del medidor");
		setMeterCalib(meter_cal, meter_cal_count, 0);

		uint16_t crc = 0;
		//Checksum 1. Needs to be calculated based off the above values.
		if(read(CSOne, &crc) < 0){
			return;
		}
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Checksum1 = %x", crc);
	}

	// chequea si hay parámetros de calibración del medidor
	if((meas_cal && meas_cal_count)){
		//Set measurement calibration values
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Enviando Calibración de medidas");
		setMeasureCalib(meas_cal, meas_cal_count, 0);


		//Checksum 2. Needs to be calculated based off the above values.
		uint16_t crc = 0;
		if(read(CSTwo, &crc) < 0){
			return;
		}
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Checksum2 = %x", crc);
	}

	//Checks correctness of 21-2B registers and starts normal metering if ok
	if((rc = write(CalStart, 0x8765)) < 0){
		return;
	}
	//Checks correctness of 31-3A registers and starts normal measurement  if ok
	if((rc = write(AdjStart, 0x8765)) < 0){
		return;
	}

	if(getSysStatus(&sys_stat) < 0){
		return;
	}

	DEBUG_TRACE_D(_EXPR_, _MODULE_, "Leyendo estado = %x", (uint16_t)sys_stat);

	if ((sys_stat & 0xC000) != 0){
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "ERR_CHECKSUM_1");
		rdy = false;
	}
	if ((sys_stat & 0x3000) != 0){
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "ERR_CHECKSUM_2");
		rdy = false;
	}
	_ready = rdy;
	DEBUG_TRACE_D(_EXPR_, _MODULE_, "READY!");
}


//------------------------------------------------------------------------------------
int32_t M90E26::autoCalibration(double max_curr, Callback<void()>turnOnLoad, Callback<void()>turnOffLoad, uint16_t* result, int result_size){
	if(!_calibStart())
		return -1;
	if(!_calibPLconst())
		return -2;
	if(!_calibSmallPowerOffset())
		return -3;
	if(!_calibGain())
		return -4;
	if(!_calibLAngle())
		return -5;
	if(!_calibTurnOnPower())
		return -6;
	if(!_calibUIgains())
		return -7;
	if(!_calibEnd())
		return -8;
	return _calibGetParams(result, result_size);
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getSysStatus(uint16_t* pdata){
	uint16_t data = 0;
	int32_t rc = read(SysStatus, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = data;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getMeterStatus(uint16_t* pdata){
	uint16_t data = 0;
	int32_t rc = read(EnStatus, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = data;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getLineVoltage(double* pdata){
	uint16_t data = 0;
	int32_t rc = read(Urms, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)data/100;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getLineCurrent(double* pdata){
	uint16_t data = 0;
	int32_t rc = read(Irms, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)data/1000;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getActivePower(double* pdata){
	uint16_t data = 0;
	int32_t rc = read(Pmean, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)((int16_t)data)/1000;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getReactivePower(double* pdata){
	uint16_t data = 0;
	int32_t rc = read(Qmean, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)((int16_t)data)/1000;
	return 0;
}

//------------------------------------------------------------------------------------
int32_t M90E26::getFrequency(double* pdata){
	uint16_t data = 0;
	int32_t rc = read(Freq, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)data/100;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getPowerFactor(double* pdata){
	uint16_t data = 0;
	int32_t rc = read(PowerF, &data);
	if(rc < 0){
		return rc;
	}
	int16_t ival = (int16_t)data;
	if(ival & 0x8000){
		ival = (ival & 0x7FFF) * -1;
	}
	*pdata = (double)ival/1000;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getImportEnergy(double* pdata){
	//Register is cleared after reading
	//returns kWh if PL constant set to 1000imp/kWh
	uint16_t data = 0;
	int32_t rc = read(APenergy, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)data * 0.0001;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getExportEnergy(double* pdata){
	//Register is cleared after reading
	//returns kWh if PL constant set to 1000imp/kWh
	uint16_t data = 0;
	int32_t rc = read(ANenergy, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)data * 0.0001;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getPhase(double* pdata){
	//Register is cleared after reading
	//returns angle
	uint16_t data = 0;
	int32_t rc = read(Pangle, &data);
	if(rc < 0){
		return rc;
	}
	int16_t ival = (int16_t)data;
	if(ival & 0x8000){
		ival = (ival & 0x7FFF) * -1;
	}
	*pdata = (double)ival/10;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getMeanAparentPower(double* pdata){
	uint16_t data = 0;
	int32_t rc = read(Smean, &data);
	if(rc < 0){
		return rc;
	}
	*pdata = (double)data/1000;
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getElectricParams(ElectricParams& eparams, uint32_t& keys, uint8_t analyzer){
	if(analyzer >= _num_analyzers){
		return -1;
	}
	double pdata;
	keys = ElecKey_None;
	if(getLineVoltage(&pdata)==0){
		eparams.voltage = pdata;
		keys |= ElecKey_Voltage;
	}
	if(getLineCurrent(&pdata)==0){
		eparams.current = pdata;
		keys |= ElecKey_Current;
	}
	if(getActivePower(&pdata)==0){
		eparams.aPow = pdata;
		keys |= ElecKey_ActivePow;
	}
	if(getReactivePower(&pdata)==0){
		eparams.rPow = pdata;
		keys |= ElecKey_ReactivePow;
	}
	if(getMeanAparentPower(&pdata)==0){
		eparams.mPow = pdata;
		keys |= ElecKey_ApparentPow;
	}
	if(getPowerFactor(&pdata)==0){
		eparams.pFactor = pdata;
		keys |= ElecKey_PowFactor;
	}
	if(getPhase(&pdata)==0){
		eparams.phase = pdata;
		keys |= ElecKey_Phase;
	}
	if(getFrequency(&pdata)==0){
		eparams.freq = pdata;
		keys |= ElecKey_Frequency;
	}
	if(getImportEnergy(&pdata)==0){
		eparams.aEnergy = pdata;
		keys |= ElecKey_ActiveEnergy;
	}
	if(getExportEnergy(&pdata)==0){
		eparams.rEnergy = pdata;
		keys |= ElecKey_ReactiveEnergy;
	}
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getEnergyData(uint16_t* pdata, uint8_t count){
	if(count < ENERGY_REG_COUNT)
		return -1;

	for(uint8_t i = 0; i<ENERGY_REG_COUNT; i++){
		uint16_t data = 0;
		int32_t rc = read(energy_regs[i], &data);
		if(rc < 0){
			return rc;
		}
		pdata[i] = data;
	}
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getMeasurementData(uint16_t* pdata, uint8_t count){
	if(count < MEASUREMENT_REG_COUNT)
		return -1;

	for(uint8_t i = 0; i<MEASUREMENT_REG_COUNT; i++){
		uint16_t data = 0;
		int32_t rc = read(measure_regs[i], &data);
		if(rc < 0){
			return rc;
		}
		pdata[i] = data;
	}
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getMeterCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer){
	if(count < METERCAL_REG_COUNT || analyzer >= _num_analyzers)
		return -1;

	for(uint8_t i = 0; i<METERCAL_REG_COUNT; i++){
		uint16_t data = 0;
		int32_t rc = read(meter_cal[i], &data);
		if(rc < 0){
			return rc;
		}
		pdata[i] = data;
	}
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::getMeasureCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer){
	if(count < METERCAL_REG_COUNT || analyzer >= _num_analyzers)
		return -1;

	for(uint8_t i = 0; i<MEASCAL_REG_COUNT; i++){
		uint16_t data = 0;
		int32_t rc = read(meas_cal[i], &data);
		if(rc < 0){
			return rc;
		}
		pdata[i] = data;
	}
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::setMeterCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer){
	if(count < METERCAL_REG_COUNT || analyzer >= _num_analyzers)
		return -1;

	for(uint8_t i = 0; i<METERCAL_REG_COUNT; i++){
		int32_t rc = write(meter_cal[i], pdata[i]);
		if(rc < 0){
			return rc;
		}
	}
	return 0;
}


//------------------------------------------------------------------------------------
int32_t M90E26::setMeasureCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer){
	if(count < METERCAL_REG_COUNT || analyzer >= _num_analyzers)
		return -1;

	for(uint8_t i = 0; i<MEASCAL_REG_COUNT; i++){
		int32_t rc = write(meas_cal[i], pdata[i]);
		if(rc < 0){
			return rc;
		}
	}
	return 0;
}


//------------------------------------------------------------------------------------
//-- PRIVATE METHODS IMPLEMENTATION --------------------------------------------------
//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------
int32_t M90E26::commEnergyIC(uint8_t rw, uint8_t address, uint16_t val){
	uint16_t output;
	int32_t rc = 0;

	//Set read write flag
	address |= rw;
	uint8_t host_chksum = address;

	_serial->startReceiver();

	// en modo escritura envía FEh + 0|7bitADDRESS + MSBdata + LSBdata + CHECKSUM(addr+data) y en menos de 5ms espera CHKSUM(addr+data)
	if(!rw)  {
		uint16_t chksum_short = (val >> 8) + (val & 0xFF) + address;
		host_chksum = chksum_short & 0xFF;
		uint8_t MSBWrite = (val >> 8);
		uint8_t LSBWrite = (val & 0xFF);
		const char buf[] = {0xFE, address, MSBWrite, LSBWrite, host_chksum};
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Escribiendo [%d %d %d %d %d]", buf[0], buf[1], buf[2], buf[3], buf[4]);
		_serial->send((void*)buf, 5);
		char recv_chksum = 0;
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Esperando byte de respuesta...");

		char sent = _serial->recv((void*)&recv_chksum, 1, _recv_timed_ms);
		// realiza espera de 20ms para cierre de transacción
		Thread::wait(20);
		// en caso de error, termina con error
		if(sent == 0 || recv_chksum != host_chksum){
			DEBUG_TRACE_W(_EXPR_, _MODULE_, "ERR_RECV");
			rc = -1;
			goto __exit_commEnergyIC;
		}
		rc = 0;
		goto __exit_commEnergyIC;
	}
	// en modo lectura envía FEh + 0|7bitADDRESS + CHECKSUM(addr) y en menos de 5ms espera recibir MSBdata+LSBdata+CHECKSUMdata
	else{
		const char buf[] = {0xFE, address, host_chksum};
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Solicitando lectura [%d %d %d]", buf[0], buf[1], buf[2]);
		_serial->send((void*)buf, 3);
		char response[3];
		DEBUG_TRACE_D(_EXPR_, _MODULE_, "Esperando respuesta...");
		for(int i=0;i<3;i++){
			if(_serial->recv((void*)&response[i], 1, _recv_timed_ms) == 0){
				DEBUG_TRACE_W(_EXPR_, _MODULE_, "ERR_RECV esperando byte %d", i);
				Thread::wait(20);
				rc = (-2-i);
				goto __exit_commEnergyIC;
			}
		}
		char recv_chksum = ((response[0] + response[1]) & 0xff);
		if(recv_chksum == response[2]) {
			rc = ((uint16_t)response[0] << 8) | (uint16_t)response[1]; //join MSB and LSB;
			goto __exit_commEnergyIC;
		}
		DEBUG_TRACE_W(_EXPR_, _MODULE_, "ERR_CHECKSUM recv=%d esperado=%d", recv_chksum, response[2]);
		rc = -5;
	}

__exit_commEnergyIC:
	_serial->stopReceiver();
	return rc;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibStart(){
	return false;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibPLconst(){
	return false;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibSmallPowerOffset(){
	return false;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibGain(){
	return false;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibLAngle(){
	return false;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibTurnOnPower(){
	return false;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibUIgains(){
	return false;
}


//------------------------------------------------------------------------------------
bool M90E26::_calibEnd(){
	return false;
}


//------------------------------------------------------------------------------------
int32_t M90E26::_calibGetParams(uint16_t* result, int result_size){
	return -1;
}

