/*
 * M90E26.h
 *
 *  Created on: Feb 2018
 *      Author: raulMrello
 *
 *	M90E26 es el driver de control del chip RTC M90E26, que utiliza un canal I2C. Implementa el interfaz RealTimeClock
 *	para leer y escribir la hora del chip RTC.
 */
 
#ifndef M90E26_H
#define M90E26_H
 
#include "mbed.h"
#include "AMDriver.h"

 



class M90E26 : public AMDriver {
public:
 
    /** Constructor del driver con la asociación de pines
      * @param p_tx UART_TX o SPI_MOSI
      * @param p_rx UART_RX o SPI_MISO
      * @param hz Velocidad del puerto serie o spi
      * @param p_rst Pin de reset (min 200us a nivel bajo)
      * @param p_irq Pin IRQ
      * @param recv_timed_ms Timeout en recepción (en millis)
      * @param defdbg Flag para depuración
      */
    M90E26(PinName p_tx, PinName p_rx, uint32_t hz, PinName p_rst, PinName p_irq, uint32_t recv_timed_ms = DefaultReceptionTimeout, bool defdbg=false);

    /** Constructor del driver con la asociación de pines y un puerto Serie ya preconfigurado
      * @param serial Puerto serie
      * @param p_rst Pin de reset (min 200us a nivel bajo)
      * @param p_irq Pin IRQ
      * @param recv_timed_ms Timeout en recepción (en millis)
      * @param defdbg Flag para depuración
      */
    M90E26(Serial* serial, PinName p_rst, PinName p_irq, uint32_t recv_timed_ms = DefaultReceptionTimeout, bool defdbg=false);

    /** Constructor del driver con la asociación de pines y un puerto SPI ya preconfigurado
      * @param spi Puerto SPI
      * @param p_rst Pin de reset (min 200us a nivel bajo)
      * @param p_irq Pin IRQ
      * @param defdbg Flag para depuración
      */
    M90E26(SPI* spi, PinName p_rst, PinName p_irq, bool defdbg=false);
 

    /** Destructor por defecto
     */
    virtual ~M90E26(){}


    /** Inicializa el chip de medida
	 *	@param meter_cal Array de datos de calibración del medidor
	 *	@param meter_cal_count Número de elementos del array anterior
	 *	@param meas_cal Array de datos de calibración de la medida
	 *	@param meas_cal_count Número de elementos del array anterior
	 */
	virtual void initEnergyIC(uint16_t* meter_cal=NULL, uint8_t meter_cal_count=0, uint16_t* meas_cal=NULL, uint8_t meas_cal_count=0);


    /** Realiza la autocalibración con una carga dada
     *
     * @param max_curr Corriente de la carga dada en A
     * @param turnOnLoad Callback para activar la carga
     * @param turnOffLoad Callback para desactivar la carga
     * @param result Recibe los parámetros de calibración
     * @param result_size Tamaño máximo que puede recibir
     * @return Resultado de la calibración (>0: Ok_registros escritos, <0: error)
     */
    virtual int32_t autoCalibration(double max_curr, Callback<void()>turnOnLoad, Callback<void()>turnOffLoad, uint16_t* result, int result_size);


    /** Obtiene el estado del sistema de medida
     *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
     */
    virtual int32_t getSysStatus(uint16_t* pdata);


    /** Obtiene el estado del medidor
     *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
     */
    virtual int32_t getMeterStatus(uint16_t* pdata);


	/** Lee el valor de la tensión de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t  getLineVoltage(double* pdata);


	/** Lee el valor de la corriente de red (Amp)
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getLineCurrent(double* pdata);


	/** Lee la potencia activa
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Resultado >= 0, Error < 0
	 */
    virtual int32_t getActivePower(double* pdata);


	/** Lee la potencia reactiva
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Resultado >= 0, Error < 0
	 */
    virtual int32_t getReactivePower(double* pdata);


	/** Lee la frecuencia de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getFrequency(double* pdata);


	/** Lee el factor de potencia
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getPowerFactor(double* pdata);


	/** Lee el ángulo de phase V-I
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getPhase(double* pdata);


	/** Lee la potencia aparente media
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeanAparentPower(double* pdata);


	/** Lee la energía de importación
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getImportEnergy(double* pdata);


	/** Lee la energía de exportación
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getExportEnergy(double* pdata);


	/** Lee el estado de inicialización del driver
	 *
	 * @return True: ready, sino False
	 */
    virtual bool ready() { return _ready; }


	/** Lee todos los parámetros de energía
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getEnergyData(uint16_t* pdata, uint8_t count);


	/** Lee todos los parámetros de medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasurementData(uint16_t* pdata, uint8_t count);


	/** Lee todos los parámetros de calibración del medidor
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeterCalib(uint16_t* pdata, uint8_t count);


	/** Lee todos los parámetros de calibración de la medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasureCalib(uint16_t* pdata, uint8_t count);


	/** Escribe todos los parámetros de calibración del medidor
	 *
     *	@param pdata Datos de calibración
     *	@param count Número de parámetros consecutivos a escribir
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t setMeterCalib(uint16_t* pdata, uint8_t count);


	/** Escribe todos los parámetros de calibración de la medida
	 *
     *	@param pdata Datos de calibración
     *	@param count Número de parámetros consecutivos a escribir
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t setMeasureCalib(uint16_t* pdata, uint8_t count);


    /** Obtiene los parámetros eléctricos de la/las líneas solicitadas
     *
     * @param eparams Recibe los parámetros eléctricos
     * @param keys Recibe los parámetros que se han leído <ElectricParamKeys>
     * @param lines Líneas L1, L2, L3 (combinadas o no)
     * @return Código de error OK=0, Error<0
     */
    virtual int32_t getElectricParams(ElectricParams eparams[], uint32_t keys[], uint8_t lines);

private:

	/** Timeout por defecto en recepción (en millis) */
	static const uint32_t DefaultReceptionTimeout = 5;

	/** Controlador puerto serie */
    Serial* _serial;

    /** Controlador puerto spi */
    SPI* _spi;

    /** Controladores de GPIOs dedicados */
    DigitalOut* _out_rst;
    DigitalOut* _out_cs;
    DigitalOut* _out_clk;
    InterruptIn* _iin_irq;

    /** Flag para depuración */
    bool _defdbg;

    /** Flag de estado */
    bool _ready;

    /** Valor de la temporización de recepción para esperar una respuesta válida antes de timeout */
    uint32_t _recv_timed_ms;


    /** Rutina para iniciar una operación de lectura
     *
     * @param address Registro del que leer
     * @param pdata Recibe los datos recibidos
     * @return Resultado OK = 0,  Error < 0
     */
    int32_t read(uint8_t address, uint16_t* pdata){
    	int32_t rc = commEnergyIC(0x80, address, 0);
    	if(rc < 0){
    		return rc;
    	}
    	*pdata = (uint16_t)rc;
    	return 0;
    }


    /** Rutina para iniciar una operación de escritura
     *
     * @param address Registro al que escribir
     * @param val Valor a escribir
     * @return Resultado OK = 0,  Error < 0
     */
    int32_t write(uint8_t address, uint16_t val){
    	return commEnergyIC(0x00, address, val);
    }


    /** Rutina para iniciar comunicación con el chip de medida y enviar/recibir datos
     *
     * @param RW Tipo de operación read/write
     * @param address Registro de lectura/escritura
     * @param val Valor a escribir
     * @return Valor leído
     */
    int32_t commEnergyIC(uint8_t rw, uint8_t address, uint16_t val);

    /** Funciones relativas a la calibración */
    bool _calibStart();
    bool _calibPLconst();
    bool _calibSmallPowerOffset();
    bool _calibGain();
    bool _calibLAngle();
    bool _calibTurnOnPower();
    bool _calibUIgains();
    bool _calibEnd();
    int32_t _calibGetParams(uint16_t* result, int result_size);

};
 
#endif      // M90E26_H
