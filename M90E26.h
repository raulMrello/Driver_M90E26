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
 
    /** Constructor del driver con la asociaci�n de pines
      * @param p_tx UART_TX o SPI_MOSI
      * @param p_rx UART_RX o SPI_MISO
      * @param hz Velocidad del puerto serie o spi
      * @param p_rst Pin de reset (min 200us a nivel bajo)
      * @param p_irq Pin IRQ
      * @param recv_timed_ms Timeout en recepci�n (en millis)
      * @param defdbg Flag para depuraci�n
      */
    M90E26(PinName p_tx, PinName p_rx, uint32_t hz, PinName p_rst, PinName p_irq, uint32_t recv_timed_ms = DefaultReceptionTimeout, bool defdbg=false);

    /** Constructor del driver con la asociaci�n de pines y un puerto Serie ya preconfigurado
      * @param serial Puerto serie
      * @param p_rst Pin de reset (min 200us a nivel bajo)
      * @param p_irq Pin IRQ
      * @param recv_timed_ms Timeout en recepci�n (en millis)
      * @param defdbg Flag para depuraci�n
      */
    M90E26(Serial* serial, PinName p_rst, PinName p_irq, uint32_t recv_timed_ms = DefaultReceptionTimeout, bool defdbg=false);

    /** Constructor del driver con la asociaci�n de pines y un puerto SPI ya preconfigurado
      * @param spi Puerto SPI
      * @param p_rst Pin de reset (min 200us a nivel bajo)
      * @param p_irq Pin IRQ
      * @param defdbg Flag para depuraci�n
      */
    M90E26(SPI* spi, PinName p_rst, PinName p_irq, bool defdbg=false);
 

    /** Destructor por defecto
     */
    virtual ~M90E26(){}


    /** Inicializa el chip de medida
	 *	@param meter_cal Array de datos de calibraci�n del medidor
	 *	@param meter_cal_count N�mero de elementos del array anterior
	 *	@param meas_cal Array de datos de calibraci�n de la medida
	 *	@param meas_cal_count N�mero de elementos del array anterior
	 */
	virtual void initEnergyIC(uint16_t* meter_cal=NULL, uint8_t meter_cal_count=0, uint16_t* meas_cal=NULL, uint8_t meas_cal_count=0);


    /** Realiza la autocalibraci�n con una carga dada
     *
     * @param max_curr Corriente de la carga dada en A
     * @param turnOnLoad Callback para activar la carga
     * @param turnOffLoad Callback para desactivar la carga
     * @param result Recibe los par�metros de calibraci�n
     * @param result_size Tama�o m�ximo que puede recibir
     * @return Resultado de la calibraci�n (>0: Ok_registros escritos, <0: error)
     */
    virtual int32_t autoCalibration(double max_curr, Callback<void()>turnOnLoad, Callback<void()>turnOffLoad, uint16_t* result, int result_size);


    /** Obtiene el estado del sistema de medida
     *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
     */
    virtual int32_t getSysStatus(uint16_t* pdata);


    /** Obtiene el estado del medidor
     *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
     */
    virtual int32_t getMeterStatus(uint16_t* pdata);


	/** Lee el valor de la tensi�n de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t  getLineVoltage(double* pdata);


	/** Lee el valor de la corriente de red (Amp)
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
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
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getFrequency(double* pdata);


	/** Lee el factor de potencia
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getPowerFactor(double* pdata);


	/** Lee el �ngulo de phase V-I
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getPhase(double* pdata);


	/** Lee la potencia aparente media
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeanAparentPower(double* pdata);


	/** Lee la energ�a de importaci�n
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getImportEnergy(double* pdata);


	/** Lee la energ�a de exportaci�n
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getExportEnergy(double* pdata);


	/** Lee el estado de inicializaci�n del driver
	 *
	 * @return True: ready, sino False
	 */
    virtual bool ready() { return _ready; }


	/** Lee todos los par�metros de energ�a
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getEnergyData(uint16_t* pdata, uint8_t count);


	/** Lee todos los par�metros de medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasurementData(uint16_t* pdata, uint8_t count);


	/** Lee todos los par�metros de calibraci�n del medidor
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeterCalib(uint16_t* pdata, uint8_t count);


	/** Lee todos los par�metros de calibraci�n de la medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasureCalib(uint16_t* pdata, uint8_t count);


	/** Escribe todos los par�metros de calibraci�n del medidor
	 *
     *	@param pdata Datos de calibraci�n
     *	@param count N�mero de par�metros consecutivos a escribir
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t setMeterCalib(uint16_t* pdata, uint8_t count);


	/** Escribe todos los par�metros de calibraci�n de la medida
	 *
     *	@param pdata Datos de calibraci�n
     *	@param count N�mero de par�metros consecutivos a escribir
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t setMeasureCalib(uint16_t* pdata, uint8_t count);


    /** Obtiene los par�metros el�ctricos de la/las l�neas solicitadas
     *
     * @param eparams Recibe los par�metros el�ctricos
     * @param keys Recibe los par�metros que se han le�do <ElectricParamKeys>
     * @param lines L�neas L1, L2, L3 (combinadas o no)
     * @return C�digo de error OK=0, Error<0
     */
    virtual int32_t getElectricParams(ElectricParams eparams[], uint32_t keys[], uint8_t lines);

private:

	/** Timeout por defecto en recepci�n (en millis) */
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

    /** Flag para depuraci�n */
    bool _defdbg;

    /** Flag de estado */
    bool _ready;

    /** Valor de la temporizaci�n de recepci�n para esperar una respuesta v�lida antes de timeout */
    uint32_t _recv_timed_ms;


    /** Rutina para iniciar una operaci�n de lectura
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


    /** Rutina para iniciar una operaci�n de escritura
     *
     * @param address Registro al que escribir
     * @param val Valor a escribir
     * @return Resultado OK = 0,  Error < 0
     */
    int32_t write(uint8_t address, uint16_t val){
    	return commEnergyIC(0x00, address, val);
    }


    /** Rutina para iniciar comunicaci�n con el chip de medida y enviar/recibir datos
     *
     * @param RW Tipo de operaci�n read/write
     * @param address Registro de lectura/escritura
     * @param val Valor a escribir
     * @return Valor le�do
     */
    int32_t commEnergyIC(uint8_t rw, uint8_t address, uint16_t val);

    /** Funciones relativas a la calibraci�n */
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
