/*
 * AMDriver.h
 *
 *  Created on: May 2018
 *      Author: raulMrello
 *
 *	AMDriver es un interfaz común para todo tipo de drivers de medida.
 */
 
#ifndef AMDriver_H
#define AMDriver_H
 
#include "mbed.h"

 



class AMDriver  {
public:

	/** Definición del estado de resultado correcto */
	static const int32_t Success = 0;

	/** Errores de estado del driver */
	enum SysStat{
		ErrOK,
		ErrMeterCalib = 0xC000,
		ErrMeasCalib = 0x3000
	};

	/** Parámetros elétricos comunes */
	struct ElectricParams{
		double voltage;
		double current;
		double aPow;
		double rPow;
		double mPow;
		double pFactor;
		double thdVolt;
		double thdAmp;
		double freq;
		double phase;
		double aEnergy;
		double rEnergy;
	};


	/** Identificadores de los parámetros elétricos anteriores */
	enum ElectricParamKeys{
		ElecKey_None		 = 0,
		ElecKey_Voltage 	 = (1 << 0),
		ElecKey_Current 	 = (1 << 1),
		ElecKey_ActivePow 	 = (1 << 2),
		ElecKey_ReactivePow  = (1 << 3),
		ElecKey_ApparentPow  = (1 << 4),
		ElecKey_PowFactor 	 = (1 << 5),
		ElecKey_THDVoltage 	 = (1 << 6),
		ElecKey_THDAmpere 	 = (1 << 7),
		ElecKey_Frequency 	 = (1 << 8),
		ElecKey_Phase 		 = (1 << 9),
		ElecKey_ActiveEnergy = (1 << 10),
		ElecKey_ReactiveEnergy = (1 << 11),
	};


    /** Constructor del interfaz
      */
	AMDriver(const char* version, uint8_t num_analyzers) : _version(version), _num_analyzers(num_analyzers){}


    /** Destructor por defecto
     */
    virtual ~AMDriver(){}


    /** Reasigna el valor de la versión utilizada
     *
     * @param version Nueva versión a implementar
     */
    void setVersion(const char* version) { _version = version; }


    /** Obtiene el nombre de versión del driver
     *
     * @return Versión del driver
     */
    const char* getVersion(){ return _version; }


    /** Obtiene el número de analizadores
     *
     * @return Analizadores
     */
    virtual uint8_t getNumAnalyzers() {return _num_analyzers;}


    /** Obtiene el número de serie del analizador o "" si no existe
     * @param serial Recibe el número de serie
     * @param max_len Máximo tamaño del número de serie
     * @param analyzer Identificador del analizador [0,1,2..]
     */
    virtual void getAnalyzerSerial(char* serial, int max_len, uint8_t analyzer){
    	if(analyzer >= _num_analyzers)
    		strcpy(serial ,"");
    	snprintf(serial, max_len-1, "%s_%d", getVersion(), analyzer);
    }


    /** Inicializa el chip de medida
     *	@param meter_cal Array de datos de calibración del medidor
     *	@param meter_cal_count Número de elementos del array anterior
     *	@param meas_cal Array de datos de calibración de la medida
     *	@param meas_cal_count Número de elementos del array anterior
     */
    virtual void initEnergyIC(uint16_t* meter_cal=NULL, uint8_t meter_cal_count=0, uint16_t* meas_cal=NULL, uint8_t meas_cal_count=0) = 0;


    /** Realiza la autocalibración con una carga dada
     *
     * @param max_curr Corriente de la carga dada en A
     * @param turnOnLoad Callback para activar la carga
     * @param turnOffLoad Callback para desactivar la carga
     * @param result Recibe los registros escritos de calibración
     * @param result_size Tamaño máximo que es posible recibir
     * @return Resultado de la calibración (>0: Ok_registros_escritos, <0: error)
     */
    virtual int32_t autoCalibration(double max_curr, Callback<void()>turnOnLoad, Callback<void()>turnOffLoad, uint16_t* result, int result_size) = 0;

    /** Obtiene el estado del sistema de medida
     *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
     */
    virtual int32_t getSysStatus(uint16_t* pdata) = 0;


    /** Obtiene el estado del medidor
     *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
     */
    virtual int32_t getMeterStatus(uint16_t* pdata) = 0;


	/** Lee el valor de la tensión de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t  getLineVoltage(double* pdata) = 0;


	/** Lee el valor de la corriente de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getLineCurrent(double* pdata) = 0;


	/** Lee la potencia activa
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Resultado >= 0, Error < 0
	 */
    virtual int32_t getActivePower(double* pdata) = 0;


	/** Lee la potencia reactiva
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Resultado >= 0, Error < 0
	 */
    virtual int32_t getReactivePower(double* pdata) = 0;


	/** Lee la frecuencia de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getFrequency(double* pdata) = 0;


	/** Lee el factor de potencia
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getPowerFactor(double* pdata) = 0;


	/** Lee el ángulo de phase V-I
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getPhase(double* pdata) = 0;


	/** Lee la potencia aparente media
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeanAparentPower(double* pdata) = 0;


	/** Lee la energía de importación
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getImportEnergy(double* pdata) = 0;


	/** Lee la energía de exportación
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getExportEnergy(double* pdata) = 0;



	/** Lee el estado de inicialización del driver
	 *
	 * @return True: ready, sino False
	 */
    virtual bool ready() = 0;


	/** Lee todos los parámetros de energía
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getEnergyData(uint16_t* pdata, uint8_t count) = 0;


	/** Lee todos los parámetros de medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasurementData(uint16_t* pdata, uint8_t count) = 0;


	/** Lee todos los parámetros de calibración del medidor
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
     *	@param analyzer Analizador del que leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeterCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


	/** Lee todos los parámetros de calibración de la medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
     *	@param analyzer Número de analizador a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasureCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


	/** Escribe todos los parámetros de calibración del medidor
	 *
     *	@param pdata Datos de calibración
     *	@param count Número de parámetros consecutivos a escribir
     *	@param analyzer Número de analizador a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t setMeterCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


	/** Escribe todos los parámetros de calibración de la medida
	 *
     *	@param pdata Datos de calibración
     *	@param count Número de parámetros consecutivos a escribir
     *	@param analyzer Número de analizador a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t setMeasureCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


    /** Obtiene los parámetros eléctricos de la/las líneas solicitadas
     *
     * @param eparams Recibe los parámetros eléctricos
     * @param keys Recibe los parámetros que se han leído
     * @param analyzer Número de analizador
     * @return Código de error OK=0, Error<0
     */
    virtual int32_t getElectricParams(ElectricParams& eparams, uint32_t& keys, uint8_t analyzer) = 0;

protected:

    /** Nombre de la versión del driver */
    const char* _version;

    /** Número de analizadores integrados */
    uint8_t _num_analyzers;
};
 
#endif      // AMDriver_H
