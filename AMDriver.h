/*
 * AMDriver.h
 *
 *  Created on: May 2018
 *      Author: raulMrello
 *
 *	AMDriver es un interfaz com�n para todo tipo de drivers de medida.
 */
 
#ifndef AMDriver_H
#define AMDriver_H
 
#include "mbed.h"

 



class AMDriver  {
public:

	/** Definici�n del estado de resultado correcto */
	static const int32_t Success = 0;

	/** Errores de estado del driver */
	enum SysStat{
		ErrOK,
		ErrMeterCalib = 0xC000,
		ErrMeasCalib = 0x3000
	};

	/** Par�metros el�tricos comunes */
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


	/** Identificadores de los par�metros el�tricos anteriores */
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


    /** Reasigna el valor de la versi�n utilizada
     *
     * @param version Nueva versi�n a implementar
     */
    void setVersion(const char* version) { _version = version; }


    /** Obtiene el nombre de versi�n del driver
     *
     * @return Versi�n del driver
     */
    const char* getVersion(){ return _version; }


    /** Obtiene el n�mero de analizadores
     *
     * @return Analizadores
     */
    virtual uint8_t getNumAnalyzers() {return _num_analyzers;}


    /** Obtiene el n�mero de serie del analizador o "" si no existe
     * @param serial Recibe el n�mero de serie
     * @param max_len M�ximo tama�o del n�mero de serie
     * @param analyzer Identificador del analizador [0,1,2..]
     */
    virtual void getAnalyzerSerial(char* serial, int max_len, uint8_t analyzer){
    	if(analyzer >= _num_analyzers)
    		strcpy(serial ,"");
    	snprintf(serial, max_len-1, "%s_%d", getVersion(), analyzer);
    }


    /** Inicializa el chip de medida
     *	@param meter_cal Array de datos de calibraci�n del medidor
     *	@param meter_cal_count N�mero de elementos del array anterior
     *	@param meas_cal Array de datos de calibraci�n de la medida
     *	@param meas_cal_count N�mero de elementos del array anterior
     */
    virtual void initEnergyIC(uint16_t* meter_cal=NULL, uint8_t meter_cal_count=0, uint16_t* meas_cal=NULL, uint8_t meas_cal_count=0) = 0;


    /** Realiza la autocalibraci�n con una carga dada
     *
     * @param max_curr Corriente de la carga dada en A
     * @param turnOnLoad Callback para activar la carga
     * @param turnOffLoad Callback para desactivar la carga
     * @param result Recibe los registros escritos de calibraci�n
     * @param result_size Tama�o m�ximo que es posible recibir
     * @return Resultado de la calibraci�n (>0: Ok_registros_escritos, <0: error)
     */
    virtual int32_t autoCalibration(double max_curr, Callback<void()>turnOnLoad, Callback<void()>turnOffLoad, uint16_t* result, int result_size) = 0;

    /** Obtiene el estado del sistema de medida
     *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
     */
    virtual int32_t getSysStatus(uint16_t* pdata) = 0;


    /** Obtiene el estado del medidor
     *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
     */
    virtual int32_t getMeterStatus(uint16_t* pdata) = 0;


	/** Lee el valor de la tensi�n de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t  getLineVoltage(double* pdata) = 0;


	/** Lee el valor de la corriente de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
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
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getFrequency(double* pdata) = 0;


	/** Lee el factor de potencia
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getPowerFactor(double* pdata) = 0;


	/** Lee el �ngulo de phase V-I
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getPhase(double* pdata) = 0;


	/** Lee la potencia aparente media
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeanAparentPower(double* pdata) = 0;


	/** Lee la energ�a de importaci�n
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getImportEnergy(double* pdata) = 0;


	/** Lee la energ�a de exportaci�n
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getExportEnergy(double* pdata) = 0;



	/** Lee el estado de inicializaci�n del driver
	 *
	 * @return True: ready, sino False
	 */
    virtual bool ready() = 0;


	/** Lee todos los par�metros de energ�a
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getEnergyData(uint16_t* pdata, uint8_t count) = 0;


	/** Lee todos los par�metros de medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasurementData(uint16_t* pdata, uint8_t count) = 0;


	/** Lee todos los par�metros de calibraci�n del medidor
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
     *	@param analyzer Analizador del que leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeterCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


	/** Lee todos los par�metros de calibraci�n de la medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
     *	@param analyzer N�mero de analizador a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasureCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


	/** Escribe todos los par�metros de calibraci�n del medidor
	 *
     *	@param pdata Datos de calibraci�n
     *	@param count N�mero de par�metros consecutivos a escribir
     *	@param analyzer N�mero de analizador a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t setMeterCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


	/** Escribe todos los par�metros de calibraci�n de la medida
	 *
     *	@param pdata Datos de calibraci�n
     *	@param count N�mero de par�metros consecutivos a escribir
     *	@param analyzer N�mero de analizador a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t setMeasureCalib(uint16_t* pdata, uint8_t count, uint8_t analyzer) = 0;


    /** Obtiene los par�metros el�ctricos de la/las l�neas solicitadas
     *
     * @param eparams Recibe los par�metros el�ctricos
     * @param keys Recibe los par�metros que se han le�do
     * @param analyzer N�mero de analizador
     * @return C�digo de error OK=0, Error<0
     */
    virtual int32_t getElectricParams(ElectricParams& eparams, uint32_t& keys, uint8_t analyzer) = 0;

protected:

    /** Nombre de la versi�n del driver */
    const char* _version;

    /** N�mero de analizadores integrados */
    uint8_t _num_analyzers;
};
 
#endif      // AMDriver_H
