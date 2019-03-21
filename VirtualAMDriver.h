/*
 * VirtualAMDriver.h
 *
 *  Created on: May 2018
 *      Author: raulMrello
 *
 *	VirtualAMDriver es un driver de ejemplo sin conexi�n a ning�n bus, generalmente utilizado en la fase de TEST
 *	en plataformas sin un driver espec�fico.
 */
 
#ifndef VirtualAMDriver_H
#define VirtualAMDriver_H
 
#include "mbed.h"
#include "AMDriver.h"

 



class VirtualAMDriver : public AMDriver  {
public:

    /** Constructor del interfaz
      */
	VirtualAMDriver() : AMDriver("SIM", 1){
		_ready = false;
	}

    /** Destructor por defecto
     */
    virtual ~VirtualAMDriver(){}


    /** Inicializa el chip de medida
     *	@param meter_cal Array de datos de calibraci�n del medidor
     *	@param meter_cal_count N�mero de elementos del array anterior
     *	@param meas_cal Array de datos de calibraci�n de la medida
     *	@param meas_cal_count N�mero de elementos del array anterior
     */
    virtual void initEnergyIC(uint16_t* meter_cal=NULL, uint8_t meter_cal_count=0, uint16_t* meas_cal=NULL, uint8_t meas_cal_count=0){
    	_ready = true;
    }


    /** Realiza la autocalibraci�n con una carga dada
     *
     * @param max_curr Corriente de la carga dada en A
     * @param turnOnLoad Callback para activar la carga
     * @param turnOffLoad Callback para desactivar la carga
     * @param result Recibe los registros escritos de calibraci�n
     * @param result_size Tama�o m�ximo que es posible recibir
     * @return Resultado de la calibraci�n (>0: Ok_registros_escritos, <0: error)
     */
    virtual int32_t autoCalibration(double max_curr, Callback<void()>turnOnLoad, Callback<void()>turnOffLoad, uint16_t* result, int result_size){
    	return result_size;
    }

    /** Obtiene el estado del sistema de medida
     *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
     */
    virtual int32_t getSysStatus(uint16_t* pdata){
    	*pdata = 0;
    	return ErrOK;
    }


    /** Obtiene el estado del medidor
     *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
     */
    virtual int32_t getMeterStatus(uint16_t* pdata){
    	return ErrOK;
    }


	/** Lee el valor de la tensi�n de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t  getLineVoltage(double* pdata){
    	return ErrOK;
    }


	/** Lee el valor de la corriente de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getLineCurrent(double* pdata){
    	return ErrOK;
    }


	/** Lee la potencia activa
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Resultado >= 0, Error < 0
	 */
    virtual int32_t getActivePower(double* pdata){
    	return ErrOK;
    }


	/** Lee la potencia reactiva
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Resultado >= 0, Error < 0
	 */
    virtual int32_t getReactivePower(double* pdata){
    	return ErrOK;
    }


	/** Lee la frecuencia de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getFrequency(double* pdata){
    	return ErrOK;
    }


	/** Lee el factor de potencia
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getPowerFactor(double* pdata){
    	return ErrOK;
    }


	/** Lee el �ngulo de phase V-I
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getPhase(double* pdata){
    	return ErrOK;
    }


	/** Lee la potencia aparente media
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeanAparentPower(double* pdata){
    	return ErrOK;
    }


	/** Lee la energ�a de importaci�n
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getImportEnergy(double* pdata){
    	return ErrOK;
    }


	/** Lee la energ�a de exportaci�n
	 *
     *	@param pdata Recibe el resultado
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getExportEnergy(double* pdata){
    	return ErrOK;
    }



	/** Lee el estado de inicializaci�n del driver
	 *
	 * @return True: ready, sino False
	 */
    virtual bool ready(){
    	return _ready;
    }


	/** Lee todos los par�metros de energ�a
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getEnergyData(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Lee todos los par�metros de medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasurementData(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Lee todos los par�metros de calibraci�n del medidor
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeterCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Lee todos los par�metros de calibraci�n de la medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count N�mero de par�metros consecutivos a leer
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasureCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Escribe todos los par�metros de calibraci�n del medidor
	 *
     *	@param pdata Datos de calibraci�n
     *	@param count N�mero de par�metros consecutivos a escribir
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t setMeterCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Escribe todos los par�metros de calibraci�n de la medida
	 *
     *	@param pdata Datos de calibraci�n
     *	@param count N�mero de par�metros consecutivos a escribir
	 * 	@return C�digo de error OK = 0, Error < 0
	 */
    virtual int32_t setMeasureCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


    /** Obtiene los par�metros el�ctricos de la/las l�neas solicitadas
     *
     * @param eparams Recibe los par�metros el�ctricos
     * @param keys Recibe los par�metros que se han le�do <ElectricParamKeys>
     * @param lines L�neas L1, L2, L3 (combinadas o no)
     * @return C�digo de error OK=0, Error<0
     */
    virtual int32_t getElectricParams(ElectricParams eparams[], uint32_t keys[], uint8_t lines){
    	keys[0] = ElecKey_None;
    	keys[1] = ElecKey_None;
    	keys[2] = ElecKey_None;
    	return 0;
    }
private:
    bool _ready;
};
 
#endif      // VirtualAMDriver_H
