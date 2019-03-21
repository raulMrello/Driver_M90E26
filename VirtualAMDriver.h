/*
 * VirtualAMDriver.h
 *
 *  Created on: May 2018
 *      Author: raulMrello
 *
 *	VirtualAMDriver es un driver de ejemplo sin conexión a ningún bus, generalmente utilizado en la fase de TEST
 *	en plataformas sin un driver específico.
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
     *	@param meter_cal Array de datos de calibración del medidor
     *	@param meter_cal_count Número de elementos del array anterior
     *	@param meas_cal Array de datos de calibración de la medida
     *	@param meas_cal_count Número de elementos del array anterior
     */
    virtual void initEnergyIC(uint16_t* meter_cal=NULL, uint8_t meter_cal_count=0, uint16_t* meas_cal=NULL, uint8_t meas_cal_count=0){
    	_ready = true;
    }


    /** Realiza la autocalibración con una carga dada
     *
     * @param max_curr Corriente de la carga dada en A
     * @param turnOnLoad Callback para activar la carga
     * @param turnOffLoad Callback para desactivar la carga
     * @param result Recibe los registros escritos de calibración
     * @param result_size Tamaño máximo que es posible recibir
     * @return Resultado de la calibración (>0: Ok_registros_escritos, <0: error)
     */
    virtual int32_t autoCalibration(double max_curr, Callback<void()>turnOnLoad, Callback<void()>turnOffLoad, uint16_t* result, int result_size){
    	return result_size;
    }

    /** Obtiene el estado del sistema de medida
     *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
     */
    virtual int32_t getSysStatus(uint16_t* pdata){
    	*pdata = 0;
    	return ErrOK;
    }


    /** Obtiene el estado del medidor
     *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
     */
    virtual int32_t getMeterStatus(uint16_t* pdata){
    	return ErrOK;
    }


	/** Lee el valor de la tensión de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t  getLineVoltage(double* pdata){
    	return ErrOK;
    }


	/** Lee el valor de la corriente de red
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
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
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getFrequency(double* pdata){
    	return ErrOK;
    }


	/** Lee el factor de potencia
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getPowerFactor(double* pdata){
    	return ErrOK;
    }


	/** Lee el ángulo de phase V-I
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getPhase(double* pdata){
    	return ErrOK;
    }


	/** Lee la potencia aparente media
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeanAparentPower(double* pdata){
    	return ErrOK;
    }


	/** Lee la energía de importación
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getImportEnergy(double* pdata){
    	return ErrOK;
    }


	/** Lee la energía de exportación
	 *
     *	@param pdata Recibe el resultado
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getExportEnergy(double* pdata){
    	return ErrOK;
    }



	/** Lee el estado de inicialización del driver
	 *
	 * @return True: ready, sino False
	 */
    virtual bool ready(){
    	return _ready;
    }


	/** Lee todos los parámetros de energía
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getEnergyData(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Lee todos los parámetros de medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasurementData(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Lee todos los parámetros de calibración del medidor
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeterCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Lee todos los parámetros de calibración de la medida
	 *
     *	@param pdata Recibe el resultado
     *	@param count Número de parámetros consecutivos a leer
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t getMeasureCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Escribe todos los parámetros de calibración del medidor
	 *
     *	@param pdata Datos de calibración
     *	@param count Número de parámetros consecutivos a escribir
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t setMeterCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


	/** Escribe todos los parámetros de calibración de la medida
	 *
     *	@param pdata Datos de calibración
     *	@param count Número de parámetros consecutivos a escribir
	 * 	@return Código de error OK = 0, Error < 0
	 */
    virtual int32_t setMeasureCalib(uint16_t* pdata, uint8_t count){
    	return ErrOK;
    }


    /** Obtiene los parámetros eléctricos de la/las líneas solicitadas
     *
     * @param eparams Recibe los parámetros eléctricos
     * @param keys Recibe los parámetros que se han leído <ElectricParamKeys>
     * @param lines Líneas L1, L2, L3 (combinadas o no)
     * @return Código de error OK=0, Error<0
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
