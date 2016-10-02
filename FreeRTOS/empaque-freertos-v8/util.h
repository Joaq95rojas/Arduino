#ifndef _UTIL_H_
#define _UTIL_H_


// Include
//


#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>


// Macros
//


#define BIT_PIN(i)	digitalPinToBitMask(i)
#define OUT_PIN(i)	portOutputRegister(digitalPinToPort(i))
#define REG_PIN(i)	portModeRegister(digitalPinToPort(i))
#define IN_PIN(i)	*portInputRegister(digitalPinToPort(i))

#define segundosEnHora(segundos) ((float)segundos/60.0/60.0)
#define horaEnSegundos(hora) floor(hora*60*60)
#define parteEntera(f) ((uint16_t)f)
#define parteDecimal(f) (((uint16_t)(f*10000))%10000)

#ifdef DEBUG
	#define _bln_debug_write(i) vTaskSuspendAll(); debugSerial.write(i); xTaskResumeAll();
	#define _bln_debug_print(i) vTaskSuspendAll(); debugSerial.print(i); debugSerial.flush(); xTaskResumeAll();
	#define _bln_debug_println(i) vTaskSuspendAll(); debugSerial.println(i); debugSerial.flush(); xTaskResumeAll();
#else
	#define _bln_debug_write(i)
	#define _bln_debug_print(i)
	#define _bln_debug_println(i)
#endif


// Clase
//


/**
 *
 */
class Util {

public:
	static void configPin(const uint8_t& pNum, const uint8_t& modo);

	static void digWrite(const uint8_t& pNum, const uint8_t& statePin) {
		volatile uint8_t* out = OUT_PIN(pNum);

		if ( statePin ) {
			*out |= BIT_PIN(pNum);
		} else  {
			*out &= ~BIT_PIN(pNum);
		}
	}

	static bool digRead(const uint8_t& pNum) {
		if ( IN_PIN(pNum) & BIT_PIN(pNum) ) {
			return true;
		}

		return false;
	}

	static uint8_t contarOcurrenciasLetra(const char* cadena, const char& letra);

	static size_t trimwhitespace(char *out, size_t len, const char *str);

};

#endif
