#include "util.h"


void Util::configPin(const uint8_t& pNum, const uint8_t& modo) {
	volatile uint8_t *reg,*out;

	reg = REG_PIN(pNum);
	out = OUT_PIN(pNum);

	uint8_t oldSREG = SREG;

	cli();

	if ( modo ) {
		*reg |= BIT_PIN(pNum);
	} else {
		*reg &= ~BIT_PIN(pNum);
		*out &= ~BIT_PIN(pNum);
	}

	SREG = oldSREG;
}


uint8_t Util::contarOcurrenciasLetra(const char* cadena, const char& letra) {
	uint8_t contador = 0;
	uint8_t i = 0;

	for ( i = 0; i < strlen(cadena); i++ ) {
		if (cadena[i] == letra) {
			contador++;
		}
	}

	return contador;
}


size_t Util::trimwhitespace(char *out, size_t len, const char *str) {
	if ( len == 0 ) {
		return 0;
	}

	const char *end;
	size_t out_size;

	// Trim leading space
	while(isspace(*str)) str++;

	if(*str == 0) { // All spaces?
		*out = 0;
		return 1;
	}

	// Trim trailing space
	end = str + strlen(str) - 1;
	while(end > str && isspace(*end)) end--;
	end++;

	// Set output size to minimum of trimmed string length and buffer size minus 1
	out_size = (end - str) < len-1 ? (end - str) : len-1;

	// Copy trimmed string and add null terminator
	memcpy(out, str, out_size);
	out[out_size] = '\0';

	return out_size;
}
