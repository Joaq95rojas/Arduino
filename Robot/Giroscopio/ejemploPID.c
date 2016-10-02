// Seteo una diferencia de dirección MÁXIMA aceptable
#define THRESHOLD	5	// [grados]
#define SCALE_FACTOR	// Usar función map()??

// Tomo como valor de referencia, la dirección primera:
direccionReferencia = ypr[0];

// Leer angulo de Yaw actual
direccionActual = ypr[0];

// Calcular el error de dirección
errorDireccion = direccionReferencia - direccionActual;

if( fabs(errorDireccion) <= THRESHOLD )
{
	// Calculo el error acumulado para el control Integral
	ctrlIntegral += errorDireccion;
}
else ctrlIntegral = 0;	// Está fuera de los límites

// Actualizo las constantes del controlador PID
P = errorDireccion * Kp;
I = ctrlIntegral * Ki;
D = (ultimaDireccion - direccionActual) * Kd;

Drive = P + I + D;

// Le aplico un factor de escala para obtener un rango entre 0 y 255
// (Duty Cycle del PWM para cambiar la velocidad de los motores)
Drive = Drive * SCALE_FACTOR;

// analogWrite() debe ir entre 0 y 255
if( Drive < 0 )		// Decrementar velocidad?
else 				// Incrementar velocidad?

if(abs(Drive)>255)	Drive = 255;

ultimaDireccion = direccionActual;
