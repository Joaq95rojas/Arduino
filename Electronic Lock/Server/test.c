/* 
 *	PARA COMPILAR: compilar.bat
 */

#include <stdlib.h>
#include <stdio.h>
#include <mosquitto.h>


 static int run = -1;

void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
	if(rc){
		printf("Connection error.\n");
		exit(1);
	}else{
		printf("MOSQUITTO SUBSCRIBING ");
		/* Si conecta, se suscribe al tópico */
		mosquitto_subscribe(mosq, NULL, "cerraduras/andif-C1/tag", 1);	/* QoS=0 */
	}
}

void on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
	printf("MOSQUITTO_DISCONNECTED\n");
	run = rc;
}

void on_subscribe(struct mosquitto *mosq, void *obj, int mid, int qos_count, const int *granted_qos)
{
	printf(" -> SUBSCRIBED\n"); 
/*	mosquitto_disconnect(mosq); */
}

void on_message (struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{
	printf("\n");
	printf("-> Mensaje recibido\n");
	
	if (msg->payloadlen != 26)
	{
		printf("* PAYLOAD MESSAGE LENGHT ERROR *\n");
		exit(1);
	}
	else{
		printf("TOPIC: \"%s\"\n", msg->topic);
		printf("PAYLOAD: %s\n", (char*)msg->payload);
		/* 
		 * LEER BASE DE DATOS Y BUSCAR TAG.
		 * APROBAR ORDEN EN CASO DE ENCONTRARLO.
		 */
	}
}

int main(void) {
	int rc;
	struct mosquitto* mqtt;

	mosquitto_lib_init();
	
	mqtt = mosquitto_new("prueba1", false, NULL);

	mosquitto_connect_callback_set(mqtt, on_connect);
	mosquitto_disconnect_callback_set(mqtt, on_disconnect);
	mosquitto_subscribe_callback_set(mqtt, on_subscribe);
	mosquitto_message_callback_set(mqtt, on_message);
	
	printf("MOSQUITTO CONNECTING ");
	rc = mosquitto_connect(mqtt, "localhost", 1883, 10);
	/* localhost: 192.168.1.197 */

	if(rc == MOSQ_ERR_SUCCESS){
		printf(" -> CONNECTED\n");
	} else printf("CONNECTION ERROR\n");

	while (1) {
		mosquitto_loop(mqtt, -1, 1);
	}

	mosquitto_destroy(mqtt);
	mosquitto_lib_cleanup();

	return run;
}