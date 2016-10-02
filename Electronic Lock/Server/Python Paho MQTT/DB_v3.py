# -*- coding: cp1252 -*-

# IMPRIME MAL EL ID DE MI TARJETA
# DETECTAR CUANDO NO SE ENCUENTRA EL ID Y ASIGNARLE UN ERROR
# PUBLICAR EL RESULTADO EN UN TOPICO DISTINTO
# OK : VOLVER AL LOOP

import pypyodbc
import datetime     # VER COMO HACER PARA CONSEGUIR LA FECHA
import os.path

import sys
try:
    import paho.mqtt.client as mqtt
except ImportError:
    import os
    import inspect
    cmd_subfolder = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"../src")))
    if cmd_subfolder not in sys.path:
        sys.path.insert(0, cmd_subfolder)
    import paho.mqtt.client as mqtt

PATH = 'C:\\Users\\Eric\\Desktop\\mosquittoApp\\DB\\TarjetaRFID.mdb'

def on_connect(mqttc, obj, flags, rc):
    print("CONECTANDO: "+str(rc))

def on_message(mqttc, obj, msg):
    print("\n* TOPIC: " + msg.topic + "\n* MSG: " + str(msg.payload))

    TagRFID = str(msg.payload)
    print ":: " + TagRFID
    
    if os.path.isfile(PATH):
        if os.access(PATH, os.R_OK):
            print "Archivo abierto correctamente."
        else:
            print "ERROR: Cambie los privilegios de lectura del archivo."            
    else:
        print "ERROR: Archivo no encontrado."
        pypyodbc.win_create_mdb(PATH)
    
    # CONECTO A LA DB
    conn = pypyodbc.win_connect_mdb(PATH)
    cur = conn.cursor()
    print "Conección a la Base de datos exitosa." 

    # BUSCO LA "ZONA" DONDE SE ALOJAN LOS TAGS_ID
    cur.execute("SELECT id_empleado, baja FROM Tarjetas_rfid WHERE codigo LIKE ?", [TagRFID])

    for row in cur.fetchall():
        if row['baja'] == 0:
            print "ID: " + str(row['id_empleado']) + '\n',
            ID = str(row['id_empleado'])

            cur.execute("SELECT Nombre, Apellido FROM Personal WHERE Id=?", [ID])
            for row in cur.fetchall():
                # row.get('campo')
                # row['campo']
                # row[0]
                print "Persona: " + str(row[0]) + " " + str(row[1]),
            print ''
            RESP = 0    # 0: HABILITADO PARA INGRESAR
        elif row['baja'] == -1:
            print "Empleado dado de baja."
            RESP = -1   # -1: NO HABILITADO PARA INGRESAR
        else:
            print "ERROR: Baja mal asignada."
            RESP = -2   # -2: ERROR EN LA BASE DE DATOS
            
    # CIERRO TODO. SOLO PARA DEBUGGING
    try:
        cur.close()
        conn.close()
        print ">> BD Cerrada <<\n"
    except:
        print "ERROR: >> Cerrando BD <<\n"

    # PUBLICACIÓN DEL RESULTADO
    mqttc.publish("cerraduras/andif-C1/tag-resp", RESP)
    
def on_publish(mqttc, obj, mid):
    print "Mensaje publicado >> " 

def on_subscribe(mqttc, obj, mid, granted_qos):
    print("Subscribed: "+str(mid)+" "+str(granted_qos))

def on_log(mqttc, obj, level, string):
    print(string)

mqttc = mqtt.Client()
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
#mqttc.on_log = on_log
mqttc.connect("192.168.1.197", 1883, 60)
mqttc.subscribe("cerraduras/andif-C1/tag", 0)
print ''
mqttc.loop_forever()
