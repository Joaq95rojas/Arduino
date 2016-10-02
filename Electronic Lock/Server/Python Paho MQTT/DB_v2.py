# -*- coding: cp1252 -*-
'''
PROGRAMA QUE REVISA SI EXISTE UNA BASE DE DATOS.
    - NO: LA CREA
    - SI: LEE LOS TAGS RFID DEL PERSONAL
'''

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

def on_connect(mqttc, obj, flags, rc):
    print("CONECTANDO: "+str(rc))

def on_message(mqttc, obj, msg):
    print("TOPIC: " + msg.topic + " MSG: " + str(msg.payload))
    # CONECTO A LA DB
    conn = pypyodbc.win_connect_mdb(PATH)
    cur = conn.cursor()
    print "Conección a la Base de datos exitosa."

    #'%s'" % data['Device Name']   ///   

    # BUSCO LA "ZONA" DONDE SE ALOJAN LOS TAGS_ID
    cur.execute("SELECT id FROM tarjetas_rfid WHERE codigo='TagRFID'")
    for row in cur.fetchall():
            for field in row:
                    print field,
            print ''
    print TagRFID;
    # CIERRO TODO. SOLO PARA DEBUGGING
    cur.close()
    conn.close()
    print "Base de datos cerrada correctamente."

def on_publish(mqttc, obj, mid):
    print("mid: "+str(mid))

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

# CHEQUEO LA EXISTENCIA DE LA BASE DE DATOS
# http://pythoncentral.io/check-file-exists-in-directory-python/
# http://www.pfinn.net/python-check-if-file-exists.html

PATH = 'C:\\Users\\Eric\\Desktop\\mosquittoApp\\DB\\TarjetaRFID.mdb'
TagRFID = str(msg.payload)

if os.path.isfile(PATH):
    if os.access(PATH, os.R_OK):
        print "Archivo abierto correctamente."
    else:
        print "ERROR: Cambie los privilegios de lectura del archivo."            
else:
    print "ERROR: Archivo no encontrado."
    pypyodbc.win_create_mdb(PATH)

'''
# OTRA ALTERNATIVA PARA CHEQUEAR LA EXISTENCIA DEL ARCHIVO
def open_if_not_exists(filename):
    try:
        fd = os.open(filename, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except OSError, e:
        if e.errno == 17:
            print e
            return None
        else:
            raise
    else:
        print "Archivo abierto."
        return os.fdopen(fd, 'w')

open_if_not_exists(PATH)
'''

mqttc.loop_forever()
