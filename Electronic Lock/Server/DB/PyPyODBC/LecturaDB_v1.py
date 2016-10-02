'''
PROGRAMA EN PYTHON PARA LEER BASES DE DATOS
'''

########
# https://code.google.com/p/pypyodbc/
########

''' Create an Access database and a sales table in this database '''

# We command Python to identify the just installed pypyodbc module
# from it's module library, for us to use in the next steps.
import pypyodbc
# We want to create an Access database to store sales data.
# This is a featured function of PyPyODBC, so users can easily
# use it to create a blank Access database on Windows platforms.
# We name the Access database salesdb.mdb, and put it under the
# root directory of D drive.
pypyodbc.win_create_mdb('C:\\Users\\Eric\\Desktop\\mosquittoApp\\DB\\TagsDB.mdb')
# using pypyodbc module, we obtain an ODBC connection object
# 'conn' to the salesdb.mdb through an Access ODBC connection string:
conn = pypyodbc.win_connect_mdb('C:\\Users\\Eric\\Desktop\\mosquittoApp\\DB\\TagsDB.mdb')
# And from this connection object, get an operational cursor
# of the connected database:
cur = conn.cursor()
# Create a new sales record table "saleout" in the database
# We use the cursor to pass a SQL command to the Access database,
# create a table named saleout:
cur.execute('''CREATE TABLE cerraduras (
				ID AUTOINCREMENT PRIMARY KEY,
				nombre_empleado VARCHAR(25),
				horario_entrada date, 
				horario_salida date);''')
# This database is created with a table with fields of:
# - ID (ID),
# - customer_name (the customer's name),
# - product_name (product name),
# - price (selling price),
# - volume (quantity) and
# - sell_time (sale time).
# Finally, we submitted the operations, so they officially
# take effect in the database in the same time.
cur.commit()

# It can be done in one step:
# cur.execute('CREATE TABLE cerraduras(...);').commit()

''' Record sales transactions by inserting records in the table '''

# After the database and table have been created, we can fill in
# some sales records.
# The first step, we record a customer of Mr. Jiang (Jiang Wen),
# on January 21, 2013, bought two Huawei Ascend mate phone with 5000.5 yuan:
cur.execute('''INSERT INTO cerraduras(nombre_empleado,horario_entrada,
horario_salida) VALUES(?,?,?)''',(u'Emiliano',20-11-2015,21-11-2015))
# Do not forget to submit the operations, so the record
# officially take effect in the database:
cur.commit()

# Next, we then record a batch of sales:
cur.execute('''INSERT INTO cerraduras(nombre_empleado,horario_entrada,
horario_salida) VALUES(?,?,?)''',(u'Biato',02-06-2011,05-11-2013))
cur.execute('''INSERT INTO cerraduras(nombre_empleado,horario_entrada,
horario_salida) VALUES(?,?,?)''',(u'Eric',21-11-1994,20-01-2004))
# Immediately submit the changes, so these 4 records officially
# come into effect at the same time in the database:
cur.commit()
# At this point, we have 5 sales transactions recorded in the system.
# In the following steps, we will query these records.

# Insert a batch rows of data to the sellout table using a same
# query.

a_batch_rows = [(u'Miguel', '21-1-2012', '22-1-2012'),
                (u'José', '21-1-2012', '22-1-2012'),
                (u'Manuel', '21-1-2012', '22-1-2012'),
                (u'Fernando', '21-1-2012', '22-1-2012')]

cur.executemany('''INSERT INTO cerraduras(
					nombre_empleado, 
					horario_entrada, 
					horario_salida) VALUES(?,?,?)''',
					a_batch_rows)

# Make all my changes to the data take effective, Now!
cur.commit()

''' Use Python to query data in the database
and compress the MS-Access database file. '''

# If we want to query on January 21, 2012, our sales of Huawei products,
# how should we do? At this time, we will pass a SQL query to the Access
# database, and obtain the returned database query results as
# Python variables. First we pass the SQL query to Access database:
cur.execute('''SELECT * FROM cerraduras WHERE nombre_empleado LIKE '%Emiliano%';''')
# Then we get the names of the fields of the result set from the database query results:
# for d in cur.description:
# 	print d[0],
# The interactive interface will display the name of each field. Next we display
# result set row by row of the result set on the screen:
for row in cur.fetchall():
	for field in row:
		print field,
	print ''
# This shows all of the result set.
# For Access databases, after long data insertion, updating,
# deleting operations, the Access database file may become very
# bloated and huge. PyPyODBC provides another featured functions,
# which can directly call the database cleansing function in 
# Python programs. We now use this function and generate a
# compressed database file salesdb_backup.mdb from the original 
# database file salesdb.mdb: First, close the database connection:
cur.close()
conn.close()
# Then use pypyodbc win_compact_mdb function to compress the database file:
pypyodbc.win_compact_mdb('C:\\Users\\Eric\\Desktop\\mosquittoApp\\DB\\TagsDB.mdb',
	'C:\\Users\\Eric\\Desktop\\mosquittoApp\\DB\\TagsDB_Compact.mdb')
# At this time, we will find a compressed salesdb_backup.mdb
# file generated under the D drive, how ever, there is no
# big difference between the sizes of the two files in this case,
# but after long and frequent use of the database, the compression 
# effect of the cleanup will be huge.


### BUSCAR COMUNICACION ENTRE PROGRAMAS DE C Y PYTHON