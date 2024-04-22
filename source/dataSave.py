import serial
import time
import pandas as pd 
import mysql.connector
from datetime import datetime

def data_extract(serial_port, duration, number):
    ser = serial.Serial(serial_port, 9600)

    data_list = []
    start_time = time.time()

    while (time.time() - start_time) < duration:
        data = ser.readline().decode().strip()
        
        if not data:
            continue
        
        current_time = datetime.now()
        
        two_data = data.split('|')
        if number == 1:
            values = [current_time.strftime("%Y-%m-%d %H:%M:%S")] + list(map(float, two_data[0].split(',')))
        if number == 2:
            values = [current_time.strftime("%Y-%m-%d %H:%M:%S")] + list(map(float, two_data[1].split(',')))
        
        if len(values) >= 7:
            data_list.append(values)
        
        
        

    return data_list

def insert_analog_data_to_mysql(df):
    try:
        remote = mysql.connector.connect(
            host= 'iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com',
            port= 3306,
            user= 'iot',
            password= '1234',
            database= 'iot'
        )

        cursor = remote.cursor()

        sql = "INSERT INTO analog_data VALUES (%s, %s, %s, %s, %s, %s, %s)"

        for i, row in df.iterrows():
            cursor.execute(sql, tuple(row))
            print("Inserted row:", tuple(row))
            remote.commit()

    except mysql.connector.Error as err:
        print("MySQL error:", err)

    finally:
        if 'remote' in locals() and remote.is_connected():
            cursor.close()
            remote.close()
            print("MySQL connection closed")
            
def insert_boolean_data_to_mysql(df):
    try:
        remote = mysql.connector.connect(
            host= 'iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com',
            port= 3306,
            user= 'iot',
            password= '****',
            database= 'iot'
        )

        cursor = remote.cursor()

        sql = "INSERT INTO bool_data VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s)"

        for i, row in df.iterrows():
            cursor.execute(sql, tuple(row))
            print("Inserted row:", tuple(row))
            remote.commit()

    except mysql.connector.Error as err:
        print("MySQL error:", err)

    finally:
        if 'remote' in locals() and remote.is_connected():
            cursor.close()
            remote.close()
            print("MySQL connection closed")
            

data = data_extract('/dev/ttyACM0', 10, 1)
data2 = data_extract('/dev/ttyACM0', 10, 2)
df = pd.DataFrame(data)
df2 = pd.DataFrame(data2)
            
insert_analog_data_to_mysql(df)
insert_boolean_data_to_mysql(df2)

