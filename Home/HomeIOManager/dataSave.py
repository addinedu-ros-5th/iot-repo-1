import serial
import time
import pandas as pd 
from datetime import datetime
import mysql.connector

def data_extract(serial_port, duration):
    ser = serial.Serial(serial_port, 9600)

    data_list = []
    start_time = time.time()

    while (time.time() - start_time) < duration:
        data = ser.readline().decode().strip()
        
        if not data:
            continue
        
        current_time = datetime.now()
        
        two_data = data.split('|')
        
        values = [current_time.strftime("%Y-%m-%d %H:%M:%S")] + list(map(float, two_data[0].split(',')))
    
        
        if len(values) >= 7:
            data_list.append(values)
            
    ser.close()

    return data_list
            
def run(serial_port):
    ser = serial.Serial(serial_port, 9600)
    data_list = []

    try:
        while True:
            data = ser.readline().decode().strip()
            if not data:
                continue

            current_time = datetime.now()
            two_data = data.split('|')

            values = [current_time.strftime("%Y-%m-%d %H:%M:%S")] + list(map(float, two_data[1].split(',')))

            if values[-1] == 2:
                data_list.append(values)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    return data_list

data = data_extract('/dev/ttyACM0', 10)
data2 = run('/dev/ttyACM0')

df = pd.DataFrame(data)
df2 = pd.DataFrame(data2)

#print(df)
#print(df2)
     




def insert_analog_data_to_mysql(df):
    try:
        remote = mysql.connector.connect(
            host= 'iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com',
            port= 3306,
            user= 'iot',
            password= '***',
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
            
def insert_position_data_to_mysql(df):
    try:
        remote = mysql.connector.connect(
            host= 'iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com',
            port= 3306,
            user= 'iot',
            password= '***',
            database= 'iot'
        )

        cursor = remote.cursor()

        sql = "INSERT INTO position_data VALUES (%s, %s, %s, %s, %s, %s,)"

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
            
def insert_valve_data_to_mysql(df):
    try:
        remote = mysql.connector.connect(
            host= 'iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com',
            port= 3306,
            user= 'iot',
            password= '***',
            database= 'iot'
        )

        cursor = remote.cursor()

        sql = "INSERT INTO valve_data VALUES (%s, %s, %s)"

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
            
def insert_danger_data_to_mysql(df):
    try:
        remote = mysql.connector.connect(
            host= 'iot.cf64s86023q3.ap-northeast-2.rds.amazonaws.com',
            port= 3306,
            user= 'iot',
            password= '***',
            database= 'iot'
        )

        cursor = remote.cursor()

        sql = "INSERT INTO danger_data VALUES (%s,%s)"

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

            

df_analog = df.iloc[:, :7]
df_position = df2.iloc[:, :8]
df_valve = pd.concat([df2.iloc[:, 0], df2.iloc[:, 8:10]], axis=1)
df_danger = pd.concat([df2.iloc[:, 0], df2.iloc[:,10]], axis=1)


# data upload 
insert_analog_data_to_mysql(df)
insert_position_data_to_mysql(df_position)
insert_valve_data_to_mysql(df_valve)
insert_danger_data_to_mysql(df_danger)
