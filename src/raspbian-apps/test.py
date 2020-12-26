import logging
import psycopg2
from datetime import date, datetime
conn = psycopg2.connect(host="192.168.0.196", database="greenhouse", user="postgres", password="postgres")

line = "air:26.62 compost:24.19 humidity:93.36 co2:603";
values = line.split(' ')


air = float(values[0].split(':')[1])
compost  = float(values[1].split(':')[1])
humidity = float(values[2].split(':')[1])
co2 = int(values[3].split(':')[1])
time = datetime.now()

sql = f"""INSERT INTO public."data" ("time", air, compost, co2, humidity) VALUES(%s, %s, %s, %s, %s);"""

cur = conn.cursor()
cur.execute(sql, (time, air, compost, co2, humidity))
conn.commit()
cur.close()
