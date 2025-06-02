import matplotlib.pyplot as plt
import csv
import numpy as np

def leer_datos_csv(ruta_archivo, columna_vuelta, columna_radio, columna_angulo):
    vueltas = []
    radios = []
    angulos = []
        
    with open(ruta_archivo, 'r') as archivo_csv:
            lector = csv.reader(archivo_csv)
            
            # Saltar la cabecera si es necesario
            next(lector, None)
            
            for fila in lector:
                try:
                    vuelta = float(fila[columna_vuelta])
                    radio = float(fila[columna_radio])
                    angulo = float(fila[columna_angulo])
                    vueltas.append(vuelta)                    
                    radios.append(radio)
                    angulos.append(angulo)
                except (ValueError, IndexError) as e:
                    print(f"Error al procesar fila: {fila}. Error: {e}")
                    continue

    return vuelta, radios, angulos 

ruta_archivo = "/home/h/csvTries/LidarReadings1.csv"  # Reemplaza con la ruta real
columna_vuelta = 0
columna_radio = 1  # Índice de la columna con los ángulos
columna_angulo = 2  # Índice de la columna con los radios
vuelta, radios, angulos = leer_datos_csv(ruta_archivo, columna_vuelta, columna_radio, columna_angulo)

angulos_radianes = np.array(angulos) * np.pi / 180

colors = np.random.rand(100)  # Random colors

fig, ax = plt.subplots(1, 1, subplot_kw=dict(projection='polar'))
ax.scatter(angulos_radianes, radios, label="Datos del CSV")
ax.legend()
ax.set_title("Gráfico Polar desde CSV")
plt.show()