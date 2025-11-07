import serial
import csv
import os

# Abrir puerto serial
ser = serial.Serial('COM5', 9600, timeout=1)

# Archivo CSV en la misma carpeta
ruta_csv = os.path.join(os.getcwd(), "datos.csv")

datos = []  # Lista temporal para almacenar 210 datos
columnas = []  # Lista de columnas completas

print("Esperando datos...\n")

while True:
    try:
        if ser.in_waiting > 0:
            linea = ser.readline().decode('utf-8').strip()
            if linea:
                try:
                    valor = float(linea.split('=')[1])
                    print("Valor recibido:", valor)
                    datos.append(valor)

                    # Cuando lleguen 210 datos, pasarlos a una columna
                    if len(datos) == 600:
                        columnas.append(datos)
                        datos = []  # Reiniciar para la siguiente columna

                        # Guardar en CSV transpuesto (columnas)
                        max_filas = max(len(col) for col in columnas)
                        with open(ruta_csv, mode='w', newline='') as archivo_csv:
                            escritor = csv.writer(archivo_csv)
                            for i in range(max_filas):
                                fila = []
                                for col in columnas:
                                    if i < len(col):
                                        fila.append(col[i])
                                    else:
                                        fila.append('')
                                escritor.writerow(fila)
                            archivo_csv.flush()
                        print(f"Se guardaron {len(columnas)} columna(s) de 210 datos cada una.")

                except:
                    print("Dato no numérico:", linea)

    except KeyboardInterrupt:
        print("Lectura interrumpida.")
        break
    except Exception as e:
        print("Error:", e)
