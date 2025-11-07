# Scripts MATLAB - Visualización y Análisis del Robot

Esta carpeta contiene dos scripts principales que permiten analizar el comportamiento del robot y graficar los resultados experimentales obtenidos en pruebas físicas o simuladas.

---

## 📂 Archivos principales

1. **Graficas_del_sistema_en_posicion_orientacion.m**  
   Script que genera todas las gráficas y la animación 3D del movimiento del robot a partir de datos registrados (archivos CSV).  
   Permite visualizar:
   - Orientación del manipulador en el sistema **local** (móvil).  
   - Orientación del efector final en el sistema **global**.  
   - Posición del efector final y del móvil.  
   - Orientación del móvil en grados.  
   - Trayectorias del efector en X, Y, Z.  
   - Animación 3D de la trayectoria del efector final y el móvil.

2. **workspace_variables.m**  
   Script que **calcula y grafica el espacio de trabajo teórico** del manipulador.  
   Muestra los puntos alcanzables, orientaciones posibles y la posición de referencia (“home”), centrando el espacio en el origen.

---

## ⚙️ Ejecución paso a paso

### 🔹 **Para Graficas_del_sistema_en_posicion_orientacion.m**

1. Abre MATLAB y entra en la carpeta donde se encuentra este script.
2. En la parte del código donde se leen los datos CSV, selecciona la prueba que deseas visualizar.  
   Por defecto aparecen tres archivos de ejemplo:

   ```matlab
   %data = readtable('datosx_y_th_Prueba1.csv');
   data = readtable('datosx_y_th_Prueba2.csv');
   %data = readtable('datosx_y_th_Prueba3.csv');
