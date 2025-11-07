# Diseño Electrónico del Sistema de Control - EAGLE

Este directorio contiene los archivos del diseño electrónico del robot, desarrollados en Autodesk EAGLE / Fusion 360 Electronics.

## 📁 Contenido

- **ControlRobot.sch** → Archivo del esquemático con la distribución de componentes y conexiones eléctricas.  
- **ControlRobot.brd** → Diseño físico de la tarjeta PCB asociada al esquemático.  
- **ControlRobot_3D.step** → Modelo 3D exportado de la placa PCB, útil para integración en el ensamblaje mecánico (Fusion 360 o software CAD).  
- **/Librerias/** → Carpeta con las librerías personalizadas (.lbr) utilizadas en el diseño (sensores, microcontroladores y componentes específicos del sistema).

## 🧰 Requisitos

- Autodesk EAGLE 9.0 o superior **o** Autodesk Fusion 360 (módulo Electronics).  
- Las librerías personalizadas deben estar en la carpeta "Librerias" para que el diseño se abra sin errores.

## ⚙️ Uso

1. Abre el archivo **ControlRobot.sch** (esquemático).  
2. Verifica que esté vinculado al archivo **ControlRobot.brd** (placa PCB). Si ambos se abren juntos, el enlace está activo.  
3. Para visualizar o editar la PCB, abre **ControlRobot.brd** directamente.  
4. Para importar las librerías personalizadas:  
   - En EAGLE: *Library → Open Library Manager → Browse...*  
   - En Fusion: *Tools → Library Manager → Available → Add Local Library*  
5. Para visualizar el modelo 3D de la placa, abre **ControlRobot_3D.step** en tu software CAD preferido (Fusion 360, SolidWorks, FreeCAD, etc.).

## 📝 Notas adicionales

- Este diseño se elaboró con fines académicos y **no incluye archivos Gerber ni de fabricación**.  
- El circuito corresponde a la **etapa de control del robot móvil manipulador omnidireccional**.  
- Los archivos `.sch` y `.brd` están sincronizados (forward/back annotation activa).  
- El modelo STEP se generó automáticamente desde Fusion 360 Electronics, incluyendo la disposición real de componentes.

## 📎 Autoría

Proyecto desarrollado por **Carlos Salado**  
Como parte del trabajo de tesis: *Diseño, simulación y control de un robot móvil manipulador omnidireccional*  
Año: 2025
