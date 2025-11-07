# 🤖 Diseño, Construcción y Control de un Manipulador Móvil

**Autor:** Carlos Alberto Salado Chávez  
**Institución:** Universidad Tecnológica de la Mixteca – Ingeniería en Mecatrónica  
**Directores:** Dr. Oscar David Ramírez Cárdenas | Dr. Miguel Alberto Domínguez Gurría  
**Fecha:** Noviembre de 2025  

---

## 📘 Descripción general

Este repositorio contiene todo el material correspondiente al proyecto de tesis  
**“Diseño, construcción y control de un manipulador móvil”**, desarrollado en la Universidad Tecnológica de la Mixteca.

El proyecto abarca desde el modelado CAD y la simulación en CoppeliaSim,  
hasta la construcción física del robot y su control cinemático implementado en Arduino.  

El manipulador móvil diseñado utiliza **ruedas omnidireccionales** y un **brazo robótico de 5 grados de libertad**,  
controlado mediante un modelo cinemático combinado que permite posicionar el efector final en el espacio tridimensional.

---

## 🎯 Objetivo general

Diseñar y construir un robot manipulador móvil con ruedas omnidireccionales,  
desarrollando un modelo de control cinemático combinado que coordine los movimientos  
del sistema móvil y del manipulador para alcanzar puntos específicos en el espacio tridimensional.

---

## ⚙️ Estructura del repositorio

```bash
DISENO_CONSTRUCCION_Y_CONTROL_DE_UN_MANIPULADOR_MOVIL/
│
├── CAD_Modelo/                  # Modelado 3D en formato STEP
│   ├── Dibujos_Tecnicos/        # Planos técnicos en PDF
│   └── README_CAD.txt
│
├── Codigo_Arduino/              # Programación del sistema en Arduino
│   ├── Control_Cinematico_Robot_Manipulador_CAOS/
│   ├── mamalon5/
│   └── Lectura_Grafica_monitor_Serial.py
│
├── CoppeliaSim/                 # Simulación del robot en entorno virtual
│
├── Documento/                   # Tesis completa en formato PDF
│
├── PCB_Eagle/                   # Diseño electrónico de la PCB y librerías
│
├── Resultados_Matlab/           # Datos experimentales y scripts de análisis
│
├── VIDEOS/                      # Pruebas experimentales y simulaciones
│
└── README.md                    # Este archivo
