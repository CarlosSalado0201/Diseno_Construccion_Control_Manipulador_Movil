/**
 * =================================================================================
 * PROYECTO DE TESIS: "Diseño, construcción y control de un manipulador móvil"
 * AUTOR: Carlos Alberto Salado Chávez
 * INSTITUCIÓN: Universidad Tecnológica de la Mixteca (UTM)
 * FECHA: 2024-2025
 *
 * DESCRIPCIÓN:
 * Este código implementa el control cinemático combinado para un robot móvil
 * omnidireccional (base con 4 ruedas Mecanum) y un brazo manipulador serial
 * de 5 grados de libertad (GDL).
 *
 * El sistema ejecuta un único bucle de control que coordina ambos subsistemas:
 * 1.  BASE MÓVIL: Utiliza odometría y un control cinemático (P) para mover la
 * plataforma a una posición que "ayude" al brazo.
 * 2.  MANIPULADOR: Utiliza cinemática inversa (basada en el método geométrico
 * y desacoplamiento) para posicionar el efector final en el objetivo global.
 *
 * El control combinado se logra al definir el objetivo de la base no como el
 * objetivo final, sino como (Objetivo_Final - Extensión_del_Brazo).
 *
 * Basado en los modelos del Capítulo 4 de la tesis.
 *
 * =================================================================================
 * --- REFERENCIAS DE LA TESIS EN EL CÓDIGO ---
 *
 * Este código implementa directamente los modelos matemáticos del Capítulo 4:
 *
 * 1.  ODOMETRÍA (Medición de la Base):
 * - `Vfm`, `Vlm`, `Wm` (en el loop) implementan la Cinemática Directa de la Base
 * (Tesis, Capítulo 2.7, Ecuaciones 2.54 - 2.56).
 *
 * 2.  CONTROL DE LA BASE (Actuación):
 * - `V1`, `V2`, `V3`, `V4` (en el loop) implementan la Cinemática Inversa de la Base
 * (Tesis, Capítulo 4.2, Ecuación 4.9).
 *
 * 3.  CONTROL COMBINADO (La "Magia"):
 * - `ux = -k * (x - (xd - x_arm))`... implementa la Ley de Control Combinado
 * (Tesis, Capítulo 4.6, Ecuación 4.62).
 * - `calcularX1m_X2m()` implementa la Cinemática Directa del Brazo (simplificada)
 * para obtener `x_arm` y `y_arm` (Tesis, Eq. 4.14, proyectada).
 *
 * 4.  CONTROL DEL MANIPULADOR (Cinemática Inversa del Brazo):
 * - `inversa_3GDL()` implementa el cálculo de `x4, y4, z4` (Eq. 4.61) y
 * el cálculo de `th1, th2, th3` (Eq. 4.55).
 * - `actualizarAngulosLocales()` transforma la orientación deseada
 * Global -> Local (Tesis, Eq. 4.67 - 4.69).
 * - `calcularTh4()` y `calcularTh5()` implementan el resto de la
 * Ecuación 4.55 para la orientación de la muñeca.
 * =================================================================================
 */


// --- LIBRERÍAS ---
// #include "PinChangeInterrupt.h" // Se intentó usar interrupciones, pero se optó por polling
#include "motorControl.h"         // Librería personalizada para el control PID de los motores DC
#include "math.h"                 // Para operaciones matemáticas (sin, cos, atan2, etc.)
#include <SoftwareSerial.h>       // Incluido pero no utilizado en este fragmento
#include <Servo.h>                // Para el control de los servomotores del brazo

// Definir constantes para evitar cálculos repetidos
#define PI_F 3.141592653589793f
#define HALF_PI_F 1.570796326794897f
#define PI 3.141592653589793

// ====================================================================
// Variables de medición y parámetros geométricos del robot
// ====================================================================

float L = 0.097;         // Distancia desde el centro del robot hasta el punto de contacto de las ruedas (m)
float b = 0.14;          // Distancia desde el centro del robot hasta el eje del motor (m)
float r = 0.04;          // Radio de las ruedas (m)
float k = 0.15;          // Ganancia del control cinemático (ajusta la respuesta del movimiento)
float diferenciaTiempo;  // Intervalo de tiempo entre mediciones, usado en la odometría (s)

// ====================================================================
//  Definición de las velocidades y orientaciones deseadas del efector final
// ====================================================================

// Posiciones deseadas (en m)
float xd = -1;       // Componente deseada en eje X
float yd = -1;       // Componente deseada en eje Y
float zd = 0.368;    // Componente deseada en eje Z

// Orientaciones deseadas (en radianes)
double thd    = -0 * 3.1416 / 4;  // Ángulo de orientación general (de la base)
double alphaD =  3.1416 / 4;      // Ángulo de inclinación (roll)
double betaD  =  0;               // Ángulo de cabeceo (pitch)
double gammaD =  0 * 3.1416 / 6;  // Ángulo de giro de la plataforma (yaw)

// --------------------------------------------------------------------
// IMPORTANTE:
// Estas variables (alphaD, betaD, gammaD) deben volver a asignarse dentro del loop(),
// ya que en el cálculo posterior (función actualizarAngulosLocales)
// los valores se transforman al sistema de referencia local del móvil.
// Si no se reestablecen en cada ciclo, el sistema acumularía la transformación
// en cada iteración, afectando la orientación deseada.
// --------------------------------------------------------------------


// ====================================================================
// Variables temporales para registro e impresión de datos
// ====================================================================

// NOTA: 'limite' pertenece a otra parte del código (control angular o restricción de movimiento)
// y no afecta directamente la impresión de datos en el monitor serial.
float limite = 1.5708f;   // 90° en radianes (variable usada en otra sección)

// Variables de almacenamiento temporal
float x = 0;              // Posición X actual o medida
float y = 0;              // Posición Y actual o medida
int imprimir = 300;       // Cantidad de muestras a imprimir o almacenar

// Arreglos para guardar valores durante el experimento o simulación
float xvalor[300];        // Arreglo para registrar la evolución de X
float yvalor[300];        // Arreglo para registrar la evolución de Y
double thvalor[300];      // Arreglo para registrar la orientación (theta)

// Arreglos adicionales (comentados) para registrar cada articulación del manipulador
// double th1valor[600];
// double th2valor[250];
// double th3valor[250];
// double th4valor[250];
// double th5valor[250];

// --------------------------------------------------------------------
// Estos vectores permiten almacenar datos durante la ejecución del programa,
// los cuales pueden imprimirse en el monitor serial o exportarse para análisis
// (por ejemplo, en MATLAB o Python).
// --------------------------------------------------------------------


// ====================================================================
// Variables para el control cinemático del robot móvil
// ====================================================================

// Variables de velocidad deseada en el marco del robot (GLOBAL)
double ux  = 0;  // Componente de velocidad en el eje X del robot
double uy  = 0;  // Componente de velocidad en el eje Y del robot
double uth = 0;  // Velocidad angular (rotacional) deseada del robot

// Variables de velocidad lineal y angular (LOCAL)
double Vf = 0;    // Velocidad de avance (frontal)
double Vl = 0;    // Velocidad lateral
double w  = 0;    // Velocidad angular (rotación alrededor del centro)

// ====================================================================
// Variables para la odometría
// ====================================================================
// Velocidades de cada rueda (calculadas por el modelo cinemático)
double V1 = 0;
double V2 = 0;
double V3 = 0;
double V4 = 0;

// Velocidades medidas (obtenidas de sensores o encoders)
double v1m = 0;
double v2m = 0;
double v3m = 0;
double v4m = 0;

// Velocidades resultantes medidas del robot
double Vfm = 0;   // Velocidad frontal medida
double Vlm = 0;   // Velocidad lateral medida
double Wm  = 0;   // Velocidad angular medida

// ====================================================================
// Variables auxiliares para cálculos intermedios
// ====================================================================
double thm = 0;        // Ángulo de orientación estimado (odometría)
double xp = 0.0;       // Posición X estimada
double yp = 0.0;       // Posición Y estimada
double delta1 = 0.0;   // Variables de corrección o diferencia temporal
double delta2 = 0.0;
double delta3 = 0.0;
double diferentTime = 0; // Diferencia de tiempo entre ciclos (s)

// --------------------------------------------------------------------
// Estas variables se utilizan en el cálculo de la cinemática directa e
// inversa del robot móvil omnidireccional, así como en la estimación de
// su posición mediante odometría. Las variables medidas (v1m–v4m) se
// comparan con las deseadas (V1–V4) para generar la acción de control.
// --------------------------------------------------------------------
// ====================================================================
// VARIABLES PARA EL CONTROL DE LOS MOTORES Y LECTURA DE ENCODERS
// ====================================================================

// --------------------------------------------------------------------
// Variables de temporización
// --------------------------------------------------------------------
// Controlan los tiempos de muestreo y actualización del PID y la lectura
// de encoders. Los tiempos se expresan en milisegundos.
unsigned long lastTime = 0;      // Último tiempo de actualización general
unsigned long sampleTime = 100;  // Periodo de muestreo del control de velocidad (PID)
unsigned long sampleTime2 = 600; // Periodo de muestreo adicional (Manipulador)
unsigned long lastTime2 = 0;
unsigned long lastTime3 = 0;
unsigned long lastTime4 = 0;
unsigned long tiempo = 0;        // Contador global o referencia de tiempo

// --------------------------------------------------------------------
// Instancias del controlador PID para cada motor
// --------------------------------------------------------------------
// Cada objeto 'motorControl' representa un lazo PID independiente asociado
// a uno de los motores del robot. Todos se inicializan con el mismo tiempo
// de muestreo definido arriba.
motorControl motor1(sampleTime);
motorControl motor2(sampleTime);
motorControl motor3(sampleTime);
motorControl motor4(sampleTime);

// --------------------------------------------------------------------
// Configuración de pines PWM y de dirección de los motores
// --------------------------------------------------------------------
// NOTA IMPORTANTE:
// La asignación de pines depende directamente de la conexión física
// entre el Arduino, el puente H DRV8833 y el conector JST de cada motor.
// Por tanto, los valores definidos a continuación deben ajustarse según
// la orientación y el cableado descrito en la página 55 de la tesis.
// --------------------------------------------------------------------

// Motor 1
const int BI1  = 8;
const int BI12 = 9;
// Motor 2
const int BI2  = 10;
const int BI22 = 11;
// Motor 3
const int BI3  = 4;
const int BI32 = 5;
// Motor 4
const int BI4  = 6;
const int BI42 = 7;
// --------------------------------------------------------------------
// Variables de salida del control
// --------------------------------------------------------------------
// Cada variable almacena el valor PWM calculado por el controlador PID
// correspondiente al motor. Estos valores (0–255) se aplican mediante
// la función analogWrite() para accionar el driver de potencia.
int outValue1 = 0;
int outValue2 = 0;
int outValue3 = 0;
int outValue4 = 0;

// --------------------------------------------------------------------
// DESCRIPCIÓN GENERAL:
// Este bloque implementa el control de bajo nivel del sistema, en el cual
// cada motor dispone de un controlador PID independiente. El lazo PID
// ajusta la velocidad medida por el encoder a la velocidad deseada que
// proviene del modelo cinemático del robot móvil omnidireccional.
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// NOTA IMPORTANTE:
// Este bloque se encarga del control de bajo nivel de los actuadores,
// en el que cada motor tiene un lazo PID independiente encargado de
// ajustar su velocidad medida (por encoder) a la velocidad deseada
// proveniente del control cinemático del robot móvil.
// --------------------------------------------------------------------
// ====================================================================
// VARIABLES Y CONFIGURACIÓN DE LOS ENCODERS DE LOS MOTORES
// ====================================================================

// --------------------------------------------------------------------
// NOTA IMPORTANTE:
// La asignación de pines para los encoders depende directamente del
// conexionado físico entre el Arduino, los encoders,
// mediante los conectores JST. El orden de las señales puede variar
// según la orientación del conector y el esquema mostrado en la página 55
// de la tesis. Por tanto, los valores definidos a continuación deben
// ajustarse según la disposición real del hardware.
// --------------------------------------------------------------------

// ====================================================================
// Motor 1
// ====================================================================
const int E1  = 30;  // Entrada A del encoder del motor 1
const int E12 = 32;  // Entrada B del encoder del motor 1

volatile int n1 = 0;     // Contador de pulsos del canal A
volatile int n12 = 0;    // Contador de pulsos del canal B
volatile int ant1 = 0;   // Valor anterior del canal A
volatile int act1 = 0;   // Valor actual del canal A
volatile int ant12 = 0;  // Valor anterior del canal B
volatile int act12 = 0;  // Valor actual del canal B

// ====================================================================
// Motor 2
// ====================================================================
const int E2  = 36;  // Entrada A del encoder del motor 2
const int E22 = 34;  // Entrada B del encoder del motor 2

volatile int n2 = 0;
volatile int n22 = 0;
volatile int ant2 = 0;
volatile int act2 = 0;
volatile int ant22 = 0;
volatile int act22 = 0;

// ====================================================================
// Motor 3
// ====================================================================
const int E3  = 26;  // Entrada A del encoder del motor 3
const int E32 = 28;  // Entrada B del encoder del motor 3

volatile int n3 = 0;
volatile int n32 = 0;
volatile int ant3 = 0;
volatile int act3 = 0;
volatile int ant32 = 0;
volatile int act32 = 0;

// ====================================================================
// Motor 4
// ====================================================================
const int E4  = 24;  // Entrada A del encoder del motor 4
const int E42 = 22;  // Entrada B del encoder del motor 4

volatile int n4 = 0;
volatile int n42 = 0;
volatile int ant4 = 0;
volatile int act4 = 0;
volatile int ant42 = 0;
volatile int act42 = 0;

// --------------------------------------------------------------------
// DESCRIPCIÓN GENERAL:
// Este bloque define las entradas digitales utilizadas para leer los
// pulsos de los encoders de los cuatro motores. Cada encoder dispone
// de dos canales (A y B) para medir el sentido de giro y la velocidad.
// Las variables 'ant' y 'act' se emplean para el conteo incremental
// y para calcular la velocidad angular instantánea de cada rueda.
// --------------------------------------------------------------------

// ====================================================================
// VARIABLES ASOCIADAS AL CONTROL PID DE LOS MOTORES DE C.D.
// ====================================================================

// --------------------------------------------------------------------
// Velocidades angulares individuales de cada rueda
// --------------------------------------------------------------------
// w1–w4 representan la velocidad angular (rad/s) de cada motor o rueda,
// calculadas a partir de los pulsos de los encoders y el periodo de muestreo.
double w1 = 0;
double w2 = 0;
double w3 = 0;
double w4 = 0;

// --------------------------------------------------------------------
// Contadores y variables auxiliares
// --------------------------------------------------------------------
// 'contadorTh' se usa para espaciar temporalmente el cálculo de la orientación
// o actualizar ciertas rutinas del control en intervalos fijos.
// 'var' y 'var2' son variables auxiliares o banderas temporales.
static uint8_t contadorTh = 0;
int var = 0;
int var2 = 0;

// --------------------------------------------------------------------
// Constantes de relación mecánica del sistema de tracción
// --------------------------------------------------------------------
// 'constValue1' y 'constValue2' provienen del análisis del sistema de engranajes
// y la cinemática de transmisión entre el motor y la rueda.
// Estas constantes permiten convertir (ticks / ms) en (rad / s).
//
// constValue = (2 * PI) / (Pulsos_por_Vuelta_Motor * Reduccion)
// Su valor depende del tipo de motorreductor empleado y fue calibrado.
const double constValue1 = 1.386404525; // Para motores 2, 3, 4
const double constValue2 = 1.453768002; // Para motor 1 (calibración distinta)

// --------------------------------------------------------------------
// DESCRIPCIÓN GENERAL:
// Este bloque pertenece al lazo PID de control de velocidad de los motores
// de corriente directa. Las velocidades w1–w4 se obtienen a partir de los
// encoders, corregidas mediante las relaciones mecánicas de transmisión
// definidas por constValue1 y constValue2. Estas variables permiten calcular
// la velocidad real de cada rueda y, por tanto, cerrar el lazo de control
// en términos de la velocidad lineal o angular deseada del robot.
// --------------------------------------------------------------------
// ====================================================================
// SUBSISTEMA DEL MANIPULADOR: CONTROL DE SERVOMOTORES
// ====================================================================

// --------------------------------------------------------------------
// Definición de servomotores (actuadores del manipulador)
// --------------------------------------------------------------------
// Cada objeto 'Servo' representa una articulación del manipulador.
// servo1–servo6 corresponden a los diferentes grados de libertad
// del brazo robótico acoplado al robot móvil.
Servo servo1; // Eje 1 (Base Yaw)
Servo servo2; // Eje 2 (Hombro Pitch)
Servo servo3; // Eje 3 (Codo Pitch)
Servo servo4; // Eje 4 (Muñeca Pitch)
Servo servo5; // Eje 5 (Muñeca Roll)
Servo servo6; // Gripper (Pinza)

// --------------------------------------------------------------------
// Variables de posición actual de cada servo (en grados)
// --------------------------------------------------------------------
// Estas variables representan la posición física actual de cada articulación,
// medida en el rango de 0° a 180°, correspondiente al ángulo real del eje.
//
// NOTA IMPORTANTE:
// Los valores iniciales (pos1–pos5) fueron calibrados experimentalmente.
// Debido a las condiciones físicas de la estructura del manipulador,
// una posición nominal de 90° no correspondía a una postura neutra real.
// Se ajustaron manualmente para obtener una configuración equilibrada
// con la posición de reposo deseada.
int pos1 = 90;
int pos2 = 90;
int pos3 = 110;
int pos4 = 85;
int pos5 = 87;

// --------------------------------------------------------------------
// Variables de posición objetivo de cada servo (en grados)
// --------------------------------------------------------------------
// Se actualizan en cada iteración del loop() por la cinemática inversa.
// Estas posiciones representan el ángulo físico (0-180) objetivo
// que el sistema intenta alcanzar suavemente.
int targetPos1 = 90;
int targetPos2 = 90;
int targetPos3 = 110;
int targetPos4 = 85;
int targetPos5 = 87;

// --------------------------------------------------------------------
// Variables de referencia cinemática (ángulos articulares)
// --------------------------------------------------------------------
// th1–th5 representan los ángulos articulares LÓGICOS (en radianes)
// deseados del manipulador, obtenidos del cálculo cinemático.
double th1 = 0;
double th2 = 0;
double th3 = 0;
double th4 = 0;
double th5 = 0;
double lastTh1 = 0;
double lastTh2 = 0;
double lastTh3 = 0;

// --------------------------------------------------------------------
// Parámetros de control de movimiento suave
// --------------------------------------------------------------------
//   - stepDelay: tiempo entre pasos sucesivos (ms)
//  - stepSize: incremento angular por actualización (grados)
const int stepDelay = 10;  // Intervalo base entre pasos (ms)
const int stepSize  = 1;   // Incremento angular por iteración (°)

// --------------------------------------------------------------------
// Temporización para control no bloqueante de servos
// --------------------------------------------------------------------
// Almacenan el último instante de actualización de cada servo,
// permitiendo un movimiento suave sin bloquear el loop principal.
unsigned long lastUpdateServo1 = 0;
unsigned long lastUpdateServo2 = 0;
unsigned long lastUpdateServo3 = 0;
unsigned long lastUpdateServo4 = 0;
unsigned long lastUpdateServo5 = 0;

// --------------------------------------------------------------------
// DESCRIPCIÓN GENERAL:
// Este bloque implementa el control de bajo nivel de los actuadores
// del manipulador. El sistema mantiene y actualiza las posiciones
// angulares mediante incrementos controlados, basándose en referencias
// articulares (th1-th5) obtenidas de la cinemática inversa del brazo.
// --------------------------------------------------------------------
// ====================================================================
// PARÁMETROS Y VARIABLES CINEMÁTICAS DEL MANIPULADOR
// ====================================================================

// --------------------------------------------------------------------
// Longitudes de los eslabones del brazo manipulador (en metros)
// --------------------------------------------------------------------
// Coinciden con el diseño CAD y la Tesis (Tabla 4.1)
double L1 = 0.15951; // Altura base a eje 2
double L2 = 0.12668; // Eslabon 2 (Hombro)
double L3 = 0.12668; // Eslabon 3 (Codo)
double L5 = 0.18485; // Eslabon 4 + 5 (Muñeca a efector final)

// --------------------------------------------------------------------
// Variables de posición y términos intermedios
// --------------------------------------------------------------------
// Almacenan resultados parciales del cálculo cinemático.
double term = 0;  // Término común en cinemática directa
double x_arm = 0; // Extensión del brazo en X (Global) - para control combinado
double y_arm = 0; // Extensión del brazo en Y (Global) - para control combinado
double x4 = 0;    // Posición X deseada del centro de la muñeca (P4)
double y4 = 0;    // Posición Y deseada del centro de la muñeca (P4)
double z4 = 0;    // Posición Z deseada del centro de la muñeca (P4)
double rm = 0;    // Proyección 2D de la muñeca (sqrt(x4^2 + y4^2))
double zp = 0;    // Altura relativa de la muñeca (z4 - L1)
double r2 = 0;    // Distancia 3D desde eje 2 al eje 4

// --------------------------------------------------------------------
// Variables para el cálculo de ángulos (Cinemática Inversa)
// --------------------------------------------------------------------
// M1–M4: magnitudes intermedias (resultados de Ley de Cosenos)
// alpha, beta, gamma: ángulos auxiliares (Tesis Fig. 4.10)
double M1 = 0;
double M2 = 0;
double M3 = 0;
double M4 = 0;
double alpha = 0;
double beta = 0;
double gamma = 0;

// --------------------------------------------------------------------
// Variables auxiliares para cálculos intermedios
// --------------------------------------------------------------------
double t1 = 0, t2 = 0, d1 = 0, d2 = 0, num = 0, den = 0;
double m1 = 0, m2 = 0, m3 = 0;

// --------------------------------------------------------------------
// Límites de velocidad de los motores de la base (rad/s)
// --------------------------------------------------------------------
// Derivados de la caracterización experimental.
const float pvMax1 = 8.06;
const float pvMax2 = 8.06;
const float pvMax3 = 8.39;
const float pvMax4 = 8.30;

// --------------------------------------------------------------------
// DESCRIPCIÓN GENERAL:
// Este bloque agrupa todas las variables relacionadas con la cinemática
// del manipulador. Las longitudes L1–L5 son medidas físicas.
// Las variables M1–M4, alpha, beta, gamma, etc., se utilizan en los
// cálculos trigonométricos del modelo geométrico inverso.
// --------------------------------------------------------------------
// ====================================================================
// VARIABLES AUXILIARES DE TIEMPO Y MUESTREO
// ====================================================================

// Variable para conteo de muestras o iteraciones del lazo principal.
double muestras = 0;

// Variable para almacenar el intervalo de tiempo (en segundos).
float deltaTime = 0;

// --------------------------------------------------------------------
// DESCRIPCIÓN GENERAL:
// Variables de apoyo. "muestras" cuenta ciclos y "deltaTime"
// mide el tiempo real entre iteraciones para integración y derivación.
// --------------------------------------------------------------------


// ====================================================================
// FUNCIÓN DE INICIALIZACIÓN DEL SISTEMA
// ====================================================================
// Configura pines, controladores PID, servos y comunicación serial.
// ====================================================================

void setup() {

  // ================================================================
  // CONFIGURACIÓN DE PINES PARA MOTORES Y ENCODERS
  // ================================================================
  // Cada motor DC (1–4) tiene dos pines de control (BIx, BIx2)
  // conectados al puente H DRV8833 y dos entradas digitales (Ex, Ex2)
  // provenientes del encoder incremental correspondiente.

  // ---------------- MOTOR 2 ----------------
  pinMode(BI22, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(E2, INPUT_PULLUP);
  pinMode(E22, INPUT_PULLUP);
  digitalWrite(BI22, LOW);
  digitalWrite(BI2, LOW);

  // ---------------- MOTOR 3 ----------------
  pinMode(BI32, OUTPUT);
  pinMode(BI3, OUTPUT);
  pinMode(E3, INPUT_PULLUP);
  pinMode(E32, INPUT_PULLUP);
  digitalWrite(BI32, LOW);
  digitalWrite(BI3, LOW);

  // ---------------- MOTOR 4 ----------------
  pinMode(BI42, OUTPUT);
  pinMode(BI4, OUTPUT);
  pinMode(E4, INPUT_PULLUP);
  pinMode(E42, INPUT_PULLUP);
  digitalWrite(BI42, LOW);
  digitalWrite(BI4, LOW);

  // ---------------- MOTOR 1 ----------------
  pinMode(BI12, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(E1, INPUT_PULLUP);
  pinMode(E12, INPUT_PULLUP);
  digitalWrite(BI12, LOW);
  digitalWrite(BI1, LOW);

  // ================================================================
  // CONFIGURACIÓN DE LOS CONTROLADORES DE MOTOR (PID)
  // ================================================================
  // Se definen los límites de la señal de control (CV = 0-255),
  // los límites de velocidad (PV = rad/s) y las ganancias (Kp, Ti, Td)
  // para cada motor, obtenidos mediante calibración.

  motor1.setCvLimits(255, 0);
  motor1.setPvLimits(pvMax1, 0);
  motor1.setGains(0.43, 0.21, 0.05);  // (Kp, tau_i, tau_d)

  motor2.setCvLimits(255, 0);
  motor2.setPvLimits(pvMax2, 0);
  motor2.setGains(0.48, 0.17, 0.03);  // (Kp, tau_i, tau_d)

  motor3.setCvLimits(255, 0);
  motor3.setPvLimits(pvMax3, 0);
  motor3.setGains(0.53, 0.25, 0.05);  // (Kp, tau_i, tau_d)

  motor4.setCvLimits(255, 0);
  motor4.setPvLimits(pvMax4, 0);
  motor4.setGains(0.45, 0.15, 0.00);  // (Kp, tau_i, tau_d)

  // ================================================================
  // COMUNICACIÓN SERIAL
  // ================================================================
  Serial.begin(9600); // Para debugging y/o Bluetooth

  // ================================================================
  // CONFIGURACIÓN DE SERVOMOTORES DEL MANIPULADOR
  // ================================================================
  // Se conectan los servos a sus pines PWM correspondientes.
  servo1.attach(52);  // Eje 1 (Base)
  servo2.attach(49);  // Eje 2 (Hombro)
  servo3.attach(50);  // Eje 3 (Codo)
  servo4.attach(48);  // Eje 4 (Muñeca Pitch)
  servo5.attach(46);  // Eje 5 (Muñeca Roll)
  servo6.attach(2);   // Pinza (no influye en la cinemática)

  // Inicialización de servos a sus posiciones físicas calibradas
  servo1.write(pos1);
  servo2.write(pos2);
  servo3.write(pos3);
  servo4.write(pos4);
  servo5.write(pos5);

  // ================================================================
  // CONFIGURACIÓN INICIAL DE LOS ENCODERS
  // ================================================================
  // Se leen los valores iniciales de los encoders para tener un
  // punto de referencia para la primera decodificación en cuadratura.
  ant12 = (digitalRead(E1) << 1) | digitalRead(E12);
  ant22 = (digitalRead(E2) << 1) | digitalRead(E22);
  ant32 = (digitalRead(E3) << 1) | digitalRead(E32);
  ant42 = (digitalRead(E4) << 1) | digitalRead(E42);

  // ================================================================
  // SINCRONIZACIÓN INICIAL
  // ================================================================
  // Guarda el tiempo de referencia inicial para el primer cálculo de dt_ms.
  lastTime = millis();

  // ----------------------------------------------------------------
  // ESTADO INICIAL SEGURO:
  // Todos los motores inician detenidos y los servos en su posición
  // neutral. Se garantiza que no haya movimiento al encender el sistema.
  // ----------------------------------------------------------------
}


// ====================================================================
// BUCLE DE CONTROL PRINCIPAL
// ====================================================================
void loop() {
  // ===============================================================
  // 🔹 0. REASIGNACIÓN DE OBJETIVO GLOBAL
  // ===============================================================
  // (Tesis, Nota Importante en Variables)
  // Re-establece las orientaciones deseadas GLOBALES en cada ciclo.
  // La función 'actualizarAngulosLocales' las modificará para
  // convertirlas en objetivos LOCALES para el brazo.
  alphaD = 3.1416 / 4; // 45°
  betaD  = 0;
  gammaD = 0;

  // ===============================================================
  // 🔹 1. LECTURA DE ESTADO (ENCODERS Y SERVOS)
  // ===============================================================
  // En cada iteración del loop principal se actualiza el estado
  // de los sensores (encoders) y los actuadores (servomotores).

  // ---------------- 🧩 Lectura de Encoders -----------------------
  // Guarda el estado anterior ('ant') y lee el estado actual ('act')
  // de los 2 pines (Canal A y Canal B) de cada encoder.
  // (digitalRead(A) << 1) | digitalRead(B) combina las lecturas
  // en un solo número de 2 bits (0-3) para la lógica de cuadratura.

  ant12 = act12;
  act12 = (digitalRead(E1) << 1) | digitalRead(E12);

  ant22 = act22;
  act22 = (digitalRead(E2) << 1) | digitalRead(E22);

  ant32 = act32;
  act32 = (digitalRead(E3) << 1) | digitalRead(E32);

  ant42 = act42;
  act42 = (digitalRead(E4) << 1) | digitalRead(E42);

  // ---------------- ⚙️ Actualización de Servos ------------------
  // Llama a la función de movimiento suave (no bloqueante).
  //
  // 💡 Nota: servo2 (hombro) usa el servo DS51150 (150 kg·cm).
  // Se le aplica un factor ×2.3 en el retardo para ajustar su
  // velocidad y adaptarla a su respuesta física real (calibración).

  updateServo(servo1, pos1, targetPos1, stepDelay,       lastUpdateServo1); // Base
  updateServo(servo2, pos2, targetPos2, stepDelay * 2.3, lastUpdateServo2); // Hombro (servo lento)
  updateServo(servo3, pos3, targetPos3, stepDelay,       lastUpdateServo3); // Codo
  updateServo(servo4, pos4, targetPos4, stepDelay,       lastUpdateServo4); // Muñeca Pitch
  updateServo(servo5, pos5, targetPos5, stepDelay,       lastUpdateServo5); // Muñeca Roll


  // ===============================================================
  // 🔹 2. DECODIFICACIÓN DE ENCODERS EN CUADRATURA
  // ===============================================================
  // Compara el estado anterior (`ant`) con el actual (`act`) para
  // incrementar o decrementar el contador de pulsos (n*2).
  // Esto mide la rotación y la dirección de cada rueda.
  // ---------------------------------------------------------------

  // === MOTOR 1 ===
  if (ant12 == 0 && act12 == 1) n12++;
  if (ant12 == 1 && act12 == 3) n12++;
  if (ant12 == 2 && act12 == 0) n12++;
  if (ant12 == 3 && act12 == 2) n12++;
  if (ant12 == 0 && act12 == 2) n12--;
  if (ant12 == 1 && act12 == 0) n12--;
  if (ant12 == 2 && act12 == 3) n12--;
  if (ant12 == 3 && act12 == 1) n12--;

  // === MOTOR 2 ===
  if (ant22 == 0 && act22 == 1) n22++;
  if (ant22 == 1 && act22 == 3) n22++;
  if (ant22 == 2 && act22 == 0) n22++;
  if (ant22 == 3 && act22 == 2) n22++;
  if (ant22 == 0 && act22 == 2) n22--;
  if (ant22 == 1 && act22 == 0) n22--;
  if (ant22 == 2 && act22 == 3) n22--;
  if (ant22 == 3 && act22 == 1) n22--;

  // === MOTOR 3 ===
  if (ant32 == 0 && act32 == 1) n32++;
  if (ant32 == 1 && act32 == 3) n32++;
  if (ant32 == 2 && act32 == 0) n32++;
  if (ant32 == 3 && act32 == 2) n32++;
  if (ant32 == 0 && act32 == 2) n32--;
  if (ant32 == 1 && act32 == 0) n32--;
  if (ant32 == 2 && act32 == 3) n32--;
  if (ant32 == 3 && act32 == 1) n32--;

  // === MOTOR 4 ===
  if (ant42 == 0 && act42 == 1) n42++;
  if (ant42 == 1 && act42 == 3) n42++;
  if (ant42 == 2 && act42 == 0) n42++;
  if (ant42 == 3 && act42 == 2) n42++;
  if (ant42 == 0 && act42 == 2) n42--;
  if (ant42 == 1 && act42 == 0) n42--;
  if (ant42 == 2 && act42 == 3) n42--;
  if (ant42 == 3 && act42 == 1) n42--;


  // ================================================================
  // 🔹 3. CICLO DE CONTROL PRINCIPAL (10 Hz)
  // ================================================================
  // Este bloque se ejecuta cada 'sampleTime' milisegundos.
  // Es el núcleo del sistema de control combinado.
  // ---------------------------------------------------------------

  if (millis() - lastTime >= sampleTime) {

    // --- 3.1 CÁLCULO DEL TIEMPO ---
    unsigned long dt_ms = millis() - lastTime;
    float dt_s = dt_ms / 1000.0;  // Tiempo transcurrido (s)

    // --- 3.2 ODOMETRÍA (VELOCIDAD DE RUEDAS) ---
    // Convierte los ticks del encoder (n*2) en velocidad angular (rad/s)
    w1 = constValue2 * n12 / dt_ms;
    w2 = constValue1 * n22 / dt_ms;
    w3 = constValue1 * n32 / dt_ms;
    w4 = constValue1 * n42 / dt_ms;

    // Reinicia los contadores para el siguiente ciclo
    n12 = n22 = n32 = n42 = 0;

    // Velocidad tangencial (m/s)
    v1m = w1 * r;
    v2m = w2 * r;
    v3m = w3 * r;
    v4m = w4 * r;

    // --- 3.3 CINEMÁTICA DIRECTA (BASE MÓVIL) ---
    // (Tesis Eq. 2.54-2.56)
    // Calcula la velocidad local (Frontal, Lateral, Angular) de la base
    // a partir de las 4 velocidades de las ruedas medidas.
    Vfm = (v1m + v2m + v3m + v4m) / 4.0;            // Velocidad frontal
    Vlm = (-v1m + v2m + v3m - v4m) / 4.0;           // Velocidad lateral
    Wm  = (-v1m + v2m - v3m + v4m) / (4.0 * (L + b)); // Velocidad angular (rad/s)

    // --- 3.4 ESTIMACIÓN DE POSICIÓN (ODOMETRÍA INTEGRADA) ---
    
    // Calcula la extensión actual del brazo (x_arm, y_arm) en el marco global.
    // Esta función usa (th1, th2, th3, th4) y (thm) para la cinemática directa del brazo.
    calcularX1m_X2m();

    // Integra la velocidad angular (Wm) para obtener la orientación (thm)
       delta1 = (Wm * (millis() - lastTime)) / 1000;
        thm += delta1;
    // Proyecta las velocidades locales (Vfm, Vlm) al marco global usando 'thm'
    // (Tesis Eq. 4.1)
    xp = Vfm * cos(thm) - Vlm * sin(thm); // Velocidad global en X
    yp = Vfm * sin(thm) + Vlm * cos(thm); // Velocidad global en Y

    // Integra las velocidades globales para obtener la posición (x, y)
        delta2 = (xp * (millis() - lastTime)) / 1000;
        x += delta2;
        delta3 = (yp * (millis() - lastTime)) / 1000;
        y += delta3;

    // --- 3.5 REGISTRO DE DATOS (DEPURACIÓN) ---
    // Guarda el estado actual en arreglos para análisis posterior
    xvalor[var]  = x;
    yvalor[var]  = y;
    thvalor[var] = thm;

    /*
    if (var == imprimir) {
        // Sección opcional para imprimir valores por Serial
    }
    */

    // --- 3.6 LEY DE CONTROL CINEMÁTICO (CONTROL COMBINADO) ---
    // (Tesis Eq. 4.62)
    // Calcula el error. Esta es la clave del CONTROL COMBINADO.
    // El objetivo de la base no es (xd, yd), sino (xd - x_arm, yd - y_arm).
    // La base se mueve para "ayudar" al brazo a alcanzar el objetivo final.
    ux  = -k * (x - (xd - x_arm)); // Error de posición en X -> Velocidad deseada X
    uy  = -k * (y - (yd - y_arm)); // Error de posición en Y -> Velocidad deseada Y
    uth = -k * (thm - thd);      // Error de orientación -> Velocidad angular deseada

    // Ajuste de ganancia (Gain Scheduling):
    // Reduce la ganancia 'k' cuando está cerca del objetivo para
    // disminuir la velocidad y evitar oscilaciones (frenado suave).
    if (abs(x - (xd - x_arm)) < 0.15 && abs(y - (yd - y_arm)) < 0.25) {
      k = 0.3;
    }

    // Transformación de velocidades globales deseadas (ux, uy) al marco local (Vf, Vl)
    // (Tesis Eq. 4.8)
    Vf = ux * cos(thm) + uy * sin(thm);
    Vl = -ux * sin(thm) + uy * cos(thm);
    w  = uth;

    // --- 3.7 CINEMÁTICA INVERSA (BASE MÓVIL) ---
    // (Tesis Eq. 4.9)
    // Convierte las velocidades locales deseadas (Vf, Vl, w) en las
    // velocidades tangenciales deseadas para cada una de las 4 ruedas (V1-V4).
    V1 = (Vf - Vl - (L + b) * w);
    V2 = (Vf + Vl + (L + b) * w);
    V3 = (Vf + Vl - (L + b) * w);
    V4 = (Vf - Vl + (L + b) * w);

    // Convierte de m/s a rad/s (velocidad angular de la rueda)
    V1 /= r; V2 /= r; V3 /= r; V4 /= r;

    // --- 3.8 SATURACIÓN (LÍMITES DE VELOCIDAD) ---
    // Limita la velocidad deseada (V1-V4) a los máximos físicos
    // (pvMax*) que los motores pueden alcanzar.
    V1 = constrain(V1, -pvMax1, pvMax1);
    V2 = constrain(V2, -pvMax2, pvMax2);
    V3 = constrain(V3, -7, pvMax3);  // Límite especial calibrado
    V4 = constrain(V4, -pvMax4, pvMax4);

    // --- 3.9 CONTROL PID Y ACTUACIÓN (MOTORES BASE) ---
    // Calcula la señal de control (PWM) para cada motor.
    // Compara la velocidad DESEADA (V*) con la MEDIDA (w*) y calcula el PWM.
    outValue1 = motor1.compute(V1, w1);
    outValue2 = motor2.compute(V2, w2);
    outValue3 = motor3.compute(V3, w3);
    outValue4 = motor4.compute(V4, w4);

    // Envía la señal PWM al Puente H (DRV8833) para cada motor
        if (outValue2 > 0) anticlockwise(BI2, BI22, outValue2);
        if (outValue2 <= 0) clockwise(BI2, BI22, abs(outValue2));
        if (outValue3 > 0) anticlockwise(BI3, BI32, outValue3);
        if (outValue3 <= 0) clockwise(BI3, BI32, abs(outValue3));
        if (outValue4 > 0) anticlockwise(BI4, BI42, outValue4);
        if (outValue4 <= 0) clockwise(BI4, BI42, abs(outValue4));
        if (outValue1 > 0) anticlockwise(BI1, BI12, outValue1);
        if(outValue1 <= 0) clockwise(BI1, BI12, abs(outValue1));
        
    // --- 3.10 CONTROL Y ACTUACIÓN DEL MANIPULADOR ---
    var++; // Incrementa el contador de muestras

    // Calcula los nuevos ángulos objetivo (th1 a th5) para el brazo
    // basándose en la posición actual de la base (thm) y el objetivo global (xd, yd, zd).
    
    // (Tesis Eq. 4.67-4.69) Convierte la orientación global deseada (alphaD,...)
    // en una orientación local relativa a la base (thm).
    actualizarAngulosLocales(thm); 
    
    // (Tesis Eq. 4.61 y 4.55) Calcula th1, th2, th3 (Posicionamiento)
    inversa_3GDL();
    
    // (Tesis Eq. 4.55) Calcula th4 (Orientación)
    calcularTh4();
    
    // (Tesis Eq. 4.55) Calcula th5 (Orientación)
    calcularTh5();

    // Envía los nuevos ángulos (th1-th5) a la función de
    // movimiento suave (no bloqueante) del brazo.
    moverServosSuave();

    // --- 3.11 RESET DE TIEMPO ---
    // Guarda el tiempo actual para el próximo cálculo de 'dt_ms'
    lastTime = millis();
  }
} // FIN DEL VOID LOOP

// ================================================================
// 🔹 4. FUNCIONES DE CINEMÁTICA DEL MANIPULADOR
// ================================================================
// Esta sección implementa la cadena de cálculo de cinemática inversa
// del brazo de 5 GDL, como se describe en la Tesis (Cap 4.4 y 4.6).
// ---------------------------------------------------------------

/**
 * @brief Actualiza los ángulos de orientación deseados (alphaD, betaD, gammaD)
 * de GLOBAL a LOCAL (relativos a la base móvil).
 * @param th Orientación actual de la base móvil (thm).
 * @desc Implementa la Tesis, Eq. 4.67 - 4.69.
 * Toma los 'alphaD' globales (reasignados al inicio del loop)
 * y los recalcula para que sean el objetivo local del brazo.
 */
void actualizarAngulosLocales(double th) {
  // Guarda los valores GLOBALES deseados (asignados al inicio del loop)
  double a_prev = alphaD;
  double b_prev = betaD;
  double g_prev = gammaD;

  // --- Paso 1: Calcular r31 (Tesis Eq. 4.68) ---
  double r31 = sin(a_prev) * sin(g_prev) - cos(a_prev) * cos(g_prev) * sin(b_prev);

  // --- Paso 2: Calcular cosB ---
  double cosB = sqrt(1 - (r31 * r31));

  // --- Paso 3: Actualizar betaD (LOCAL) ---
  // (Tesis Eq. 4.67)
  betaD = -asin(r31);

  // --- Paso 4: Preparar numerador y denominador de gammaD ---
  double term1 = cos(a_prev) * cos(g_prev) - sin(a_prev) * sin(b_prev) * sin(g_prev);
  double term2 = cos(a_prev) * sin(g_prev) + cos(g_prev) * sin(a_prev) * sin(b_prev);

  double num = -(sin(th) * term1 - cos(b_prev) * cos(th) * sin(g_prev));
  double den =  (sin(th) * term2 + cos(b_prev) * cos(g_prev) * cos(th));

  // División explícita entre cosB
  num /= cosB;
  den /= cosB;

  // --- Paso 5: Actualizar gammaD (LOCAL) ---
  // (Tesis Eq. 4.68)
  gammaD = atan2(num, den);

  // --- Paso 6: Calcular alphaD (LOCAL) ---
  // (Tesis Eq. 4.69)
  double numA = -(-sin(b_prev) * sin(th) - cos(b_prev) * sin(a_prev) * cos(th));
  double denA =  cos(a_prev) * cos(b_prev);

  numA /= cosB;
  denA /= cosB;

  alphaD = atan2(numA, denA);
}

/**
 * @brief Calcula los ángulos de posicionamiento (th1, th2, th3).
 * @desc Implementa el método geométrico y de desacoplamiento.
 * 1. Calcula la posición objetivo de la muñeca (x4, y4, z4)
 * restando la longitud del efector (L5) al objetivo global
 * (Tesis Eq. 4.61).
 * 2. Resuelve el problema 2D para (th2, th3) usando ley de cosenos
 * (Tesis Fig. 4.10).
 * 3. Resuelve (th1) usando atan2.
 * (Implementa Tesis Eq. 4.55 para q1, q2, q3)
 */
void inversa_3GDL() {

  // Precalcular funciones trigonométricas
  float c_thm = cos(thm);
  float s_thm = sin(thm);
  float c_alpha = cos(alphaD);
  float s_alpha = sin(alphaD);
  float c_beta = cos(betaD);
  float s_beta = sin(betaD);
  
  // --- 1. Calcular x4, y4, z4 (Tesis Eq. 4.61) ---
  // Posición deseada de la muñeca (P4) en el marco LOCAL de la base
  float dx = xd - x; // Error global en X
  float dy = yd - y; // Error global en Y
  
  x4 = c_thm * dx + s_thm * dy - L5 * s_beta;
  y4 = -s_thm * dx + c_thm * dy + L5 * c_beta * s_alpha;
  z4 = zd - L5 * c_alpha * c_beta;
  
  // --- 2. Método Geométrico (Tesis Fig. 4.10) ---
  rm = sqrt(x4 * x4 + y4 * y4); // Proyección 2D
  zp = z4 - L1;                 // Altura relativa
  r2 = sqrt(rm * rm + zp * zp); // Distancia 3D (eje 2 a 4)
  
  // Ley de cosenos
  float denom1 = 2.0f * L2 * L3;
  float denom2 = 2.0f * L2 * r2;
  
  M1 = ((L2 * L2) + (L3 * L3) - (r2 * r2)) / denom1;
  M2 = ((L2 * L2) + (r2 * r2) - (L3 * L3)) / denom2;
  
  // Limitar valores para evitar errores de dominio en acos()
  M1 = constrain(M1, -1.0f, 1.0f);
  M2 = constrain(M2, -1.0f, 1.0f);
  
  // Calcular ángulos intermedios (alpha, beta, gamma de Fig. 4.10)
  alpha = acos(M1);
  beta  = atan2(zp, rm);
  gamma = acos(M2);
  
  // --- 3. Calcular Ángulos Articulares (th1, th2, th3) ---
  // (Tesis Eq. 4.55)
  th1 = atan2(y4, x4);
  th1 = constrain(th1, -limite, limite);

  // Solución "codo arriba" o "codo abajo"
  if (z4 > 0.4f) { // Calibración de umbral
    th2 = -(-(gamma + beta) + HALF_PI_F); // Codo Arriba
    th3 = -(PI_F - alpha);
  } else {
    th2 = -(-(-gamma + beta) + HALF_PI_F); // Codo Abajo
    th3 = -(PI_F + alpha);
  }

  th2 = constrain(th2, -limite, limite);
}

/**
 * @brief Calcula el ángulo th4 (Muñeca Pitch).
 * @desc Implementa la Tesis Eq. 4.55 para q4.
 * Resuelve la orientación usando los ángulos (th1, th2, th3) ya calculados
 * y la orientación local deseada (alphaD, betaD).
 */
void calcularTh4() {
  // Precalcular funciones trigonométricas
  float c_th1 = cos(th1);
  float s_th1 = sin(th1);
  float th23 = th2 + th3;
  float c_th23 = cos(th23);
  float s_th23 = sin(th23);
  float c_alpha = cos(alphaD);
  float s_alpha = sin(alphaD);
  float c_beta = cos(betaD);
  float s_beta = sin(betaD);
  
  // Calcular M3 (Término de la matriz de rotación, Tesis Eq. 4.53)
  float M3 = (c_th23 * c_alpha * c_beta)
           - (s_th23 * s_beta * c_th1)
           + (s_th23 * c_beta * s_alpha * s_th1);
  
  // Limitar para evitar NaN en acos()
  M3 = constrain(M3, -1.0f, 1.0f);
  
  th4 = acos(M3);
  th4 = constrain(th4, -limite, limite);
}

/**
 * @brief Calcula el ángulo th5 (Muñeca Roll/Yaw).
 * @desc Implementa la Tesis Eq. 4.55 para q5.
 * Resuelve el ángulo final de orientación.
 */
void calcularTh5() {
  // Precalcular funciones trigonométricas
  float c_th1 = cos(th1);
  float s_th1 = sin(th1);
  float c_alpha = cos(alphaD);
  float s_alpha = sin(alphaD);
  float c_beta = cos(betaD);
  float s_beta = sin(betaD);
  float c_gamma = cos(gammaD);
  float s_gamma = sin(gammaD);
  
  // Precalcular términos comunes (de la matriz de rotación, Tesis Eq. 4.53)
  float term1 = c_alpha * s_gamma + c_gamma * s_alpha * s_beta;
  float term2 = c_alpha * c_gamma - s_alpha * s_beta * s_gamma;
  
  // Calcular numerador y denominador para atan2
  float num = -(c_beta * c_gamma * s_th1 - c_th1 * term1);
  float den = c_th1 * term2 + c_beta * s_gamma * s_th1;
  
  // Evitar división por cero (atan2(0,0) es indefinido)
  if (fabs(num) < 1e-6f && fabs(den) < 1e-6f) {
    den = 1e-6f;
  }
  
  th5 = atan2(num, den);
  th5 = constrain(th5, -limite, limite);
}

// ================================================================
// 🔹 5. FUNCIONES UTILITARIAS Y DE ACTUACIÓN
// ================================================================

/**
 * @brief Gira un motor en sentido horario (CW).
 * @param pin1 Pin de dirección 1 del DRV8833
 * @param pin2 Pin de dirección 2 (PWM) del DRV8833
 * @param pwm Valor de 0 a 255
 */
void clockwise(int pin1, int pin2, int pwm) {
  if (pwm <= 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  } else {
    if (pwm > 255) pwm = 255;
    digitalWrite(pin1, LOW);
    analogWrite(pin2, pwm);
  }
}

/**
 * @brief Gira un motor en sentido antihorario (CCW).
 * @param pin1 Pin de dirección 1 (PWM) del DRV8833
 * @param pin2 Pin de dirección 2 del DRV8833
 * @param pwm Valor de 0 a 255
 */
void anticlockwise(int pin1, int pin2, int pwm) {
  if (pwm <= 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  } else {
    if (pwm > 255) pwm = 255;
    digitalWrite(pin2, LOW);
    analogWrite(pin1, pwm);
  }
}

/**
 * @brief Mueve un servo un solo paso (stepSize) hacia su objetivo.
 * @desc Esta es una función NO BLOQUEANTE. Se llama en cada loop,
 * pero solo actúa si ha pasado suficiente tiempo (delayTime).
 * @param s Referencia al objeto Servo
 * @param currentPos Referencia a la variable que guarda la pos. actual (0-180)
 * @param targetPos Posición física objetivo (0-180)
 * @param delayTime Tiempo de espera entre pasos (ms)
 * @param lastUpdateTime Referencia a la variable de tiempo del servo
 */
void updateServo(Servo &s, int &currentPos, int targetPos, int delayTime, unsigned long &lastUpdateTime) {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= delayTime) {
    lastUpdateTime = currentTime;
    if (currentPos < targetPos) {
      currentPos += stepSize;
      if (currentPos > targetPos) currentPos = targetPos;
      s.write(currentPos);
    } else if (currentPos > targetPos) {
      currentPos -= stepSize;
      if (currentPos < targetPos) currentPos = targetPos;
      s.write(currentPos);
    }
  }
}

/**
 * @brief Calcula la extensión (x_arm, y_arm) del brazo en el marco GLOBAL.
 * @desc Implementa la Cinemática Directa del Brazo (Tesis Eq. 4.14)
 * y la proyecta al marco global usando la orientación de la base (thm).
 * Estos valores (x_arm, y_arm) son cruciales para la Ley de Control
 * Combinado (Tesis Eq. 4.62).
 */
void calcularX1m_X2m() {
  // (Tesis Eq. 4.14, término 'r')
  term = L3 * sin(th2 + th3) + L2 * sin(th2) + L5 * sin(th2 + th3 + th4);
  
  // Proyección de la cinemática directa local del brazo al marco global
  // (Rotación 2D usando thm)
  x_arm = cos(thm) * (-cos(th1) * term) - sin(thm) * (-sin(th1) * term);  
  y_arm = sin(thm) * (-cos(th1) * term) + cos(thm) * (-sin(th1) * term);
}

/**
 * @brief Versión de la función map() que usa floats.
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Convierte los ángulos lógicos (th1-th5, en radianes) a
 * ángulos físicos (targetPos1-5, 0-180°) para los servos,
 * aplicando la calibración manual.
 */
void moverServosSuave() {
  float th3_norm = normalizeAngle(th3);

  // 1. Convertir Lógico (rad) a Grados
  float logical1 = th1 * 180.0 / PI;
  float logical2 = th2 * 180.0 / PI;
  float logical3 = th3_norm * 180.0 / PI;
  float logical4 = th4 * 180.0 / PI;
  float logical5 = th5 * 180.0 / PI;

  // 2. Limitar rangos lógicos a +/- 90° (según el modelo)
  logical1 = constrain(logical1, -90, 90);
  logical2 = constrain(logical2, -90, 90);
  logical3 = constrain(logical3, -90, 90);
  logical4 = constrain(logical4, -90, 90);
  logical5 = constrain(logical5, -90, 90);

  // 3. Mapear Lógico (grados) a Físico (0-180)
  // (Valores de calibración manual de la tesis)
  targetPos1 = round(mapFloat(logical1, 90, -90, 183, 3));   // centro real en 93
  targetPos2 = round(mapFloat(logical2, 90, -90, 152, 30));  // (DS51150)
  targetPos3 = round(mapFloat(logical3, 90, -90, 185, 5));   // centro real en 95
  targetPos4 = round(mapFloat(logical4, 90, -90, 171, -9));  // centro real en 81
  targetPos5 = round(mapFloat(logical5, 90, -90, 183, 3));   // centro real en 93

  // 4. Limitar a rango físico del servo [0, 180]
  targetPos1 = constrain(targetPos1, 0, 180);
  targetPos2 = constrain(targetPos2, 0, 180);
  targetPos3 = constrain(targetPos3, 0, 180);
  targetPos4 = constrain(targetPos4, 0, 180);
  targetPos5 = constrain(targetPos5, 0, 180);
}

/**
 * @brief Normaliza un ángulo al rango (-PI, PI].
 */
float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle <= -PI) angle += 2 * PI;
  return angle;
}