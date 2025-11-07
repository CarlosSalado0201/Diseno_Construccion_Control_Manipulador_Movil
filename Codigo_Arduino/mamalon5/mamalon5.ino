  // #include "PinChangeInterrupt.h"
  #include "motorControl.h"
  #include "math.h"
  #include <SoftwareSerial.h>
  // Definir constantes para evitar cálculos repetidos
  #define PI_F 3.141592653589793f
  #define HALF_PI_F 1.570796326794897f
  // Declaracion de variables

  // Variables de medicion
  float L = 0.097;         // Distancia entre el centro del robot y las ruedas
  float b = 0.14;          // Distancia entre el centro del robot y el eje del motor
  float r = 0.04;         // Radio de las ruedas
  float k = 0.15 ;          // Ganancia del control cinemático
  float diferenciaTiempo;  // Diferencia de tiempo para la odometría
  // Velocidades  deseadas
      float xd =-1;
      float yd =-1;
      float zd = 0.368;
      double thd=-0*3.1416/4;
      double alphaD=3.1416/4;
      double betaD = 0;   // 45°
      double gammaD=0*3.1416/6;//Variables para la posicion y orientacion medida
     
     ///////////////////temporales
     float limite = 1.5708f; // 90° en radianes
  float x = 0;
  float y = 0;
  int imprimir=300;
  float xvalor[300];
  float yvalor[300];
  double thvalor[300];
//  double th1valor[600];
  // double th2valor[250];
  // double th3valor[250];
  // double th4valor[250];
  // double th5valor[250];
  


  //Variables para el control cinematico
  double ux = 0;
  double uy = 0;
  double uth = 0;
  double Vf = 0;
  double Vl = 0;
  double w = 0;
  //Variables para odometria
  double V1 = 0;
  double V2 = 0;
  double V3 = 0;
  double V4 = 0;
  // variables medidas
  double v1m = 0;
  double v2m = 0;
  double v3m = 0;
  double v4m = 0;
  double Vfm = 0;
  double Vlm = 0;
  double Wm = 0;

  /// Variables de ayuda
  double thm = 0;
  double xp = 0.0;
  double yp = 0.0;
  double delta1 = 0.0;
  double delta2 = 0.0;
  double delta3 = 0.0;
  double diferentTime = 0;

  // Variables para el control de los motores y encoders
  unsigned long lastTime = 0, sampleTime = 100,sampleTime2 = 600,lastTime2 = 0,lastTime4 = 0,lastTime3 = 0, tiempo = 0;
  motorControl motor2(sampleTime);
  motorControl motor3(sampleTime);
  motorControl motor4(sampleTime);
  motorControl motor1(sampleTime);

  ////////////////////////  MOTOR //////////////////
    // Constantes y pines para el control de los motores y encoders
    const int BI1 = 8;  //moreado //antes 8 y 9
    const int BI12 = 9;
    //Constantes PWM M2
    const int BI2 = 10;  //verde // antes 8 y 9
    const int BI22 = 11;
    //Constantes PWM M3
    const int BI3 = 4;  //blanco//antes 3 y 2
    const int BI32 = 5;
    //Constantes PWM M4
    const int BI4 = 6;  //marillo//antes 5 y 4
    const int BI42 = 7;
    //Valores de salida
    int outValue1 = 0;
    int outValue2 = 0;
    int outValue3 = 0;
    int outValue4 = 0;
    // Variables para el cálculo de la velocidad de los motores
    //Constantes M1
    const int E1 = 30;   //amarillo//antes 20 y 21//
    const int E12 = 32;  //verde////18 por 24 y 19 por 25
    volatile int n1 = 0;
  volatile int n12 = 0;
  volatile int ant1 = 0;
  volatile int act1 = 0;
  volatile int ant12 = 0;
  volatile int act12 = 0;
  //Constantes M2
  const int E2 = 36;   //antes 19 y 18
  const int E22 = 34;  //21 por 23 y 20 por 22
  volatile int n2 = 0;
  volatile int n22 = 0;
  volatile int ant2 = 0;
  volatile int act2 = 0;
  volatile int ant22 = 0;
  volatile int act22 = 0;
  //Constantes M3
  const int E3 = 26;  // antes 14 y 15
  const int E32 = 28;
  volatile int n3 = 0;
  volatile int n32 = 0;
  volatile int ant3 = 0;
  volatile int act3 = 0;
  volatile int ant32 = 0;
  volatile int act32 = 0;
  //Constantes M4
  const int E4 = 24;  //antes 17 y 16
  const int E42 = 22;
  volatile int n4 = 0;
  volatile int n42 = 0;
  volatile int ant4 = 0;
  volatile int act4 = 0;
  volatile int ant42 = 0;
  volatile int act42 = 0;


  double w1 = 0;
  double w2 = 0;
  double w3 = 0;
  double w4 = 0;
      static uint8_t contadorTh = 0;
  int var=0,var2=0;
  const double constValue1 = 1.386404525;
  const double constValue2 = 1.453768002;


  /////////////////////////////////////////////////////////////////manipulador
  #include <Servo.h>
  Servo servo1;
  Servo servo2;
  Servo servo3;
  Servo servo4;
  Servo servo5;
  Servo servo6;
  // Variables para guardar la posición actual (física, 0 a 180) de cada servo
  int pos1 = 90;
  int pos2 = 90;
  int pos3 = 110;
  int pos4 = 85;
  int pos5 = 87;
  // Variables para guardar la posición objetivo de cada servo (calculada en cada loop)
  int targetPos1 = 90;
  int targetPos2 = 90;
  int targetPos3 = 110;
  int targetPos4 = 85;
  int targetPos5 = 87;
  //Configuracion BlueToth
  double th1=0;
  double th2=0;
  double th3=0;
  double th4=0;
  double th5=0;
  double lastTh1=0;
  double lastTh2=0;
  double lastTh3=0;
  // Parámetros para el movimiento suave de los servos
  const int stepDelay = 10;  // Intervalo base entre pasos (ms)
  const int stepSize  = 1;   // Incremento en grados

  // Variables de temporización para actualización no bloqueante de servos
  unsigned long lastUpdateServo1 = 0;
  unsigned long lastUpdateServo2 = 0;
  unsigned long lastUpdateServo3 = 0;
  unsigned long lastUpdateServo4 = 0;
  unsigned long lastUpdateServo5 = 0;
    //parámetros geométricos reales
  // double L1=0.15951;
  // double L2=0.12668;
  // double L3=0.12668;
  // double L5=0.1708;
    //parámetros geométricos simulacion
  double L1=0.15951;
  double L2=0.12668;
  double L3=0.12668;
  double L5=0.18485;

  double term=0;
  double x_arm =0;
  double y_arm =0;

  double x4=0;
  double y4=0;
  double z4=0;
  double rm = 0;
  double zp = 0;
  double r2 = 0;
  double M1 = 0;
  double M2 = 0;
  double alpha = 0;
  double beta  = 0;double gamma = 0;
  // Declarar las variables globales para M3 y M4
  double M3 = 0;
  double M4 = 0;
  // --- Variables auxiliares para cálculos intermedios ---
  double t1 = 0;
  double t2 = 0;
  double d1 = 0;
  double d2 = 0;
  double num = 0;
  double den = 0;
  double m1 = 0;
  double m2 = 0;
  double m3 = 0;

  const float pvMax1 = 8.06;
  const float pvMax2 = 8.06;
  const float pvMax3 = 8.39;
  const float pvMax4 = 8.3;


  double muestras=0;

  ///////////////////
    float deltaTime=0;

  void setup() {

    pinMode(BI22, OUTPUT);
    pinMode(BI2, OUTPUT);
    pinMode(E2, INPUT_PULLUP);
    pinMode(E22, INPUT_PULLUP);
    digitalWrite(BI22, LOW);
    digitalWrite(BI2, LOW);

    pinMode(BI32, OUTPUT);
    pinMode(BI3, OUTPUT);
    pinMode(E3, INPUT_PULLUP);
    pinMode(E32,  INPUT_PULLUP);
    digitalWrite(BI32, LOW);
    digitalWrite(BI3, LOW);

    pinMode(BI42, OUTPUT);
    pinMode(BI4, OUTPUT);
    pinMode(E4, INPUT_PULLUP);
    pinMode(E42, INPUT_PULLUP);
    digitalWrite(BI42, LOW);
    digitalWrite(BI4, LOW);

    pinMode(BI12, OUTPUT);
    pinMode(BI1, OUTPUT);
    pinMode(E1, INPUT_PULLUP);
    pinMode(E12, INPUT_PULLUP);
    digitalWrite(BI12, LOW);
    digitalWrite(BI1, LOW);

    /////////////////// Limites de los motores ///////////////////

    motor2.setCvLimits(255,0);
    motor2.setPvLimits(8.06,0);
    motor2.setGains(0.48, 0.17, 0.03); // (kp,tau,delay)

    motor3.setCvLimits(255, 0);
    motor3.setPvLimits(8.39,0);
    motor3.setGains(0.53, 0.25, 0.05);  // (kp,tau,delay)

    motor4.setCvLimits(255, 0);
    motor4.setPvLimits(8.3, 0);
    motor4.setGains(0.45, 0.15, 0.0);  // (kp,tau,delay)

    motor1.setCvLimits(255,0);
    motor1.setPvLimits(8.06,0);
    motor1.setGains(0.43, 0.21, 0.05); // (kp,tau,delay)
  // límites de velocidad de cada motor

    Serial.begin(9600);
    /////////////////// Configuracion de pines ///////////////////
      servo1.attach(52); // Cambia el pin según tu conexión/// griper grado 1     pin 52
      servo2.attach(49); // Cambia el pin según tu conexión//no/// griper grado 2     pin 49
      servo3.attach(50); // Cambia el pin según tu conexión/// griper grado 3     pin 50
      servo4.attach(48); // Cambia el pin según tu conexión//no /// griper grado 4     pin 48
     servo5.attach(46); // Cambia el pin según tu conexión /// griper grado 5     pin 46
     servo6.attach(2); // Cambia el pin según tu conexión //Pinza este no importa     pin 2

///////sin girar
 //  servo1.attach(51); // Cambia el pin según tu conexión/// griper grado 1     pin 52
 // servo2.attach(47); // Cambia el pin según tu conexión//no/// griper grado 2     pin 49
   //servo3.attach(53); // Cambia el pin según tu conexión/// griper grado 3     pin 50
    //servo4.attach(45); // Cambia el pin según tu conexión//no /// griper grado 4     pin 48
    // servo5.attach(43); // Cambia el pin según tu conexión /// griper grado 5     pin 46
    // servo6.attach(2); // Cambia el pin según tu conexión //Pinza este no importa     pin 2

    // Inicializar todos los servos a 90° (posición física neutra)  
    servo1.write(pos1);
    servo2.write(pos2);
    servo3.write(pos3);
    servo4.write(pos4);
    servo5.write(pos5);
  ant12 = (digitalRead(E1) << 1) | digitalRead(E12);
  ant22 = (digitalRead(E2) << 1) | digitalRead(E22);
  ant32 = (digitalRead(E3) << 1) | digitalRead(E32);
  ant42 = (digitalRead(E4) << 1) | digitalRead(E42);

    lastTime = millis();
  }

  void loop() {
      alphaD=3.1416/4;
       betaD = 0;   // 45°
       gammaD=0*3.1416/6;//Variables para la posicion y orientacion medida
     

    ant22 = act22;
    act22 = (digitalRead(E2) << 1) | digitalRead(E22);
      ant12 = act12;
    act12 = (digitalRead(E1) << 1) | digitalRead(E12);
    ant32 = act32;
    act32 = (digitalRead(E3) << 1) | digitalRead(E32);
    ant42 = act42;
    act42 = (digitalRead(E4) << 1) | digitalRead(E42);
    updateServo(servo1, pos1, targetPos1, stepDelay, lastUpdateServo1);
    updateServo(servo2, pos2, targetPos2, stepDelay * 2.3, lastUpdateServo2);
    updateServo(servo3, pos3, targetPos3, stepDelay, lastUpdateServo3);
    updateServo(servo4, pos4, targetPos4, stepDelay, lastUpdateServo4);
    updateServo(servo5, pos5, targetPos5, stepDelay, lastUpdateServo5);

  /// Motor 1 ///
    // Función de interrupción para el encoder del motor
    if (ant12 == 0 && act12 == 1) n12++;
    if (ant12 == 1 && act12 == 3) n12++;
    if (ant12 == 2 && act12 == 0) n12++;
    if (ant12 == 3 && act12 == 2) n12++;
    if (ant12 == 0 && act12 == 2) n12--;
    if (ant12 == 1 && act12 == 0) n12--;
    if (ant12 == 2 && act12 == 3) n12--;
    if (ant12 == 3 && act12 == 1) n12--;

    //Motor 2 ///

    //Motor 3 ///
    if (ant32 == 0 && act32 == 1) n32++;
    if (ant32 == 1 && act32 == 3) n32++;
    if (ant32 == 2 && act32 == 0) n32++;
    if (ant32 == 3 && act32 == 2) n32++;

    if (ant32 == 0 && act32 == 2) n32--;
    if (ant32 == 1 && act32 == 0) n32--;
    if (ant32 == 2 && act32 == 3) n32--;
    if (ant32 == 3 && act32 == 1) n32--;
    if (ant22 == 0 && act22 == 1) n22++;
    if (ant22 == 1 && act22 == 3) n22++;
    if (ant22 == 2 && act22 == 0) n22++;
    if (ant22 == 3 && act22 == 2) n22++;

    if (ant22 == 0 && act22 == 2) n22--;
    if (ant22 == 1 && act22 == 0) n22--;
    if (ant22 == 2 && act22 == 3) n22--;
    if (ant22 == 3 && act22 == 1) n22--;

    //Motor 4 ///
    if (ant42 == 0 && act42 == 1) n42++;
    if (ant42 == 1 && act42 == 3) n42++;
    if (ant42 == 2 && act42 == 0) n42++;
    if (ant42 == 3 && act42 == 2) n42++;

    if (ant42 == 0 && act42 == 2) n42--;
    if (ant42 == 1 && act42 == 0) n42--;
    if (ant42 == 2 && act42 == 3) n42--;
    if (ant42 == 3 && act42 == 1) n42--;
    /////////////////////////////// Contador ////////////////////////////////
    //////////////////////////////////////////////////
      if (millis() - lastTime >= sampleTime) {
            

    unsigned long dt_ms = millis() - lastTime;
    float dt_s = dt_ms / 1000.0;

        ///// Velocidades medidas
        w2 = constValue1 * n22 / dt_ms;
        w3 = constValue1 * n32 / dt_ms;
        w4 = constValue1 * n42 / dt_ms;
        w1 = constValue2 * n12 / dt_ms;
        n12 = 0;
        n22 = 0;
        n32 = 0;
        n42 = 0;

        v1m = (w1)*r;
        v2m = (w2)*r;
        v3m = (w3)*r;
        v4m = (w4)*r;
        /// Velocidad frontal, lateral y angular
        Vfm = (v1m + v2m + v3m + v4m) / 4;
        Vlm = (-v1m + v2m + v3m - v4m) / 4;
        Wm = (-v1m + v2m - v3m + v4m) / (4 * (L + b));
  
  calcularX1m_X2m();
        /// Integral
          delta1 = (Wm * (millis() - lastTime)) / 1000;
        thm += delta1;
        xp = Vfm * cos(thm) - Vlm * sin(thm);
        yp = Vfm * sin(thm) + Vlm * cos(thm);
        delta2 = (xp * (millis() - lastTime)) / 1000;
        x += delta2;
        delta3 = (yp * (millis() - lastTime)) / 1000;
        y += delta3;


      xvalor[var] = x;
      yvalor[var] = y;
      thvalor[var] = thm;
   //   th1valor[var] = th1;
    //   xvalor[var] = th2;
    //   yvalor[var] = th3;
    //   thvalor[var] = th4;
    //  th1valor[var] = th5;

      // if(var == imprimir){

      //   Serial.print("x: ");
      //     for (int i = 0; i < imprimir; i++) {
      //     Serial.print("xvalor[");
      //     Serial.print(i);
      //     Serial.print("] = ");
      //     Serial.println(xvalor[i]);
      //     }
      //     Serial.print("y: ");
      //     for (int i = 0; i < imprimir; i++) {
      //     Serial.print("yvalor[");
      //     Serial.print(i);
      //     Serial.print("] = ");
      //     Serial.println(yvalor[i]);
      //     }
      //     Serial.print("thm: ");
      //     for (int i = 0; i < imprimir; i++) {
      //     Serial.print("thvalor[");
      //     Serial.print(i);
      //     Serial.print("] = ");
      //     Serial.println(thvalor[i]);
      //     }
      //     // Serial.print("th1: ");
      //     // for (int i = 0; i < imprimir; i++) {
      //     // Serial.print("th1valor[");
      //     // Serial.print(i);
      //     // Serial.print("] = ");
      //     // Serial.println(th1valor[i]);
      //     // }
      //     // // Serial.print("th2: ");
      //     // for (int i = 0; i < imprimir; i++) {
      //     // Serial.print("th2valor[");
      //     // Serial.print(i);
      //     // Serial.print("] = ");
      //     // Serial.println(th2valor[i]);
      //     // }
      //     // Serial.print("th3: ");
      //     // for (int i = 0; i < imprimir; i++) {
      //     // Serial.print("th3valor[");
      //     // Serial.print(i);
      //     // Serial.print("] = ");
      //     // Serial.println(th3valor[i]);
      //     // }
      //     // Serial.print("th4: ");
      //     // for (int i = 0; i < imprimir; i++) {
      //     // Serial.print("th4valor[");
      //     // Serial.print(i);
      //     // Serial.print("] = ");
      //     // Serial.println(th4valor[i]);
      //     // }            
      //     // Serial.print("th5: ");
      //     // for (int i = 0; i < imprimir; i++) {
      //     // Serial.print("th5valor[");
      //     // Serial.print(i);
      //     // Serial.print("] = ");
      //     // Serial.println(th5valor[i]);
      //     // }
      // }

  


          ////// control cinematico
          ux = -k * (x - (xd-x_arm));
          uy = -k * (y - (yd-y_arm));
          uth = -k * (thm - thd);
          if ( abs(x - (xd - x_arm)) < 0.15 && abs(y - (yd - y_arm)) < 0.25) {
        k = 0.3;
    }
  //   if ( abs(x - (xd - x_arm)) < 0.05 && abs(y - (yd - y_arm)) < 0.08 && abs(thm - thd) < 0.02 ) {
  //       V1 = V2 = V3 = V4 = 0;
  // outValue2 = motor2.compute(V2, w2);
  //       outValue3 = motor3.compute(V3, w3);
  //       outValue4 = motor4.compute(V4, w4);
  //       outValue1 = motor1.compute(V1, w1);
  //       /// asignacion de velocidades
  //       if (outValue2 > 0) anticlockwise(BI2, BI22, outValue2);
  //       if (outValue2 <= 0) clockwise(BI2, BI22, abs(outValue2));
  //       if (outValue3 > 0) anticlockwise(BI3, BI32, outValue3);
  //       if (outValue3 <= 0) clockwise(BI3, BI32, abs(outValue3));
  //       if (outValue4 > 0) anticlockwise(BI4, BI42, outValue4);
  //       if (outValue4 <= 0) clockwise(BI4, BI42, abs(outValue4));
  //       if (outValue1 > 0) anticlockwise(BI1, BI12, outValue1);
  //       if(outValue1 <= 0) clockwise(BI1, BI12, abs(outValue1));    }
  //   else{
        Vf = ux * cos(thm) + uy * sin(thm);
        Vl = -ux * sin(thm) + uy * cos(thm);
        w = uth;
        //////Odometria del omnidirecconal
        V1 = (Vf - Vl - ((L + b) * w));
        V2 = (Vf + Vl + ((L + b) * w));
        V3 = (Vf + Vl - ((L + b) * w));
        V4 = (Vf - Vl + ((L + b) * w));
        V1 = V1 / r;
        V2 = V2 / r;
        V3 = V3 / r;
        V4 = V4 / r;

        /// restriccion para que no se salga del control
    V1 = constrain(V1, -pvMax1, pvMax1); 
    V2 = constrain(V2, -pvMax2, pvMax2);
    V3 = constrain(V3, -7, pvMax3);
    V4 = constrain(V4, -pvMax4, pvMax4);

        ///Funcion de control
        
        outValue2 = motor2.compute(V2, w2);
        outValue3 = motor3.compute(V3, w3);
        outValue4 = motor4.compute(V4, w4);
        outValue1 = motor1.compute(V1, w1);
        /// asignacion de velocidades
        if (outValue2 > 0) anticlockwise(BI2, BI22, outValue2);
        if (outValue2 <= 0) clockwise(BI2, BI22, abs(outValue2));
        if (outValue3 > 0) anticlockwise(BI3, BI32, outValue3);
        if (outValue3 <= 0) clockwise(BI3, BI32, abs(outValue3));
        if (outValue4 > 0) anticlockwise(BI4, BI42, outValue4);
        if (outValue4 <= 0) clockwise(BI4, BI42, abs(outValue4));
        if (outValue1 > 0) anticlockwise(BI1, BI12, outValue1);
        if(outValue1 <= 0) clockwise(BI1, BI12, abs(outValue1));
        

        var++;
    //   Serial.println(var);
      actualizarAngulosLocales(thm);
      inversa_3GDL();
      calcularTh4();
      calcularTh5();

        moverServosSuave();   
            lastTime = millis();

      //}
      }
    }

  void calcularTh4() {
    // Precalcular todas las funciones trigonométricas
    float c_th1 = cos(th1);
    float s_th1 = sin(th1);
    float th23 = th2 + th3;
    float c_th23 = cos(th23);
    float s_th23 = sin(th23);
    float c_alpha = cos(alphaD);
    float s_alpha = sin(alphaD);
    float c_beta = cos(betaD);
    float s_beta = sin(betaD);
    
    // Calcular M3
    float M3 = (c_th23 * c_alpha * c_beta) -
              (s_th23 * s_beta * c_th1) +
              (s_th23 * c_beta * s_alpha * s_th1);
    
    // Limitar para evitar NaN
    M3 = constrain(M3, -1.0f, 1.0f);
    
    th4 = acos(M3);
    th4 = constrain(th4, -limite, limite);

  }
  void inversa_3GDL() {
    

    // Precalcular funciones trigonométricas
    float c_thm = cos(thm);
    float s_thm = sin(thm);
    float c_alpha = cos(alphaD);
    float s_alpha = sin(alphaD);
    float c_beta = cos(betaD);
    float s_beta = sin(betaD);
    
    // Calcular x4, y4, z4
    float dx = xd - x;
    float dy = yd - y;
    
    x4 = c_thm * dx + s_thm * dy - L5 * s_beta;
    y4 = -s_thm * dx + c_thm * dy + L5 * c_beta * s_alpha;
    z4 = zd - L5 * c_alpha * c_beta;
    
    // Calcular rm, zp, r2
    rm = sqrt(x4 * x4 + y4 * y4);
    zp = z4 - L1;
    r2 = sqrt(rm * rm + zp * zp);
    
    // Ley de cosenos (evitar divisiones repetidas)
    float denom1 = 2.0f * L2 * L3;
    float denom2 = 2.0f * L2 * r2;
    
    M1 = ((L2 * L2) + (L3 * L3) - (r2 * r2)) / denom1;
    M2 = ((L2 * L2) + (r2 * r2) - (L3 * L3)) / denom2;
    
    // Limitar valores
    M1 = constrain(M1, -1.0f, 1.0f);
    M2 = constrain(M2, -1.0f, 1.0f);
    
    // Calcular ángulos
    alpha = acos(M1);
    beta = atan2(zp, rm);
    gamma = acos(M2);
    
      th1 = atan2(y4, x4);
      
      // Limitar th1

      th1 = constrain(th1, -limite, limite);
    // Serial.println(z4);
      if (z4 > 0.4) {
        th2 = -(-(gamma + beta) + 1.5708f); // math.pi / 2
        th3 = -(3.1416f - alpha);           // math.pi - alpha
      } else {
        th2 = -(-(-gamma + beta) + 1.5708f); // -(-gamma + beta) + PI/2
        th3 = -(3.1416f + alpha);            // math.pi + alpha
    }
      th2 = constrain(th2, -limite, limite);

  }
void actualizarAngulosLocales(double th) {
    // Variables previas

    double a_prev = alphaD;
    double b_prev = betaD;
    double g_prev = gammaD;

    // === Paso 1: Calcular r31 ===
    double r31 = sin(a_prev) * sin(g_prev) - cos(a_prev) * cos(g_prev) * sin(b_prev);

    // === Paso 2: Calcular cosB ===
    double cosB = sqrt(1 - (r31 * r31));

    // === Paso 3: Actualizar betaD ===
    betaD = -asin(r31);

    // === Paso 4: Preparar numerador y denominador de gammaD ===
    double term1 = cos(a_prev) * cos(g_prev) - sin(a_prev) * sin(b_prev) * sin(g_prev);
    double term2 = cos(a_prev) * sin(g_prev) + cos(g_prev) * sin(a_prev) * sin(b_prev);

    double num = -(sin(th) * term1 - cos(b_prev) * cos(th) * sin(g_prev));
    double den =  (sin(th) * term2 + cos(b_prev) * cos(g_prev) * cos(th));

    // División explícita entre cosB
    num /= cosB;
    den /= cosB;

    // === Paso 5: Actualizar gammaD ===
    gammaD = atan2(num, den);

    //=== Paso 6: (Opcional) Calcular alphaD como en Coppelia ===
    
    double numA = -(-sin(b_prev) * sin(th) - cos(b_prev) * sin(a_prev) * cos(th));
    double denA =  cos(a_prev) * cos(b_prev);

    numA /= cosB;
    denA /= cosB;

    alphaD = atan2(numA, denA);
    

    // // Debug
    // Serial.print("thm: "); Serial.print(th);
    // Serial.print("  alphaD: "); Serial.print(alphaD);
    // Serial.print("  betaD: "); Serial.print(betaD);
    // Serial.print("  gammaD: "); Serial.print(gammaD);
    // Serial.print("  cosB: "); Serial.print(cosB);
    // Serial.print("  r31: "); Serial.println(r31);
}


  void calcularTh5() {
    // Precalcular todas las funciones trigonométricas una sola vez
    float c_th1 = cos(th1);
    float s_th1 = sin(th1);
    float c_alpha = cos(alphaD);
    float s_alpha = sin(alphaD);
    float c_beta = cos(betaD);
    float s_beta = sin(betaD);
    float c_gamma = cos(gammaD);
    float s_gamma = sin(gammaD);
    
    // Precalcular términos comunes
    float term1 = c_alpha * s_gamma + c_gamma * s_alpha * s_beta;
    float term2 = c_alpha * c_gamma - s_alpha * s_beta * s_gamma;
    
    // Calcular numerador y denominador
    float num = -(c_beta * c_gamma * s_th1 - c_th1 * term1);
    float den = c_th1 * term2 + c_beta * s_gamma * s_th1;
    
    // Evitar división por cero
    if (fabs(num) < 1e-6f && fabs(den) < 1e-6f) {
      den = 1e-6f;
    }
    
    th5 = atan2(num, den);
    th5 = constrain(th5, -limite, limite);

  }


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



  // Función para actualizar un servo de forma no bloqueante
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
  void calcularX1m_X2m() {
    term = L3 * sin(th2 + th3) + L2 * sin(th2) + L5 * sin(th2 + th3 + th4);
    x_arm = cos(thm) * (-cos(th1) * term) - sin(thm) * (-sin(th1) * term);  
    y_arm = sin(thm) * (-cos(th1) * term) + cos(thm) * (-sin(th1) * term);
    
  }
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) return out_min;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
  void moverServosSuave() {
    float th3_norm = normalizeAngle(th3);

    // 1. Convertir a grados
    float logical1 = th1 * 180.0 / PI;
    float logical2 = th2 * 180.0 / PI;
    float logical3 = th3_norm * 180.0 / PI;
    float logical4 = th4 * 180.0 / PI;
    float logical5 = th5 * 180.0 / PI;

    // 2. Limitar rangos lógicos
    logical1 = constrain(logical1, -90, 90);
    logical2 = constrain(logical2, -90, 90);
    logical3 = constrain(logical3, -90, 90);
    logical4 = constrain(logical4, -90, 90);
    logical5 = constrain(logical5, -90, 90);

    // 3. Mapear a físico
targetPos1 = round(mapFloat(logical1, 90, -90, 183, 3));   // centro real en 93
targetPos2 = round(mapFloat(logical2, 90, -90, 152, 30));  // este lo dejaste sin corrección
targetPos3 = round(mapFloat(logical3, 90, -90, 185, 5));   // centro real en 95
targetPos4 = round(mapFloat(logical4, 90, -90, 171, -9));  // centro real en 81
targetPos5 = round(mapFloat(logical5, 90, -90, 183, 3));   // centro real en 93

    // 4. Limitar a [0,180]
    targetPos1 = constrain(targetPos1, 0, 180);
    targetPos2 = constrain(targetPos2, 0, 180);
    targetPos3 = constrain(targetPos3, 0, 180);
    targetPos4 = constrain(targetPos4, 0, 180);
    targetPos5 = constrain(targetPos5, 0, 180);

  }
float normalizeAngle(float angle) {
  while (angle > PI) angle -= 2 * PI;
  while (angle <= -PI) angle += 2 * PI;
  return angle;
}

