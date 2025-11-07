--[[
=================================================================================
PROYECTO DE TESIS: "Dise?o, construccion y control de un manipulador movil"
AUTOR: Carlos Alberto Salado Chavez
SIMULACION: CoppeliaSim (Script LUA)

DESCRIPCION:
Este script LUA es el "gemelo digital" del c?digo C++ (Arduino).
Implementa exactamente el mismo modelo y bucle de control cinematico combinado
descrito en el Cap?tulo 4 de la tesis, pero dentro del entorno de simulacion
CoppeliaSim.

FUNCIONAMIENTO:
1.  init(): Obtiene los "handles" (punteros) a todos los objetos de la simulacion
    (motores, articulaciones, objetivos, graficas).
2.  actuation(): Es el "void loop()". Se ejecuta en cada paso de simulaci?n:
    a. LEE el estado (posicion/orientacion actual de la base y del objetivo).
    b. CALCULA la Cinematica Directa del Brazo (Eq. 4.14) -> `Calcular_tx_ty_con_cinematica_directa()`
    c. CALCULA la transformacion de orientacion Global a Local (Eq. 4.67-4.69) -> `actualizarAngulosLocales()`
    d. CALCULA el error y las velocidades de la base (Control Combinado Eq. 4.62) -> `controlCinematicoCarrito()`
    e. CALCULA los angulos del brazo (Cinematica Inversa Brazo Eq. 4.55) -> `controlCinematicoManipulador()`
    f. ACTUA moviendo los motores de la base (`w1-w4`) y los servos del brazo (`th1-th5`).
3.  sensing(): Envia los datos calculados a las graficas para visualizaci?n en tiempo real.
=================================================================================
--]]

--[[
=================================================================================
FUNCI?N: sysCall_init()
PROPOSITO: Se ejecuta una sola vez al iniciar la simulacion.
TAREA: Obtener los "handles" (manejadores o punteros) de todos los objetos
       en la escena que necesitamos controlar o leer, e inicializar las graficas.
=================================================================================
--]]
function sysCall_init()
    -- === 1. OBTENER HANDLES DE ARTICULACIONES (MANIPULADOR) ===
    -- Obtiene los 5 ejes (servos) del brazo manipulador
    q1 = sim.getObject('./ServoMotor1')
    q2 = sim.getObject('./ServoMotor2')
    q3 = sim.getObject('./ServoMotor3')
    q4 = sim.getObject('./ServoMotor4')
    q5 = sim.getObject('./ServoMotor5')

    -- === 2. OBTENER HANDLES DE OBJETOS 'DUMMY' (OBJETIVOS) ===
    -- 'punto_f' es el "Dummy" (objeto invisible) que movemos en la escena
    -- para definir la posicion y orientacion deseada (xd, yd, zd, alphaD, betaD, gammaD).
    punto_f = sim.getObject('/Posicion_deseada')
    
    -- Otros Dummies para visualizacion y depuracion
    pd2 = sim.getObject('/PD2')
    pvr = sim.getObject('/PdVr')
    p4 = sim.getObject('/P40') -- Representa el P4 de la cinematica (centro de la mu?eca)

    -- === 3. OBTENER HANDLES DE MOTORES (BASE MOVIL) ===
    -- Obtiene las 4 ruedas (motores) de la base omnidireccional
    M1c = sim.getObject('./Motor_Izquierdo_Frente')
    M3c = sim.getObject('./Motor_Izquierdo_Atras_')
    M2c = sim.getObject('./Motor_Derecho_Frente')
    M4c = sim.getObject('./Motor_Derecho_Atras')

    -- === 4. OBTENER HANDLES DE CUERPOS PRINCIPALES ===
    -- 'pc' es el handle del cuerpo principal de la base m?vil (el chasis)
    pc = sim.getObjectHandle('./Pc') 
    -- 'pm' es el handle del efector final (pinza)
    pm = sim.getObjectHandle('./PM')
    -- 'P' es el handle de un Dummy de referencia en el efector final
    P = sim.getObjectHandle('./Posicion_de_Referencia')

    -- === 5. INICIALIZACI?N DE VARIABLES DE ?NGULO ===
    -- Inicializa todos los ?ngulos l?gicos del brazo en 0 radianes
    th1 = 0
    th2 = 0
    th3 = 0
    th4 = 0
    th5 = 0
    thm1 = 0 -- Variable auxiliar

    -- L?mites de articulaci?n (en radianes). Usados para 'constrain'
    th1_max = math.rad(90)
    th1_min = math.rad(-90)
    th2_max = math.rad(90)
    th2_min = math.rad(-90)
    th3_max = math.rad(90)
    th3_min = math.rad(-90)
    th4_max = math.rad(90)
    th4_min = math.rad(-90)
    th5_max = math.rad(90)
    th5_min = math.rad(-90)

    -- === 6. CONFIGURACI?N DE GR?FICAS (VISUALIZACI?N) ===
    -- Este bloque configura todas las gr?ficas de CoppeliaSim para
    -- mostrar los datos en tiempo real.

    -- --- Gr?fica 1: Posici?n XYZ del Efector Final (Global) ---
    graf = sim.getObject('/Graph4')
    posx = sim.addGraphStream(graf, 'x_arm', 'm', 0, {1, 0, 0}, 0) -- Rojo
    posy = sim.addGraphStream(graf, 'y_arm', 'm', 0, {0, 1, 0}, 0) -- Verde
    posz = sim.addGraphStream(graf, 'z_arm', 'm', 0, {0, 0, 1}, 0) -- Azul
    sim.addGraphCurve(graf, 'Posicion del robot', 3, {posx, posy, posz}, {0, 0, 0}, 'm', 0, {1, 0, 0}, 3)

    -- --- Gr?fica 2: Comparativa XY (Efector vs Base) ---
    graf_x_y = sim.getObject('/Graph')
    graf_x_y_fake = sim.getObject('/Graph6') -- (Gr?fica auxiliar)
    posx_Efector = sim.addGraphStream(graf_x_y, 'x_arm', 'm', 0, {1, 0, 0}, 0) -- Rojo
    posx_Movil   = sim.addGraphStream(graf_x_y, 'x_mo', 'm', 0, {0, 0, 0}, 0) -- Negro
    posy_Efector = sim.addGraphStream(graf_x_y, 'y_arm', 'm', 0, {0, 1, 0}, 0) -- Verde
    posy_Movil   = sim.addGraphStream(graf_x_y, 'y_mo', 'm', 0, {0, 0, 1}, 0) -- Azul
    -- (Streams 'fake' para configurar la curva 3D)
    posz_fake = sim.addGraphStream(graf_x_y_fake, 'Fakeeee1', 'm', 0, {0, 0, 0}, 0)
    posx_fake = sim.addGraphStream(graf_x_y_fake, 'Fakeeee2', 'm', 0, {0.6, 0, 0}, 0)
    posy_fake = sim.addGraphStream(graf_x_y_fake, 'Fakeeee3', 'm', 0, {0, 0.6, 0}, 0)
    sim.addGraphCurve(graf_x_y_fake, 'Posicion del robot', 3, {posx_fake, posy_fake, posz_fake}, {0, 0, 0}, 'm', 0, {0, 0, 0}, 3)

    -- --- Gr?fica 3: Orientaci?n (Theta) de la Base M?vil ---
    graf_th = sim.getObject('/Graph3')
    posth_movil = sim.addGraphStream(graf_th, 'g_m', 'deg', 0, {0, 0, 1}, 0) -- Azul

    -- --- Gr?fica 4: Orientaci?n RPY del Efector Final (Global) ---
    graf2 = sim.getObject('/Graph2')
    posalphaRobot = sim.addGraphStream(graf2, 'a_arm', 'deg', 0, {1, 0, 0}, 0) -- Rojo (Roll)
    posbetaRobot  = sim.addGraphStream(graf2, 'b_arm', 'deg', 0, {0, 1, 0}, 0) -- Verde (Pitch)
    posgammaRobot = sim.addGraphStream(graf2, 'g_arm', 'deg', 0, {0, 0, 1}, 0) -- Azul (Yaw)

    -- --- Gr?fica 5: Orientaci?n RPY del Efector Final (Local, relativo a la base) ---
    graf5 = sim.getObject('/Graph5')
    posalphaMovil = sim.addGraphStream(graf5, 'a_m', 'deg', 0, {1, 0, 0}, 0) -- Rojo (Roll)
    posbetaMovil  = sim.addGraphStream(graf5, 'b_m', 'deg', 0, {0, 1, 0}, 0) -- Verde (Pitch)
    posgammaMovil = sim.addGraphStream(graf5, 'g_m', 'deg', 0, {0, 0, 1}, 0) -- Azul (Yaw)
end


--[[
=================================================================================
FUNCI?N: sysCall_actuation()
PROP?SITO: Es el "void loop()" de la simulaci?n. Se ejecuta en cada paso.
TAREA: Leer sensores, calcular toda la cinem?tica combinada y enviar
       los comandos de velocidad (base) y posici?n (brazo) a los actuadores.
=================================================================================
--]]
function sysCall_actuation()
    pi = math.pi -- Constante PI

    -- === FASE 1: LECTURA DE SENSORES Y ESTADO ===

    -- --- 1.1 Leer Objetivo Global (del Dummy 'punto_f') ---
    -- Leer la posici?n (xd, yd, zd)
    td = sim.getObjectPosition(punto_f, -1) -- -1 = Coordenadas globales
    xd = td[1]
    yd = td[2]
    zd = td[3]
    
    -- Leer la orientaci?n (alphaD, betaD, gammaD)
    to = sim.getObjectOrientation(punto_f, -1) -- -1 = Coordenadas globales
    alphaD = to[1]
    betaD  = to[2]
    gammaD = to[3]
    
    -- Objetivo de orientaci?n de la base (thd) - (fijado a pi/6 o 30 grados)
    thd = pi / 6 

    -- --- 1.2 Leer Estado Actual de la Base M?vil ---
    -- 'Posc' es la posici?n del chasis ('pc') en coordenadas globales
    Posc = sim.getObjectPosition(pc, -1)
    x = Posc[1] -- Posici?n actual X
    y = Posc[2] -- Posici?n actual Y
    -- 'Oric' es la orientaci?n del chasis ('pc') en coordenadas globales
    Oric = sim.getObjectOrientation(pc, -1)
    th = Oric[3] -- Orientaci?n actual Theta (Yaw)

    -- --- 1.3 Leer Estado Actual del Efector Final (para depuraci?n y gr?ficas) ---
    -- 'Grap' y 'Grap2' leen la pos/orientaci?n del efector final ('P') en modo global
    Grap = sim.getObjectPosition(P, -1)
    Grap2 = sim.getObjectOrientation(P, -1)
    -- 'Grap3' lee la orientaci?n del efector final ('P') relativa al chasis ('pc')
    Grap3 = sim.getObjectOrientation(P, pc)

    -- (Impresiones en consola para depuraci?n, comentadas)
    -- print("-----Movil------")
    -- print("Posicion en x:", Posc[1],"Posicion en y:", Posc[2],"th1",math.deg(th1),"th2: ", math.deg(th2),"th3: ", math.deg(th3),"th4: ", math.deg(th4),"th5",math.deg(th5))
    
    -- === FASE 2: C?LCULO DEL CONTROL COMBINADO ===
    -- Esta es la implementaci?n directa del bucle de control del Arduino.
    
    -- 1. Cinem?tica Directa Brazo (Tesis Eq. 4.14)
    -- Calcula la extensi?n actual del brazo (xm1, ym1) en el marco global.
    Calcular_tx_ty_con_cinematica_directa()
    
    -- 2. Transformaci?n de Orientaci?n (Tesis Eq. 4.67-4.69)
    -- Convierte la orientaci?n deseada (alphaD, betaD, gammaD) de global a local.
    actualizarAngulosLocales()
    
    -- 3. Control Cinem?tico Base (Tesis Eq. 4.62 y 4.9)
    -- Calcula las velocidades de rueda (w1-w4) para la base.
    controlCinematicoCarrito()
    
    -- 4. Cinem?tica Inversa Brazo (Tesis Eq. 4.55 y 4.61)
    -- Calcula los ?ngulos (th1-th5) para el brazo.
    controlCinematicoManipulador()

    -- === FASE 3: ACTUACI?N ===
    
    -- --- 3.1 Actuaci?n del Brazo ---
    -- Env?a los ?ngulos calculados (en radianes) a las articulaciones del brazo.
    sim.setJointTargetPosition(q1, th1)
    sim.setJointTargetPosition(q2, th2)
    sim.setJointTargetPosition(q3, th3)
    sim.setJointTargetPosition(q4, th4)
    sim.setJointTargetPosition(q5, th5)

    -- --- 3.2 Actuaci?n de la Base ---
    -- Env?a las velocidades angulares (en rad/s) a las ruedas.
    -- NOTA: w1 y w3 son negativos debido a la orientaci?n de los
    -- motores en el modelo de CoppeliaSim para que giren en la
    -- direcci?n correcta.
    sim.setJointTargetVelocity(M1c, -w1)
    sim.setJointTargetVelocity(M2c, w2)
    sim.setJointTargetVelocity(M3c, -w3)
    sim.setJointTargetVelocity(M4c, w4)
end

--[[
=================================================================================
FUNCI?N: sysCall_sensing()
PROP?SITO: Se ejecuta despu?s de 'actuation'.
TAREA: Enviar los datos (calculados en 'actuation') a las gr?ficas
       para su visualizaci?n en tiempo real.
=================================================================================
--]]
function sysCall_sensing()
    -- Grafica 1 (XYZ Efector)
    sim.setGraphStreamValue(graf, posx, Grap[1])
    sim.setGraphStreamValue(graf, posy, Grap[2])
    sim.setGraphStreamValue(graf, posz, Grap[3])
    
    -- Grafica 2 (XY Comparativa)
    sim.setGraphStreamValue(graf_x_y, posx_Efector, Grap[1])
    sim.setGraphStreamValue(graf_x_y, posy_Efector, Grap[2])
    sim.setGraphStreamValue(graf_x_y, posx_Movil, Posc[1])
    sim.setGraphStreamValue(graf_x_y, posy_Movil, Posc[2])
    
    -- (Streams 'fake' para la gr?fica 2)
    sim.setGraphStreamValue(graf_x_y_fake, posx_fake, Posc[1])
    sim.setGraphStreamValue(graf_x_y_fake, posy_fake, Posc[2])
    sim.setGraphStreamValue(graf_x_y_fake, posz_fake, Posc[3])
    
    -- Grafica 3 (Theta Base)
    sim.setGraphStreamValue(graf_th, posth_movil, math.deg(Oric[3]))
    
    -- Grafica 4 (RPY Efector Global)
    sim.setGraphStreamValue(graf2, posalphaRobot, math.deg(Grap2[1]))
    sim.setGraphStreamValue(graf2, posbetaRobot, math.deg(Grap2[2]))
    sim.setGraphStreamValue(graf2, posgammaRobot, math.deg(Grap2[3]))
    
    -- Grafica 5 (RPY Efector Local)
    sim.setGraphStreamValue(graf5, posalphaMovil, math.deg(Grap3[1]))
    sim.setGraphStreamValue(graf5, posbetaMovil, math.deg(Grap3[2]))
    sim.setGraphStreamValue(graf5, posgammaMovil, math.deg(Grap3[3]))
end

--[[
=================================================================================
FUNCI?N: Calcular_tx_ty_con_cinematica_directa()
PROP?SITO: Calcula la extensi?n (x_arm, y_arm) del brazo en el marco GLOBAL.
REFERENCIA: Tesis Eq. 4.14 (proyectada al marco global).
            Es el 'calcularX1m_X2m()' del Arduino.
=================================================================================
--]]
function Calcular_tx_ty_con_cinematica_directa()
    -- Par?metros f?sicos (longitud de eslabones)
    L1 = 1.5951e-01
    L2 = 0.12668
    L3 = 0.12669
    L5 = 0.18485
    
    -- (Tesis Eq. 4.14, t?rmino 'r' o proyecci?n 2D del brazo)
    local term = L3 * math.sin(th2 + th3) + L2 * math.sin(th2) + L5 * math.sin(th2 + th3 + th4)
    
    -- Proyecci?n al marco global usando la orientaci?n de la base (th)
    -- (Tesis Eq. 4.62, parte x_arm/y_arm)
    xm1 = math.cos(th) * (-math.cos(th1) * term) - math.sin(th) * (-math.sin(th1) * term)
    ym1 = math.sin(th) * (-math.cos(th1) * term) + math.cos(th) * (-math.sin(th1) * term)
end

--[[
=================================================================================
FUNCI?N: controlCinematicoCarrito()
PROP?SITO: Implementa el control de la BASE M?VIL (CINEM?TICA COMBINADA).
REFERENCIA: Tesis Eq. 4.62 (Control) y Eq. 4.9 (Cinem?tica Inversa).
=================================================================================
--]]
function controlCinematicoCarrito()
    -- Par?metros f?sicos
    b = 0.14
    L = 0.097
    k = 0.15
    rc = 0.038 -- Radio de rueda (diferente al 'r' del Arduino, verificar consistencia)

    -- === LEY DE CONTROL COMBINADO (Tesis Eq. 4.62) ===
    -- ?Esta es la clave del sistema!
    -- El error no es (x - xd), sino (x - (xd - xm1)).
    -- La base se mueve para compensar la extensi?n del brazo (xm1, ym1).
    ux = -k * (x - (xd - xm1))
    uy = -k * (y - (yd - ym1))
    u_th = -k * (th - thd)
    
    -- Transformaci?n de Global (ux, uy) a Local (Vf, VL) (Tesis Eq. 4.8)
    Vf = ux * math.cos(th) + uy * math.sin(th)
    VL = -ux * math.sin(th) + uy * math.cos(th)
    w = u_th

    -- Cinem?tica Inversa de la Base (Tesis Eq. 4.9)
    -- Calcula las velocidades tangenciales (m/s) de cada rueda
    V1 = Vf - VL - (L + b) * w
    V2 = Vf + VL + (L + b) * w
    V3 = Vf + VL - (L + b) * w
    V4 = Vf - VL + (L + b) * w

    -- Convierte de m/s a rad/s (velocidad angular de la rueda)
    w1 = V1 / rc
    w2 = V2 / rc
    w3 = V3 / rc
    w4 = V4 / rc
end

--[[
=================================================================================
FUNCI?N: controlCinematicoManipulador()
PROP?SITO: Implementa la CINEM?TICA INVERSA del brazo de 5 GDL.
REFERENCIA: Tesis Cap 4.4, Eq. 4.55 y 4.61.
=================================================================================
--]]
function controlCinematicoManipulador()
    -- Par?metros f?sicos (longitud de eslabones)
    L1 = 1.5951e-01
    L2 = 0.12668
    L3 = 0.12669
    L5 = 0.18485

    -- === FASE 1: Calcular P4 (centro de la mu?eca) ===
    -- (Tesis Eq. 4.61)
    -- Calcula la posici?n objetivo de la mu?eca (x4, y4, z4) en el marco LOCAL de la base.
    x4 = (math.cos(th) * (xd - x) + math.sin(th) * (yd - y)) - L5 * math.sin(betaD)
    y4 = (-math.sin(th) * (xd - x) + math.cos(th) * (yd - y)) + L5 * math.cos(betaD) * math.sin(alphaD)
    z4 = zd - L5 * math.cos(alphaD) * math.cos(betaD)
    
    -- === FASE 2: Resolver th1, th2, th3 (Posicionamiento) ===
    -- (Tesis Eq. 4.55 - parte geom?trica)
    
    -- (Tesis Fig. 4.10)
    r = math.sqrt((x4)^2 + (y4)^2) -- Proyecci?n 2D
    zp = z4 - L1                 -- Altura relativa
    r2 = math.sqrt((r^2) + (zp^2)) -- Distancia 3D (eje 2 a 4)

    -- Ley de Cosenos
    M1 = ((L2^2) + (L3^2) - (r2^2)) / (2 * L2 * L3)
    M2 = ((L2^2) + (r2^2) - (L3^2)) / (2 * L2 * r2)

    -- 'constrain' (limitar a [-1, 1] para evitar errores de dominio en acos)
    M1 = math.max(-1, math.min(1, M1))
    M2 = math.max(-1, math.min(1, M2))

    -- (Tesis Fig. 4.10 - alpha, betha, gamma)
    alpha = math.atan2(math.sqrt(1 - (M1^2)), M1) -- acos(M1)
    betha = math.atan2(zp, r)
    gamma = math.acos(M2)

    -- Calcular th1
    th1 = math.atan2(y4, x4)
    if th1 > th1_max then
        th1 = th1_max
    elseif th1 < th1_min then
        th1 = th1_min
    end
        
    -- Soluci?n "codo arriba" / "codo abajo" (Tesis Eq. 4.55)
    if (z4 > 0.4) then
        th2 = -(-(gamma + betha) + (math.pi / 2))
        th3 = -(math.pi - alpha)
    else
        th2 = -(-(-gamma + betha) + (math.pi / 2))
        th3 = -(math.pi + alpha)
    end
    
    if th2 > th2_max then
        th2 = th2_max
    elseif th2 < th2_min then
        th2 = th2_min
    end

    -- === FASE 3: Resolver th4, th5 (Orientaci?n) ===
    -- (Tesis Eq. 4.55 - parte de orientaci?n)
    
    -- C?lculo de th4
    M3 = math.cos(th2 + th3) * math.cos(alphaD) * math.cos(betaD) 
       - math.sin(th2 + th3) * math.sin(betaD) * math.cos(th1) 
       + math.sin(th2 + th3) * math.cos(betaD) * math.sin(alphaD) * math.sin(th1)

    M3 = math.max(-1, math.min(1, M3)) -- 'constrain'
    th4 = math.acos(M3)
    
    -- C?lculo de th5
    M4 = math.atan2(-(
        math.cos(betaD) * math.cos(gammaD) * math.sin(th1) 
        - math.cos(th1) * (math.cos(alphaD) * math.sin(gammaD) 
        + math.cos(gammaD) * math.sin(alphaD) * math.sin(betaD))),
        
        -(- math.cos(th1) * (math.cos(alphaD) * math.cos(gammaD) 
        - math.sin(alphaD) * math.sin(betaD) * math.sin(gammaD))
        - math.cos(betaD) * math.sin(gammaD) * math.sin(th1))
    )
    th5 = M4

    -- Aplicar l?mites finales a th4 y th5
    if th4 > th4_max then
        th4 = th4_max
    elseif th4 < th4_min then
        th4 = th4_min
    end
    
    if th5 > th5_max then
        th5 = th5_max
    elseif th5 < th5_min then
        th5 = th5_min
    end
end

--[[
=================================================================================
FUNCI?N: actualizarAngulosLocales()
PROP?SITO: Transforma la orientaci?n deseada de Global a Local (relativa a la base).
REFERENCIA: Tesis Eq. 4.67 - 4.69.
            Es id?ntica a la funci?n del Arduino.
=================================================================================
--]]
function actualizarAngulosLocales()
    -- (Tesis Eq. 4.68)
    local r31 = math.sin(alphaD) * math.sin(gammaD) - math.cos(alphaD) * math.cos(gammaD) * math.sin(betaD)
    local cosB = math.sqrt(1 - r31^2)

    -- (Tesis Eq. 4.67) - betaD local
    betaD = -math.asin(r31)

    -- (Tesis Eq. 4.68) - gammaD local
    gammaD = math.atan2(
        -(
            math.sin(th) * (math.cos(alphaD) * math.cos(gammaD) - math.sin(alphaD) * math.sin(betaD) * math.sin(gammaD)) 
            - math.cos(betaD) * math.cos(th) * math.sin(gammaD)
        ) / cosB,
        (
            math.sin(th) * (math.cos(alphaD) * math.sin(gammaD) + math.cos(gammaD) * math.sin(alphaD) * math.sin(betaD)) 
            + math.cos(betaD) * math.cos(gammaD) * math.cos(th)
        ) / cosB
    )

    -- (Tesis Eq. 4.69) - alphaD local
    alphaD = math.atan2(
        -(
            -math.sin(betaD) * math.sin(th) 
            - math.cos(betaD) * math.sin(alphaD) * math.cos(th)
        ) / cosB,
        (
            math.cos(alphaD) * math.cos(betaD)
        ) / cosB
    )
    -- (Debug print, comentado)
    -- print("alphaD: ",alphaD,"betaD: ",betaD,"gammaD: ",gammaD,"thm: ",th);
end
