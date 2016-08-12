//////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Codigo para el control del proyector de cine de 16mm modificado segun proyecto de Xavi Hurtado V2.

El control se compone de dos partes: para el motor de arrastre se utiliza una placa Trinamic TMCM-1630 con comunicacion
RS-232, para los motores de los obturadores y la comunicacion un Arduino Zero (procesador M0). Para la comunicacion
utilizamos una mochila Ethernet2 con basada el W5500 WIZnet y para la potencia en los motores de los obturadores una
mochila Pololu dual MC33926.

La distribucion I/O es la siguiente:
0,1 - serial tll utilizado para comunicacion con la placa de control Trinamic
2 - habilitacion de PWM de salidad de la placa Pololu
7 - interrupcion para el control de velocidad del motor del obturador trasero
4 - control de la direccion de giro del motor del obturador trasero
5 - interrupcion para el control de velocidad del motor del oburador frontal
6 - control de la direccion de giro del motor del obturador frontal
3 - interrupcion para el control del tiempo diferencial del oburador trasero
8 - interrupcion para el control del tiempo diferencial del oburador frontal
9 - interrupcion para el control del tiempo diferencial del arrastre de cinta
10 - utilizado por Ethernet2 CS
11 - PWM a 22kHz, 11 bits para alimentacion del motor del obturador frontal
12 - PWM a 22kHz, 11 bits para alimentacion del LED de iluminacion
13 - PWM a 22kHz, 11 bits para alimentacion del motor del obturador trasero
A0 - entrada del sensor de corriente de la placa Pololu para el motor del obturador frontal
A1 - entrada del sensor de corriente de la placa Pololu para el motor del obturador trasero

La comunicacion se hace a traves de ethernet con protocolo UDP, la IP local es 172.26.100.172 se recibe por el
puerto 8000 y se responde en el puerto 6000.
Los datos a recibir son (UDP message: string-identificador, float o int-valor a recibir):

/Luz                  - potencia enviada a la lampara LED - int 0..255
/Vel_main   - velocidad que se pretende del motor de traccion de la cinta - int o float -43~43 en fps (frames por segundo)
/Vel_front  - velocidad que se pretende del obturador frontal en OPS (obturaciones por segundo) - 0~150
/Vel_tras   - velocidad que se pretende del obturador frontal en OPS - 0~150

datos devueltos cada 100mS:

/vel_obt_tras_act     - velocidad actual del obturador trasero en OPS
/vel_obt_front_act    - velocidad actual del obturador frontal en OPS
/ph_diff_main_front   - diferencia de fase del obturador frontal con respecto a su maestro - 0~360º
/ph_diff_main_tras    - diferencia de fase del obturador trasero con respecto a su maestro - 0~360º
/frame_count	      - cuenta actual de frames desde el momento del inicio

Descripcion del sistema de control de los motores que mueven los obturadores:
El bloque de control basico es un sistema de lazo cerrado alimentando un PID con la velocidad deseada y la velocidad actual
medida en la interrupcion correspondiente (estas interrupciones tienen una tasa de 11 pulsos por revolucion y son medidas en la polea
solidaria con el motor en cuestion), la salida de PID es aplicada al PWM que alimenta el puente H de cada motor.
El siguiente nivel de control es un segundo lazo de realimentacion que tiene como entrada el resultado de la diferencia de tiempo entre
el motor que sera esclavizado y su maestro, la salida se suma a la entrada de PID, en este caso los parametros Ki, Kp y Kd
de PID estaran ajustados en una relacion aproximada 22:1 integral con respecto a proporcional, conformando un filtro de
paso bajo. Si pensamos en el motor como un oscilador controlado por voltaje (PWM), la medicion de diferencia de tiempo es
efectivamente un comparador de fase y la PID ajustada como filtro de paso bajo, el conjunto se puede ver como un PLL.
Uno de los tiempos que entran en el comparador de fase se multiplicara por un factor (/Mul_front y /Mul_tras respectivamente)
para obtener una velocidad multiplicada como resultado pudiendo controlar de esta manera velocidades relativas al master diferentes
a la unidad.
Un tercer lazo de realimentacion existe entre, la diferencia del tiempo medido directamente en cada obturador y el medido en
quien sea su master, esta diferencia es expresada en grados (0~360) y sumada al tiempo que se le resta del maestro para obtener
el error de la comparacion de entrada a PID, manejando una peticion de error en ese valor (entradas /Ph_f_target y /Ph_t_target)
conseguiremos un 'angulo' sumado al de referencia que proporciona el maestro. Este angulo se supervisa en el main loop y se corrige
cuando se obtiene el valor de tiempo en cuestion.
Para seleccionar los estados se han colocado switches logicos con los que se permite seleccionar el origen de datos para el esclavo
(/Master_tras front y /Master_front_tras) y si el motor en cuestion es libre o esclavo (/Front_free y /Tras_free), si el motor es
libre el control sera su entrada de peticion de velocidad (/Vel_front y /Vel_tras)


Escrito por Alejandro Bizzotto en 2015-16.


variables asignadas

nombre || uso


///////////////////////////////////////////////****************//////////////////////////////////////////////////

#include <SPI.h>               // needed for Arduino versions later than 0018
#include <Ethernet2.h>         // como estamos utilizando el W5500 necesitamos ethernet2 
#include <EthernetUdp2.h>      // UDP library from: bjoern@cs.stanford.edu 12/30/2008  
#include <PID_v1.h>            // PID library: http://playground.arduino.cc/Code/PIDLibrary  -doubles cambiados a floats, sample time en uS
#include <OSCMessage.h>        // para trabajar mas facilmente el intercambio de datos por udp los formatearemos en OSC, https://github.com/CNMAT/OSC

#define UDP_TX_PACKET_MAX_SIZE 28 // amplio el buffer a 28 bytes para evitar la necesidad de fragmentacion

// MAC address and IP address
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(172, 26, 100, 177);
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
EthernetUDP Udp;  // An EthernetUDP instance

volatile unsigned long new_time_mot_tras, old_time_mot_tras, new_time_mot_front, old_time_mot_front, old_time_main, new_time_main, last_sample_time_front,
         last_sample_time_tras, tick_front_act, tick_front_ant, tick_tras_act, tick_tras_ant, abs_time_obt_front, abs_time_obt_tras, abs_time_mot_tras,
         abs_time_mot_front, new_time_obt_front, old_time_obt_front, new_time_obt_tras, old_time_obt_tras;
volatile boolean time_report_mot_tras = 1, time_report_mot_front = 1, time_report_main = 0, time_report_obt_front = 0, time_report_obt_tras = 0;
volatile long act_time_mot_front = 1, act_time_mot_tras = 1, act_time_main = 1, act_time_obt_front = 1, act_time_obt_tras = 1;
volatile boolean tras_free = 1, master_tras_front, master_front_tras, front_free = 1, tras_call_int = 0, front_call_int = 0;
volatile float error_front_main = 1.0, error_tras_main = 1.0, error_front_tras = 1.0, error_tras_front = 1.0, ph_diff_main_front, ph_diff_main_tras,
               time_diff_main_front, time_diff_main_tras, time_diff_tras_front, time_diff_front_tras, ph_diff_tras_front, ph_diff_front_tras, reg;

const float prop_mot = 180.441E3, prop_mot_2 = 90220.5;      // factor de conversion tiempo_entre_pulsos vs ops (obturaciones por segundo)

int old_time_deb = 0;                         // para el control del tiempo al enviar datos
int time_front_fwd, time_front_rwd, fwd_rwd_front, time_tras_fwd, time_tras_rwd, fwd_rwd_tras, curr_tras, curr_front, frame_count = 0, time_send,
    time_front_no_int, time_tras_no_int, count_front = 0, time_tras_park, duty_t_high, time_front_park, duty_f_high;
float fino = 1.0, fino2 = 1.0, mul_front = 1.0, mul_tras = 1.0, vel_main, ph_f_target = 140.0, ph_t_target = 170.0;
// los obturadores pasan por el lado superior de la pantalla en estos angulos con respecto a
// la señal que sincroniza main, siendo el tiempo inmediato al flanco de subida de la señal
// del main el momento en el que se ha depositado un cuadro de la cinta en su sitio, por lo que
// podemos decir que en estos angulos esta sincronizado en una proyeccion normal de 2 obturaciones
// por frame
boolean leido = 1, new_vel_main_target, tras_park = 0, front_park = 0, main_run_l, main_run_r, tras_pol_sel, front_pol_sel;
byte count = 0;
char command[9], response[9];

// instancias PID
//     consigna recibida,  velocidad actual, resultado de PID
float vel_obt_tras_target, vel_obt_tras_act, pwm_mot_obt_tras;
//   factor proporcional, f. integral,       f. derivado
float Kp_obt_tras = 10.0, Ki_obt_tras = 50.0, Kd_obt_tras = 4.7E-4;  // por defecto la configuracion de PID es relajada
//                    entrada            salida             consigna
PID vel_obt_tras_PID(&vel_obt_tras_act, &pwm_mot_obt_tras, &vel_obt_tras_target, Kp_obt_tras, Ki_obt_tras, Kd_obt_tras, DIRECT); // motor del obturador trasero

float vel_obt_front_target, vel_obt_front_act, pwm_mot_obt_front, buff;
float Kp_obt_front = 10.0, Ki_obt_front = 50.0, Kd_obt_front = 4.7E-4;
PID vel_obt_front_PID(&vel_obt_front_act, &pwm_mot_obt_front, &vel_obt_front_target, Kp_obt_front, Ki_obt_front, Kd_obt_front, DIRECT); // motor del obturador frontal

void setup() {

  pinMode(2, OUTPUT); // aqui conecto la habilitacion del pwm de salida
  pinMode(4, OUTPUT); // seleccion de direccion de giro del motor del obturador trasero
  pinMode(6, OUTPUT); // seleccion de direccion de giro del motor del obturador frontal
  pinMode(3, INPUT);  // sensor optico para la posicion de parking y correccion de fase en obturador trasero
  pinMode(8, INPUT);  // sensor optico para la posicion de parking y correccion de fase en obturador frontal
  pinMode(A5, OUTPUT); // debug
  pinMode(A0, INPUT);  // sensor de corriente para el motor del obturador frontal
  pinMode(A1, INPUT);  // sensor de corriente para el motor del obturador trasero

  // start the Ethernet and UDP:
  Ethernet.begin(mac, ip);
  Udp.begin(8000);
  Serial5.begin(57600);      // con la placa Trinamic nos comunicaremos por serie
  // SerialUSB.begin(57600);

  attachInterrupt(7, time_control_mot_tras, RISING);   // interrupcion en pin 3 para el lector optico de velocidad en el motor del obturador trasero
  attachInterrupt(3, time_control_obt_tras, RISING);   // interrupcion en pin 3 para el lector optico de fase en el obturador trasero
  attachInterrupt(5, time_control_mot_front, RISING);  // interrupcion en pin 5 para el lector optico de velocidad en el motor del obturador delantero
  attachInterrupt(8, time_control_obt_front, RISING);  // interrupcion en pin 8 para el lector optico de fase en el obturador frontal
  attachInterrupt(9, time_control_main, RISING);       // interrupcion en pin 9 para el lector optico de fase en motor de arrastre
  
  analogWrite (13, 0);   // preparo las salidas de pwm en pines 13 y 11 para luego escribir directamente en sus registros
  analogWrite(12, 0);    // 12 utiliza TC3 WO1, el prescaler lo cambiare luego de llamar a analogWrite del pin 4 a DIV8 para tener ~170Hz
  analogWrite(11, 0);    // 13 y 11 utilizan TCC2, 11 bits de resolucion y ~23kHz
  REG_TCC2_PER = 0x800;

  analogWrite(4, 0);     // 4 utiliza TC3 WO0
  REG_TC3_CTRLA &= ~TCC_CTRLA_ENABLE;
  while (REG_TC3_STATUS == TC_STATUS_SYNCBUSY);
  REG_TC3_CTRLA |= TCC_CTRLA_PRESCALER_DIV64;    // frecuencia del pwm de salida ~ 3kHz
  REG_TC3_CTRLA |= TCC_CTRLA_ENABLE;
  while (REG_TC3_STATUS == TC_STATUS_SYNCBUSY);

  analogWrite(6, 0);     // 6 utililiza TCC0 WO6
  REG_TCC0_CTRLA &= ~TC_CTRLA_ENABLE;
  while (REG_TCC0_SYNCBUSY == TCC_SYNCBUSY_STATUS);
  REG_TCC0_CTRLA |= TC_CTRLA_PRESCALER_DIV256;    // frecuencia del pwm de salida ~ 90Hz
  REG_TCC0_CTRLA |= TC_CTRLA_ENABLE;
  while (REG_TCC0_SYNCBUSY == TCC_SYNCBUSY_STATUS);
  REG_TCC0_PER = 0x800;


  vel_obt_tras_PID.SetOutputLimits(0, 2048);  // el PWM de salida es de 11 bits
  vel_obt_tras_act = 1;                      // cualquier cosa que no sea 0 aqui
  vel_obt_tras_PID.SetMode(AUTOMATIC);      // si no esta automatic esta manual
  vel_obt_tras_PID.SetSampleTime(1);       // sampletime minimo, luego se asignara dinamicamente

  vel_obt_front_PID.SetOutputLimits(0, 2048);
  vel_obt_front_act = 1;
  vel_obt_front_PID.SetMode(AUTOMATIC);
  vel_obt_front_PID.SetSampleTime(1);

}

void loop() {

  int period_105_front = int (prop_mot / vel_obt_front_target * float(1.05));  // 105% periodo de motor frontal
  if (!vel_obt_front_target) period_105_front = 0.2;                           // si vel_obt_xxx_target == 0 el tiempo anterior para la comparacion es infinito y nunca entra
  if (((micros() - time_front_no_int) > period_105_front) && (!front_park)) {  // ha pasado el 105% del periodo del motor frontal
    time_front_no_int = micros();
    if (tick_front_act == tick_front_ant) {                             // y la interrupcion no se ha producido
      act_time_mot_front += int(prop_mot_2 / vel_obt_front_target);     // entonces digo que el periodo es mas alto de lo que debiera, incrementandolo en cada iteracion
      vel_obt_front_pid_comp();                                         // y llamo a PID
      front_call_int = 1;                                               // levanto la bandera para que no se llame en la proxima interrupcion
    }
    else {
      tick_front_ant = tick_front_act;    // si se ha producido la interrupcion, igualo los valores para el siguiente intervalo de periodo
      front_call_int = 0;                 // bajo la bandera para poder llamar a PID en la interrupcion
    }
  }

  /*
     las inercias del sistema hacen que la medida de tiempo en la interrupcion pueda ser erronea en velocidades bajas por falta de torque,
     para saberlo voy a poner un contador incrementado en la interrupcion (tick_front_act y tick_tras_act), que en un tiempo 5% superior
     al periodo que habria de tener (prop_mot/vel_obt_xxx_target * float(1.05)) voy a comparar con su valor anterior, si son iguales es que no se ha
     producido la interrupcion en ese tiempo, si ademas la bandera de la ultima llamada de vel_obt_xxx_comp (xxx_call_int) no se ha levantado en
     la interrupcion es que puedo llamarla aqui
     en ese caso llamare a vel_obt_xxx_comp con un valor de periodo (act_time_mot_xxx) mas alto del teorico para que PID entregue mas torque y
     corrija en la siguiente segunda interrupcion
   */
  int period_105_tras = int (prop_mot / vel_obt_tras_target * float(1.05));
  if (!vel_obt_tras_target) period_105_tras = 0.2;
  if (((micros() - time_tras_no_int) > period_105_tras) && (!tras_park)) {
    time_tras_no_int = micros();
    if (tick_tras_act == tick_tras_ant) {
      act_time_mot_tras += int(prop_mot_2 / vel_obt_tras_target);
      vel_obt_tras_pid_comp();
      tras_call_int = 1;
    }
    else {
      tick_tras_ant = tick_tras_act;
      tras_call_int = 0;
    }
  }

  OSCMessage msg_in;
  int size_packet;

  //receive a message
  if ( (size_packet = Udp.parsePacket()) > 0)
  {
    uint8_t packetBuffer[UDP_TX_PACKET_MAX_SIZE];      // esta funcion tarda ~500uS, la funcion del ejemplo: while(size--) msg.fill(Udp.read());
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);    // tarda 6mS!!!
    msg_in.fill(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    // si la porcion string coincide, obtengo el integer o float y lo asigno a cada una de las variables de destino
    if (msg_in.match("/Luz")) {
      int val =  msg_in.getInt(0);
      if (val > 254) val = 254;
      if (val < 0) val = 0;
      REG_TC3_COUNT8_CC1 = val;          // pwm en pin 12, aqui el float no tiene sentido
      OSCMessage res ("/Luz");           // preparo respuesta con el mismo mensaje que se ha recibido
      res.add((int) val);                // y el valor numerico
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);                     // envio el mensaje de respuesta
      Udp.endPacket();
      res.empty();
    }

    if ((msg_in.match("/Vel_main")) && (!new_vel_main_target))        // identifico velocidad_del_motor principal
    {
      if (msg_in.isInt(0)) vel_main = msg_in.getInt(0); if (msg_in.isFloat(0)) vel_main = msg_in.getFloat(0); // si es Int me lo quedo, si es Float tambien
      const float rpm_fps = 69.45;            // cosas de las poleas y los segundos
      int val_num = abs(int(rpm_fps * vel_main));
      command[0] = 0x01;                                                          // identificacion de origen
      if (vel_main > 0 ) {
        command[1] = 0x01;  //numero de instruccion 1=ROR (rotate right), si es positivo voy a la derecha
        main_run_r = 1;
      }
      if (vel_main < 0 ) {
        command[1] = 0x02;  //numero de instruccion 2=ROL (rotate left), si es negativo voy a la izquierda
        main_run_l = 1;
      }
      if (vel_main == 0) {
        command[1] = 0x03;  //numero de instruccion 3=MST (motor stop)
        main_run_r = 0; main_run_l = 0;
      }
      command[2] = 0x00; //tipo
      command[3] = 0x00; //motor nº 0 porque es el unico

      command[4] = 0x00; // parametro de velocidad
      command[5] = 0x00; // como el maximo es 3000 los dos primeros seran cero
      command[6] = 0x00; // como la proxima escritura sera atomica, inicio a 0
      command[7] = 0x00;

      for (int n = 12; n >= 8; n--) {
        boolean foo = bitRead(val_num, n);     // leo cada uno de los bits del dato de velocidad
        if (foo) bitSet(command[6], (n - 8)); // y los escribo en el array de transmision, MSB first
      }

      for (int n = 8; n >= 0; n--) {
        boolean foo = bitRead(val_num, n);
        if (foo) bitSet(command[7], n);
      }

      int checksum = command[0];
      for (int i = 1; i < 8; i++) checksum += command[i]; // el ultimo byte del array es la suma
      command[8] = checksum;

      new_vel_main_target = 1;                            // flag para comenzar la transmision hacia la placa Trinamic
      count = 0;                                          // pointer para el array, se empieza en 0
      if (!front_free) {                                  // ante un cambio de velocidad en modo esclavo, asigno la misma velocidad para la base del calculo
        vel_obt_front_target = abs(vel_main * float(2.0));// de esta forma va mas rapido hasta ahi
      }
      if (!tras_free) {
        vel_obt_tras_target = abs(vel_main * float(2.0));
      }
    }

    if (msg_in.match("/Vel_front")) {
      if (msg_in.isInt(0)) vel_obt_front_target = msg_in.getInt(0);
      if (msg_in.isFloat(0)) vel_obt_front_target = msg_in.getFloat(0);
      if (!tras_free && master_tras_front) vel_obt_tras_target = vel_obt_front_target; // si el maestro del trasero es el frontal, igualo sus velocidades de base para ir mas rapido a la fase
        OSCMessage res ("/Vel_front");
        res.add((float) vel_obt_front_target);
        Udp.beginPacket(Udp.remoteIP(), 6000);
        res.send(Udp);
        Udp.endPacket();
        res.empty();
    }

    if (msg_in.match("/Vel_tras")) {
      if (msg_in.isInt(0)) vel_obt_tras_target = msg_in.getInt(0);
      if (msg_in.isFloat(0)) vel_obt_tras_target = msg_in.getFloat(0);
      if (!front_free && master_front_tras) vel_obt_front_target = vel_obt_tras_target;
        OSCMessage res ("/Vel_tras");
        res.add((float) vel_obt_tras_target);
        Udp.beginPacket(Udp.remoteIP(), 6000);
        res.send(Udp);
        Udp.endPacket();
        res.empty();
    }

    if (msg_in.match("/Master_front_tras")) {                         // los motores pueden ser esclavos del main o del motor del otro obturador
      if (msg_in.isInt(0)) master_front_tras = msg_in.getInt(0);
      if (msg_in.isBoolean(0)) master_front_tras = msg_in.getBoolean(0);
      OSCMessage res ("/Master_front_tras");
      res.add((boolean) master_front_tras);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Master_tras_front")) {
      if (msg_in.isInt(0)) master_tras_front = msg_in.getInt(0);
      if (msg_in.isBoolean(0)) master_tras_front = msg_in.getBoolean(0);
      OSCMessage res ("/Master_tras_front");
      res.add((boolean) master_tras_front);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Front_free")) {                                // cuando los obturadores son 'libres' no se enganchan en fase con nadie
      int foo = msg_in.getInt(0);                                     // y se manejaran con su control de velocidad
      if (front_free && (!foo)) {                                     // si antes era libre y se pide que sea esclavo
        if (master_front_tras) vel_obt_front_target = vel_obt_tras_act;
        else vel_obt_front_target = abs(vel_main * float(2.0));     // igualo la velocidad con su master, para ir mas rapido hasta ahi
        error_front_main = 0;
        error_front_tras = 0;
        //   Kp_obt_front = 40.0; Ki_obt_front = 900.0; Kd_obt_front = 4.7E-4;  // y ajusto PID en parametros reactivos
        // vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front);
      }
      if ((!front_free) && foo) {                                     // si antes era esclavo y se pide que sea libre
        // Kp_obt_front = 10.0; Ki_obt_front = 50.0; Kd_obt_front = 4.7E-4;
        // vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front); // ajusto PID en parametros relajados (mas estable)
      }
      front_free = foo;
      OSCMessage res ("/Front_free");
      res.add((boolean) front_free);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Tras_free")) {
      int foo = msg_in.getInt(0);
      if (tras_free && (!foo)) {
        if (master_tras_front) vel_obt_tras_target = vel_obt_front_act;
        else vel_obt_tras_target = abs(vel_main * float(2.0));
        error_tras_main = 0;
        error_tras_front = 0;
        // Kp_obt_tras = 40.0; Ki_obt_tras = 900.0; Kd_obt_tras = 4.7E-4;
        // vel_obt_tras_PID.SetTunings(Kp_obt_tras, Ki_obt_tras, Kd_obt_tras);
      }
      if ((!tras_free) && foo) {
        // Kp_obt_tras = 10.0; Ki_obt_tras = 50.0; Kd_obt_tras = 4.7E-4;
        // vel_obt_tras_PID.SetTunings(Kp_obt_tras, Ki_obt_tras, Kd_obt_tras);
      }
      tras_free = foo;
      OSCMessage res ("/Tras_free");
      res.add((boolean) tras_free);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Front_park")) {
      front_park = msg_in.getInt(0);
      OSCMessage res ("/Front_park");
      res.add((boolean) front_park);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
      if (!front_park) {
        TCC0->CC[2].reg = 0;
      }
    }

    if (msg_in.match("/Tras_park")) {
      tras_park = msg_in.getInt(0);
      OSCMessage res ("/Tras_park");
      res.add((boolean) tras_park);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
      if (!tras_park) {
        REG_TC3_COUNT8_CC0 = 0;             // si no esta en parking vuelvo este pin a 0, que es el control de la polaridad del puente H
      }
    }

    if (msg_in.match("/Mul_front")) {
      if (msg_in.isInt(0)) mul_front = msg_in.getInt(0);
      if (msg_in.isFloat(0)) mul_front = msg_in.getFloat(0);
      if (mul_front == 0 ) mul_front = 1.0E-10;
      OSCMessage res ("/Mul_front");
      res.add((float) mul_front);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Mul_tras")) {
      if (msg_in.isInt(0)) mul_tras = msg_in.getInt(0);       // factor para dividir la frecuencia del objeto comparado
      if (msg_in.isFloat(0)) mul_tras = msg_in.getFloat(0);
      if (mul_tras == 0 ) mul_tras = 1.0E-10;                 // evito el 0, posible nan
      OSCMessage res ("/Mul_tras");
      res.add((float) mul_tras);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Ph_f_target")) {                        // objetivo de diferencia de fase
      if (msg_in.isInt(0)) ph_f_target = msg_in.getInt(0);
      if (msg_in.isFloat(0)) ph_f_target = msg_in.getFloat(0);
      //ph_f_target -= 29;
      OSCMessage res ("/Ph_f_target");
      res.add((float) ph_f_target);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Ph_t_target")) {
      if (msg_in.isInt(0)) ph_t_target = msg_in.getInt(0);
      if (msg_in.isFloat(0)) ph_t_target = msg_in.getFloat(0);
      //ph_t_target += 20;
      OSCMessage res ("/Ph_t_target");
      res.add((float) ph_t_target);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      res.send(Udp);
      Udp.endPacket();
      res.empty();
    }

    if (msg_in.match("/Fino"))
    {
      if (msg_in.isInt(0)) fino = msg_in.getInt(0);
      if (msg_in.isFloat(0)) fino = msg_in.getFloat(0);
    }
    if (msg_in.match("/Fino2"))
    {
      if (msg_in.isInt(0)) fino2 = msg_in.getInt(0);
      if (msg_in.isFloat(0)) fino2 = msg_in.getFloat(0);
    }

    if (msg_in.match("/Kp_obt_tras")) {
      Kp_obt_tras = msg_in.getFloat(0);
      vel_obt_tras_PID.SetTunings(Kp_obt_tras, Ki_obt_tras, Kd_obt_tras);
    }
    if (msg_in.match("/Ki_obt_tras")) {
      Ki_obt_tras = msg_in.getFloat(0);
      vel_obt_tras_PID.SetTunings(Kp_obt_tras, Ki_obt_tras, Kd_obt_tras);
    }
    if (msg_in.match("/Kd_obt_tras")) {
      Kd_obt_tras = msg_in.getFloat(0);
      vel_obt_tras_PID.SetTunings(Kp_obt_tras, Ki_obt_tras, Kd_obt_tras);
    }

    if (msg_in.match("/Kp_obt_front")) {
      Kp_obt_front = msg_in.getFloat(0);
      vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front);
    }
    if (msg_in.match("/Ki_obt_front")) {
      Ki_obt_front = msg_in.getFloat(0);
      vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front);
    }
    if (msg_in.match("/Kd_obt_front")) {
      Kd_obt_front = msg_in.getFloat(0);
      vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front);
    }

  }
  /*
   if (((millis() - trans_time) > 5) && leido)
   {
     for (int i = 0; i < 9; i++) Serial5.write(command[i]); // transmito el array hacia la placa Trinamic
     trans_time = millis();
     leido = 0;
   }

   if ((Serial5.available()) > 0)
   {
     char response[9];
     for (int i=0; i < 9; i++) response[i] = Serial5.read();
     leido = 1;
   }
  */

  if (new_vel_main_target) {
    Serial5.write(command[count]);      // la transmision hacia la placa trinamic la hare atomica, si la hago en un solo for se cuelga
    count++;                            // por motivo aun desconocido
    if (count > 8) new_vel_main_target = 0;
  }

  if (pwm_mot_obt_tras || pwm_mot_obt_front || tras_park || front_park) digitalWrite(2, HIGH);  // si hay algo que sacar PWM Output enable
  else digitalWrite(2, LOW);  // si no hay nada importante PWM Output disable, extra de paranioa

  if (tras_park) {
    tras_free = 1;
    vel_obt_tras_PID.SetMode(MANUAL);
    detachInterrupt (7);
    detachInterrupt (3);    
    if (((millis() - time_tras_park > 40) && tras_pol_sel)) {
      time_tras_park = millis();
      tras_pol_sel = 0;
      digitalWrite (4, tras_pol_sel);
    }
    if (((millis() - time_tras_park > duty_t_high) && !tras_pol_sel)) {
      time_tras_park = millis();
      tras_pol_sel = 1;
      digitalWrite (4, tras_pol_sel);
    }

    if (digitalRead(3)) {                      // si estoy en la zona de parking aplico freno de motor
      if (main_run_r) duty_t_high = 35;        // la vibracion que produce la uña de arrastre de la cinta hace que las inercias del sistema se puedan vencer mas facilmente
      if (main_run_l) duty_t_high = 45;        // por lo que cuando el main esta funcionando, el freno de motor tira mas hacia el lado contrario
      if (!main_run_r && !main_run_l) duty_t_high = 40;
      TCC2->CC[1].reg = 350;            
    } else { 
      duty_t_high  = 15;
      TCC2->CC[1].reg = 450;
    }
    } else {
      attachInterrupt(7, time_control_mot_tras, RISING);   // interrupcion en pin 3 para el lector optico de velocidad en el motor del obturador trasero
      attachInterrupt(3, time_control_obt_tras, RISING);   // interrupcion en pin 3 para el lector optico de fase en el obturador trasero
      vel_obt_tras_PID.SetMode(AUTOMATIC);
      digitalWrite(4, 0);
   }

  if (front_park) {
    front_free = 1;
    vel_obt_front_PID.SetMode(MANUAL);
    detachInterrupt (5);
    detachInterrupt (8);    
    if (((millis() - time_front_park > 40) && front_pol_sel)) {
      time_front_park = millis();
      front_pol_sel = 0;
      digitalWrite (6, front_pol_sel);
    }
    if (((millis() - time_front_park > duty_f_high) && !front_pol_sel)) {
      time_front_park = millis();
      front_pol_sel = 1;
      digitalWrite (6, front_pol_sel);
    }    

    if (digitalRead(8)) {
      if (main_run_r) duty_f_high = 35;
      if (main_run_l) duty_f_high = 75;
      if (!main_run_r && !main_run_l) duty_f_high = 40;
      if (abs(vel_main) > 24) TCC2->CC[0].reg = 480; 
      else if (vel_main < -24) TCC2->CC[0].reg = 540;
      else TCC2->CC[0].reg = 450;
    } else {
      duty_f_high = 10;
      TCC2->CC[0].reg = 450;
    }
  } else {
    attachInterrupt(5, time_control_mot_front, RISING);  // interrupcion en pin 5 para el lector optico de velocidad en el motor del obturador delantero
    attachInterrupt(8, time_control_obt_front, RISING);  // interrupcion en pin 8 para el lector optico de fase en el obturador frontal
    vel_obt_front_PID.SetMode(AUTOMATIC);
    digitalWrite(6, 0);
  }

  if (!tras_free) {
    if (master_tras_front) fwd_rwd_tras = ph_t_target - ph_diff_front_tras;
    else fwd_rwd_tras = ph_t_target - ph_diff_main_tras;  // agregare este offset al calculo de error para conseguir un desplazamiento de fase controlable
  }
  if (!front_free) {
    if (master_front_tras) fwd_rwd_front = ph_f_target - ph_diff_tras_front;
    else fwd_rwd_front = ph_f_target - ph_diff_main_front;
  }

  // mensajes a enviar
  if ((millis() - time_send) > 100) {
    time_send = millis();
    OSCMessage msg_1("/vel_obt_front_act");
    msg_1.add((float)vel_obt_front_act);
    Udp.beginPacket(Udp.remoteIP(), 6000);
    msg_1.send(Udp);
    Udp.endPacket();
    msg_1.empty();

    OSCMessage msg_2("/vel_obt_tras_act");
    msg_2.add((float)vel_obt_tras_act);
    Udp.beginPacket(Udp.remoteIP(), 6000);
    msg_2.send(Udp);
    Udp.endPacket();
    msg_2.empty();

    OSCMessage msg_3("/frame_count");
    msg_3.add((float)frame_count);
    Udp.beginPacket(Udp.remoteIP(), 6000);
    msg_3.send(Udp);
    Udp.endPacket();
    msg_3.empty();

    if (!front_free) {
      OSCMessage msg_out("/ph_diff_front_act");
      if (master_front_tras) msg_out.add((float) ph_diff_tras_front);
      else msg_out.add((float) ph_diff_main_front);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      msg_out.send(Udp);
      Udp.endPacket();
      msg_out.empty();
    }

    if (!tras_free) {
      OSCMessage msg_out("/ph_diff_tras_act");
      if (master_tras_front) msg_out.add((float) ph_diff_front_tras);
      else msg_out.add((float) ph_diff_main_tras);
      Udp.beginPacket(Udp.remoteIP(), 6000);
      msg_out.send(Udp);
      Udp.endPacket();
      msg_out.empty();
    }
  }

  /*
    //// debug
    if ((millis() - old_time_deb) > 100) // 100 mS entre datos enviados
    {
      old_time_deb = millis();

      OSCMessage msg_out("/vel_obt_tras_act");              // preparo el encabezado del mensaje
      msg_out.add((float)vel_obt_tras_act);  // y los datos numericos a transmitir
      msg_out.add((float)vel_obt_front_act);

      msg_out.add((float) pwm_mot_obt_front);
      msg_out.add((float) pwm_mot_obt_tras);

      msg_out.add((float) vel_obt_tras_target);
      msg_out.add((float) vel_obt_front_target);

      msg_out.add((float) ph_diff_tras_front);
      msg_out.add((float) ph_diff_front_tras);

      msg_out.add((float) front_call_int);

      msg_out.add((float)ph_diff_main_front);
      msg_out.add((float)ph_diff_main_tras);

      msg_out.add((float)reg);

      Udp.beginPacket(Udp.remoteIP(), 6000);
      msg_out.send(Udp); // send the bytes to the SLIP stream
      Udp.endPacket();
      msg_out.empty(); // free space occupied by message
    }
     */
}

float ganancia_pll = -1.0E-4;    // * fino;

void vel_obt_tras_pid_comp() {

  // ops = rpm / 60 = microsegundos en un minuto * ganancia de la transmision (1.089413) / numero de pasos del sensor (12) * tiempo entre pulsos / 60
  vel_obt_tras_act = prop_mot / act_time_mot_tras;  // obtengo ops actuales
  if (!tras_free) {                                                                // si el motor no es libre
    if (master_tras_front) vel_obt_tras_target += error_tras_front * ganancia_pll; // puede ser esclavo del otro
    else vel_obt_tras_target += error_tras_main * ganancia_pll;                    // o del main
  }
  if (vel_obt_tras_target > 150) vel_obt_tras_target = 150;   // limito la velocidad a 150 ops (4500rpm)
  if (vel_obt_tras_target < 0) vel_obt_tras_target = 0;
  if (vel_obt_tras_target == 0) {                                                    // cuando la velocidad pedida es 0 PID tarda mucho tiempo para averiguarlo
    vel_obt_tras_PID.SetMode(MANUAL);                                                // asi que lo forzare preguntando por el 0
    TCC2->CC[1].reg = 0;
    vel_obt_tras_act = 0;
    act_time_mot_tras = 1500;                                                        // arranque suave desde 0 en la proxima aceleracion
  } else {
    vel_obt_tras_PID.SetMode(AUTOMATIC);
    if (vel_obt_tras_target < float(7.0)) REG_TC3_COUNT8_CC0 = 83;                   // para velocidades bajas manejo el motor como un pseudo-stepper alternando
    else REG_TC3_COUNT8_CC0 = 0;                                                     // la polaridad del puente H
    if (tras_free) {
      Kp_obt_tras = 40.0; Ki_obt_tras = 30.0; Kd_obt_tras = 1.2E-4;
      vel_obt_tras_PID.SetTunings(Kp_obt_tras, Ki_obt_tras, Kd_obt_tras);
    } else {
      Kp_obt_tras = 64.0; Ki_obt_tras = 580.0; Kd_obt_tras = 2.9E-4;
      vel_obt_tras_PID.SetTunings(Kp_obt_tras, Ki_obt_tras, Kd_obt_tras);
    }
    vel_obt_tras_PID.SetSampleTime(micros() - last_sample_time_tras);                // los intervalos entre los que se llama a PID son variables
    vel_obt_tras_PID.Compute();
    TCC2->CC[1].reg = int(pwm_mot_obt_tras);
    last_sample_time_tras = micros();
  }
}

void vel_obt_front_pid_comp() {

  vel_obt_front_act = prop_mot  / act_time_mot_front;
  if (!front_free) {
    if (master_front_tras) vel_obt_front_target += error_front_tras * ganancia_pll;
    else vel_obt_front_target += error_front_main * ganancia_pll;
  }
  if (vel_obt_front_target > 150) vel_obt_front_target = 150;
  if (vel_obt_front_target < 0) vel_obt_front_target = 0;
  if (vel_obt_front_target == 0 ) {
    vel_obt_front_PID.SetMode(MANUAL);
    TCC2->CC[0].reg = 0;
    vel_obt_front_act = 0;
    act_time_mot_front = 1500;
  } else {
    vel_obt_front_PID.SetMode(AUTOMATIC);
    if (vel_obt_front_target < float(7.0)) {
      TCC0->CC[2].reg = 500;
      Kp_obt_front = 200.0; Ki_obt_front = 50.0; Kd_obt_front = 2E-4;
      vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front);
    }
    else {
      TCC0->CC[2].reg = 0;
      if (front_free) {
        Kp_obt_front = 40.0; Ki_obt_front = 50.0; Kd_obt_front = 3.0E-4;
        vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front);
      } else {
        Kp_obt_front = 84.0; Ki_obt_front = 540.0; Kd_obt_front = 5.5E-4;
        vel_obt_front_PID.SetTunings(Kp_obt_front, Ki_obt_front, Kd_obt_front);
      }
    }
    vel_obt_front_PID.SetSampleTime(micros() - last_sample_time_front);
    vel_obt_front_PID.Compute();
    TCC2->CC[0].reg = int(pwm_mot_obt_front);
    last_sample_time_front = micros();
  }
}

// interrupcion para control del obturador trasero
void time_control_mot_tras() {
  switch (time_report_mot_tras) {
    case 1:
      old_time_mot_tras = micros();                                // pongo marca de tiempo en el primer pulso
      act_time_mot_tras = old_time_mot_tras - new_time_mot_tras;  //calculo el tiempo que ha pasado entre las marcas (las marcas cambian de orden cada vez)
      time_report_mot_tras = 0;
      break;
    case 0:
      new_time_mot_tras = micros();                                // pongo marca de tiempo en el segundo pulso
      act_time_mot_tras = new_time_mot_tras - old_time_mot_tras;  //calculo el tiempo que ha pasado entre las marcas (asi no pierdo lecturas entre los pulsos)
      time_report_mot_tras = 1;
      break;
  }
  tick_tras_act ++;
  error_tras_main = act_time_main / (float(10.9928)) - (act_time_mot_tras + fwd_rwd_tras) * mul_tras; // comparador de fase para trasero esclavo de main
  error_tras_front = act_time_mot_front - (act_time_mot_tras + fwd_rwd_tras) * mul_tras;              // comparador de fase para trasero esclavo de frontal
  // phase error   =   referencia           vco a comparar      offset (fase)  multiplicador de frecuencia
  if (!tras_call_int) {
    vel_obt_tras_pid_comp();
    tras_call_int = 1;
  } else tras_call_int = 0;
}

// interrupcion para control de velocidad del obturador frontal
void time_control_mot_front() {
  switch (time_report_mot_front) {
    case 1:
      old_time_mot_front = micros();
      act_time_mot_front = old_time_mot_front - new_time_mot_front;
      time_report_mot_front = 0;
      break;
    case 0:
      new_time_mot_front = micros();
      act_time_mot_front = new_time_mot_front - old_time_mot_front;
      time_report_mot_front = 1;
      break;
  }
  tick_front_act ++;
  error_front_main = act_time_main / (float(11.0161)) - (act_time_mot_front + fwd_rwd_front) * mul_front;
  error_front_tras = act_time_mot_tras  - (act_time_mot_front + fwd_rwd_front ) * mul_front;
  if (!front_call_int) {
    vel_obt_front_pid_comp();
    front_call_int = 1;
  } else front_call_int = 0;
}

// interrupcion para el control de la fase del main
void time_control_main() {
  switch (time_report_main) {
    case 1:
      old_time_main = micros();
      act_time_main = old_time_main - new_time_main;
      time_diff_main_front = old_time_main - abs_time_obt_front;      // diferencia de tiempo del obturador al main, calculada a partir de los tiempos absolutos
      time_diff_main_tras = old_time_main - abs_time_obt_tras;
      time_report_main = 0;
      break;
    case 0:
      new_time_main = micros();
      act_time_main = new_time_main - old_time_main;
      time_diff_main_front = new_time_main - abs_time_obt_front;
      time_diff_main_tras = new_time_main - abs_time_obt_tras;
      time_report_main = 1;
      break;
  }
  ph_diff_main_front = time_diff_main_front / act_time_main * 360;  // diferencia expresada en grados de fase
  ph_diff_main_tras = time_diff_main_tras / act_time_main * 360;
  if (main_run_r) frame_count++;
  if (main_run_l) frame_count--;
}

// interrupcion para el control de la fase del obturador frontal
void time_control_obt_front() {
  switch (time_report_obt_front) {
    case 1:
      old_time_obt_front = micros();
      act_time_obt_front = old_time_obt_front - new_time_obt_front;
      time_diff_front_tras = old_time_obt_front - abs_time_obt_tras;
      time_report_obt_front = 0;
      break;
    case 0:
      new_time_obt_front = micros();
      act_time_obt_front = new_time_obt_front - old_time_obt_front;
      time_diff_front_tras = new_time_obt_front - abs_time_obt_tras;
      time_report_obt_front = 1;
      break;
  }
  ph_diff_front_tras = time_diff_front_tras / act_time_obt_front * 360;
  abs_time_obt_front = micros();            // aqui la medida sera absoluta para poder agregar el desplazamiento de fase
}

// interrupcion para el control de la fase del obturador trasero
void time_control_obt_tras() {
  switch (time_report_obt_tras) {
    case 1:
      old_time_obt_tras = micros();
      act_time_obt_tras = old_time_obt_tras - new_time_obt_tras;
      time_diff_tras_front = old_time_obt_tras - abs_time_obt_front;
      time_report_obt_tras = 0;
      break;
    case 0:
      new_time_obt_tras = micros();
      act_time_obt_tras = new_time_obt_tras - old_time_obt_tras;
      time_diff_tras_front = new_time_obt_tras - abs_time_obt_front;
      time_report_obt_tras = 1;
      break;
  }
  ph_diff_tras_front = time_diff_tras_front / act_time_obt_tras * 360;
  abs_time_obt_tras = micros();
}
