// LINEA PRINCIPAL DEL DESARROLLO

// Includes
#include <CRC32.h> // Librería para deteccion de errores en la transmision

// Definicion de de pines
#define M0 24 // Pin para configurar el modo del lora
#define M1 31 // Pin para configurar el modo del lora

#define ENC_R_PIN 2 // Pin del encoder de la rueda DERECHA
#define ENC_L_PIN 18 // Pin del encoder de la rueda IZQUIERDA
#define PWM_R_PIN 3 // Pin del control PWM de la rueda DERECHA
#define PWM_L_PIN 46 // Pin del control PWM de la rueda IZQUIERDA

#define BRAKE_PIN 38
#define IGNITION_PIN 34
#define REVERSE_R_PIN 30
#define REVERSE_L_PIN 26

// Variables de configuracion
const double Tm = 0.1; // Periodo de muestreo (NO MODIFICAR DE NO SER COMPLETAMENTE NECESARIO y en caso hacerlo modificar el TIMER1 acorde con el nuevo valor)

const double Kp_R = 3.5; // Constante proporcional de la rueda DERECHA
const double Kd_R = 0.001; // Constante diferencial de la rueda DERECHA
const double Ki_R = 1.2; // Constante integral de la rueda DERECHA

const double Kp_L = 3.5; // Constante proporcional de la rueda IZQUIERDA
const double Kd_L = 0.001; // Constante diferencial de la rueda IZQUIERDA
const double Ki_L = 1.2; // Constante integral de la rueda IZQUIERDA

const float SCALER_R = 0.75; // Constante para calibracion de la rueda DERECHA
const float SCALER_L = 1.0; // Constante para calibracion de la rueda IZQUIERDA

const float MAX_TORQ = 2000; // Torque máximo (sin unidades) (cambiar solo este parametro no cambia el torque maximo)
const float MAX_SETPOINT = 200; // Set point maximo de RPM de las ruedas (dar maximo 1000);
const float DIR_DELTA = 90; //Diferencia entre los setpoints al girar

// Variables globales
struct RoverParameters {  // Parametros recibidos del mando
  int Throttle;           // Velocidad de las ruedas
  int Direction;          // Direccion de las ruedas
  int Pump;               // Bomba
} receivedParameters;

enum State {
  Run,
  ToggleForward,
  ToggleRight,
  ToggleLeft,
  ToggleBackward
};

State currentState = Run;
State nextState = Run;

bool ToggleForward_Blocked = true;
bool ToggleRight_Blocked = false;
bool ToggleLeft_Blocked = false;
bool ToggleBackward_Blocked = false;

uint32_t receivedChecksum; //VAriable en la que guardaremos la suma de verificacion de error

volatile uint32_t encoderCount_R = 0; // Contador de pulsos del enconder de la rueda DERECHA
volatile uint32_t encoderCount_L = 0; // Contador de pulsos del enconder de la rueda IZQUEIRDA

double direction = 0; // Diferencia de velocidad de las ruedas para girar
double setPoint = 0; // Set Point de velocidad general de las ruedas

double setPoint_R = 0; //Set point de velocidad de la rueda DERECHA
double error_R = 0.0; // Error entre la velocidad y el setpoint
double error1_R = 0.0; // Error anterior
double error2_R = 0.0; // Error anterior anterior
double pidSignal_R = 0.0; // Resultado del PID
double pidSignal1_R = 0.0; // Resultado anterior del PID

double setPoint_L = 0; //Set point de velocidad de la rueda IZQUIERDA
double error_L = 0.0; // Error entre la velocidad y el setpoint
double error1_L = 0.0; // Error anterior
double error2_L = 0.0; // Error anterior anterior
double pidSignal_L = 0.0; // Resultado del PID
double pidSignal1_L = 0.0; // Resultado anterior del PID

double rpm_R = 0.0; // Velocidad de la rueda DERECHA
double rpm_L = 0.0; // Velocidad de la rueda IZQUIERDA

uint16_t output_R; // Salida del PID en forma de PWM
uint16_t output_L; //Salida del PID en forma de PWM

uint32_t brakeCount; // Cuenta para activar el freno en caso de desconexion con el mando

// Inicializar objetos
CRC32 crc; // Inicializar objeto CRC32 para la deteccion de errores

void setup() {
  
  // Inizializar perifericos
  Serial.begin(9600); // Inicializar la comunicacion serial
  
  //Configuracion de perifericos
  configureReles();
  configureLORA(); //Configurar el modulo LORA
  configureEncoderInterrupts(); //Configurar las interrupciones por los encoders
  configurePWMTimers(); //Configurar las salidas PWM a 16 bits
  configureTimerInterrupts(); //configurar las interrupciones por timers

  currentState = Run;
}

void loop() {

  //Recibir el mensaje con los paremetros
  while (Serial3.read() != 0xAA) { // Esperar a la siguiente cabecera del mensaje
    
    brakeCount++;
    
    if (brakeCount > 75000) {

      digitalWrite(BRAKE_PIN, LOW);      

    }
  } 
  brakeCount = 0;
  //digitalWrite(BRAKE_PIN, HIGH);


  Serial3.readBytes((uint8_t*)&receivedParameters, sizeof(receivedParameters)); // Leer el payload
  Serial3.readBytes((uint8_t*)&receivedChecksum, sizeof(receivedChecksum));   // Leer el checksum

  // Verificar el checksum del mensaje y guardar los valores
  uint32_t calculatedChecksum = crc.calculate((uint8_t*)&receivedParameters, sizeof(receivedParameters));

  if (receivedChecksum == calculatedChecksum) {

    setPoint = (receivedParameters.Throttle - 511) * ((MAX_SETPOINT) / 511.0);
    direction = (receivedParameters.Direction - 511) * (DIR_DELTA / 511.0);

  } else {

    //Serial.println("Fallo de verificación de checksum. Mensaje corrupto.");

  }
  
  if (setPoint <= 5 && setPoint >= -5 && direction <= 5 && direction >= -5) {
    
    digitalWrite(BRAKE_PIN, LOW);
    
    //setPoint_R = 0; //Set point de velocidad de la rueda DERECHA
    error_R = 0.0; // Error entre la velocidad y el setpoint
    error1_R = 0.0; // Error anterior
    error2_R = 0.0; // Error anterior anterior
    pidSignal_R = 0.0; // Resultado del PID
    pidSignal1_R = 0.0; // Resultado anterior del PID

    //setPoint_L = 0; //Set point de velocidad de la rueda IZQUIERDA
    error_L = 0.0; // Error entre la velocidad y el setpoint
    error1_L = 0.0; // Error anterior
    error2_L = 0.0; // Error anterior anterior
    pidSignal_L = 0.0; // Resultado del PID
    pidSignal1_L = 0.0; // Resultado anterior del PID

  } else {

    digitalWrite(BRAKE_PIN, HIGH);

  }

  if (rpm_R == 0.0 && rpm_L == 0.0) {

    if (setPoint <= -5) {

      digitalWrite(REVERSE_R_PIN, LOW);
      digitalWrite(REVERSE_L_PIN, LOW);

    } else {

      digitalWrite(REVERSE_R_PIN, HIGH);
      digitalWrite(REVERSE_L_PIN, HIGH);

    }

  }
  /*
  if (rpm_R == 0.0 && rpm_L == 0.0) {

    executeState();

  }
  */
  /*
  if (setPoint < -100) {

    digitalWrite(BRAKE_PIN, LOW); // Activar el freno

  }
  else{

    digitalWrite(BRAKE_PIN, HIGH); // Desactivar el freno

  }
  */

}

ISR(TIMER1_COMPA_vect) { // Rutina de interrupción por comparacion de Timer1

  // Reiniciar el contador
  TCNT1 = 0;

  // Calcular RPMs
  rpm_R = (encoderCount_R * SCALER_R) * 60 / (80.0 * Tm); // 40 pulsos por vuelta
  encoderCount_R = 0; // Reiniciar contador de pulsos

  rpm_L = (encoderCount_L * SCALER_L) * 60 / (80.0 * Tm); // 40 pulsos por vuelta
  encoderCount_L = 0; // Reiniciar contador de pulsos

  // Calcular los set points
  if (direction <= 10 && direction >= -10) direction = 0;

  setPoint_R = setPoint + direction;
  setPoint_L = setPoint - direction;

  if (setPoint_R > MAX_SETPOINT) setPoint_R = MAX_SETPOINT;
  if (setPoint_R <= 10.0 && setPoint_R >= -100) setPoint_R = -90.0;
  if (setPoint_R <= -100) setPoint_R = -1 * setPoint_R;
  if (setPoint_L > MAX_SETPOINT) setPoint_L = MAX_SETPOINT;
  if (setPoint_L <= 10.0 && setPoint_L >= -100) setPoint_L = -90.0;
  if (setPoint_L <= -100) setPoint_L = -1 * setPoint_L;

  //Control PID
  error_R = setPoint_R - rpm_R;
  pidSignal_R = pidSignal1_R + (Kp_R + Kd_R / Tm) * error_R + (-Kp_R + Ki_R * Tm - 2*Kd_R / Tm) * error1_R + (Kd_R / Tm) * error2_R;

  if (pidSignal_R > (MAX_TORQ)) pidSignal_R = (MAX_TORQ);
  if (pidSignal_R < 10.0) pidSignal_R = 0.0;

  pidSignal1_R = pidSignal_R;
  error2_R = error1_R;
  error1_R = error_R;

  error_L = setPoint_L - rpm_L;
  pidSignal_L = pidSignal1_L + (Kp_L + Kd_L / Tm) * error_L + (-Kp_L + Ki_L * Tm - 2*Kd_L / Tm) * error1_L + (Kd_L / Tm) * error2_L;

  if (pidSignal_L > (MAX_TORQ)) pidSignal_L = (MAX_TORQ);
  if (pidSignal_L < 10.0) pidSignal_L = 0.0;

  pidSignal1_L = pidSignal_L;
  error2_L = error1_L;
  error1_L = error_L;

  //Convertir el resultado del PID a PWM para el actuador
  output_L = map(pidSignal_L, 0.0, MAX_TORQ, 6500, 14333);
  output_R = map(pidSignal_R, 0.0, MAX_TORQ, 6500, 14333);

  //Enviar el PWM a las ruedas
  analogWrite(PWM_R_PIN, output_R);
  analogWrite(PWM_L_PIN, output_L);
  
  //Imprimir los valores internos
  Serial.print(setPoint);
  Serial.print("\t");
  Serial.print(direction);
  Serial.print("\t");
  
  Serial.print(setPoint_R);
  Serial.print("\t");
  Serial.print(setPoint_L);
  Serial.print("\t");
  
  Serial.print(pidSignal_R);
  Serial.print("\t");
  Serial.print(pidSignal_L);
  Serial.print("\t"); 

  Serial.print(rpm_R);
  Serial.print("\t");
  Serial.println(rpm_L);
  
}

// Funciones

void updateEncoder_R() {
  // Actualizar el contador de pulsos del ENC_R_PIN
  encoderCount_R++;
}

void updateEncoder_L() {
  // Actualizar el contador de pulsos del ENC_R_PIN
  encoderCount_L++;
}

void configureReles() {

  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(IGNITION_PIN, OUTPUT);
  pinMode(REVERSE_R_PIN, OUTPUT);
  pinMode(REVERSE_L_PIN, OUTPUT);

  digitalWrite(BRAKE_PIN, HIGH);
  digitalWrite(IGNITION_PIN, LOW);
  digitalWrite(REVERSE_R_PIN, HIGH);
  digitalWrite(REVERSE_L_PIN, HIGH);

}

void configureLORA() {
  //Comunicacion serial con el modulo LORA
  Serial3.begin(9600);

  // Pins para el modo de transmision
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  // Receiving and Transmitting mode
  digitalWrite(M0, LOW);
  digitalWrite(M0, LOW);

}

void configureEncoderInterrupts() {
  // Pins para los encoders
  pinMode(ENC_R_PIN, INPUT); 
  pinMode(ENC_L_PIN, INPUT);

  //Habilitar las interrucpiones externas para los encoders
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), updateEncoder_R, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), updateEncoder_L, CHANGE);

}

void configureTimerInterrupts() {
  // Configuración del Timer1 por comparacion
  cli();          // Deshabilitar interrupciones globales
  TCCR1A = 0x00;  // Configurar Timer1 en modo normal
  TCCR1B = 0x04;  // Prescaler de 256
  TCNT1  = 0x00;  // Reiniciar el contador
  TIMSK1 = 0x02;  // Habilitar interrupción por COMPARACION
  OCR1A  = 6250;  // Valor calculado para generar la interrupcion cada 100ms
  sei();          // Habilitar interrupciones globales

}

void configurePWMTimers() {
  // Configuracion de un PWM de 16bits de resolucion para el pin 3
  cli();
  pinMode(PWM_R_PIN, OUTPUT);
  TCCR3A = 0; // Clear Timer/Counter Control Register A
  TCCR3B = 0; // Clear Timer/Counter Control Register B
  TCCR3A |= (1 << WGM31); // Set WGM bits to 1110 (Fast PWM, TOP = ICR3)
  TCCR3B |= (1 << WGM32) | (1 << WGM33);
  
  // Set prescaler to 1
  TCCR3B |= (1 << CS30);
  
  // Set ICR3 as TOP (65535 for 16-bit resolution)
  ICR3 = 65535;

  sei();

  // Configuracion de un PWM de 16bits de resolucion para el pin 46
  cli();
  pinMode(PWM_L_PIN, OUTPUT); // pin 45 is connected to OC5A
  TCCR5A = 0; // Clear Timer/Counter Control Register A
  TCCR5B = 0; // Clear Timer/Counter Control Register B
  TCCR5A |= (1 << WGM51); // Set WGM bits to 1110 (Fast PWM, TOP = ICR5)
  TCCR5B |= (1 << WGM52) | (1 << WGM53);

  // Set prescaler to 1
  TCCR5B |= (1 << CS50);

  // Set ICR5 as TOP (65535 for 16-bit resolution)
  ICR5 = 65535;

  sei();

}

void executeState() {
  switch (currentState) {
    case Run:
      //Serial.println("RUN");
      if (direction < 100 && direction > -100 && !ToggleForward_Blocked){
        nextState = ToggleForward;
      }
      if (direction < -200 && !ToggleRight_Blocked){
        nextState = ToggleRight;
      }
      if (direction > 200 && !ToggleLeft_Blocked){
        nextState = ToggleLeft;
      }
      //Serial.println("Run mode");
      break;

    case ToggleForward:
      digitalWrite(IGNITION_PIN, HIGH); // Apagar los controladores
      delay(300); // Esperar
      digitalWrite(REVERSE_L_PIN, HIGH); // Desconectar la reversa de L
      digitalWrite(REVERSE_R_PIN, HIGH); // Desconectar la reversa de R
      delay(300); // Esperar
      digitalWrite(IGNITION_PIN, LOW); // Encender
      ToggleForward_Blocked = true; // Bloquear este estado y desbloquear los otros
      ToggleRight_Blocked = false;
      ToggleLeft_Blocked = false;
      ToggleBackward_Blocked = false;
      nextState = Run; // Volver al estado Run en el siguiente ciclo
      //Serial.println("Listo para avanzar hacia adelante");
      break;

    case ToggleRight:
      digitalWrite(IGNITION_PIN, HIGH); // Apagar los controladores
      delay(300); // Esperar
      digitalWrite(REVERSE_L_PIN, HIGH); // Desconectar la reversa de L
      digitalWrite(REVERSE_R_PIN, LOW); // Conectar la reversa de R
      delay(300); // Esperar
      digitalWrite(IGNITION_PIN, LOW); // Encender
      ToggleForward_Blocked = false;
      ToggleRight_Blocked = true; // Bloquear este estado y desbloquear los otros
      ToggleLeft_Blocked = false;
      ToggleBackward_Blocked = false;
      nextState = Run; // Volver al estado Run en el siguiente ciclo
      //Serial.println("Listo para avanzar girar a la derecha");
      break;

    case ToggleLeft:
      digitalWrite(IGNITION_PIN, HIGH); // Apagar los controladores
      delay(300); // Esperar
      digitalWrite(REVERSE_L_PIN, LOW); // Conectar la reversa de L
      digitalWrite(REVERSE_R_PIN, HIGH); // Desconecctar la reversa de R
      delay(300); // Esperar
      digitalWrite(IGNITION_PIN, LOW); // Encender
      ToggleForward_Blocked = false;
      ToggleRight_Blocked = false;
      ToggleLeft_Blocked = true; // Bloquear este estado y desbloquear los otros
      ToggleBackward_Blocked = false;
      nextState = Run; // Volver al estado Run en el siguiente ciclo
      //Serial.println("Listo para avanzar a la izquierda");
      break;

    case ToggleBackward:
      digitalWrite(IGNITION_PIN, HIGH); // Apagar los controladores
      delay(300); // Esperar
      digitalWrite(REVERSE_L_PIN, HIGH); // Desconectar la reversa de L
      digitalWrite(REVERSE_R_PIN, HIGH); // Conectar la reversa de R
      delay(300); // Esperar
      digitalWrite(IGNITION_PIN, LOW); // Encender
      ToggleForward_Blocked = false;
      ToggleRight_Blocked = false;
      ToggleLeft_Blocked = false;
      ToggleBackward_Blocked = true; // Bloquear este estado y desbloquear los otros
      nextState = Run; // Volver al estado Run en el siguiente ciclo
      //Serial.println("Listo para avanzar hacia atras");
      break;

  }
  currentState = nextState;
}

  
  
  
  
