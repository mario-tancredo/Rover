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

// Variables de configuracion
const double Tm = 0.1; // Periodo de muestreo

const double Kp = 0.5; // Constante proporcional de la rueda DERECHA
const double Kd = 0.03; // Constante diferencial de la rueda DERECHA
const double Ki = 0.9; // Constante integral de la rueda DERECHA

const double Kp_L = 0.5; // Constante proporcional de la rueda IZQUIERDA
const double Kd_L = 0.03; // Constante diferencial de la rueda IZQUIERDA
const double Ki_L = 0.9; // Constante integral de la rueda IZQUIERDA

const float SCALER_R = 0.75; // Constante para calibracion de la rueda DERECHA
const float SCALER_L = 1.0; // Constante para calibracion de la rueda IZQUIERDA

const float MAX_TORQ = 1500; // Torque máximo (sin unidades) (cambiar solo este parametro no cambia el torque maximo)
const float MAX_SETPOINT = 350; // Set point maximo de RPM de las ruedas (dar maximo 1000);
const float DIR_DELTA = 350; //Diferencia entre los setpoints al girar

// Variables globales
struct RoverParameters {  // Parametros recibidos del mando
  int Throttle;           // Velocidad de las ruedas
  int Direction;          // Direccion de las ruedas
  int Pump;               // Bomba
} receivedParameters;

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


// Inicializar objetos
CRC32 crc; // Inicializar objeto CRC32 para la deteccion de errores

void setup() {
  
  // Inizializar perifericos
  Serial.begin(9600); // Inicializar la comunicacion serial
  
  //Configuracion de perifericos
  configureLORA(); //Configurar el modulo LORA
  configureEncoderInterrupts(); //Configurar las interrupciones por los encoders
  configurePWMTimers(); //Configurar las salidas PWM a 16 bits
  configureTimerInterrupts(); //configurar las interrupciones por timers

}

void loop() {

  //Recibir el mensaje con los paremetros
  while (Serial3.read() != 0xAA); // Esperar a la siguiente cabecera del mensaje
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

  
}

ISR(TIMER1_COMPA_vect) { // Rutina de interrupción por overflow de Timer1

  // Reiniciar el contador
  TCNT1 = 0;

  // Calcular RPMs
  rpm_R = (encoderCount_R * SCALER_R) * 60 / (80.0 * Tm); // 80 pulsos por vuelta
  encoderCount_R = 0; // Reiniciar contador de pulsos

  rpm_L = (encoderCount_L * SCALER_L) * 60 / (80.0 * Tm); // 80 pulsos por vuelta
  encoderCount_L = 0; // Reiniciar contador de pulsos
  
  // Calcular los set points
  if (direction <= 10 && direction >= -10) direction = 0;

  if (setPoint > MAX_SETPOINT) setPoint = MAX_SETPOINT;
  if (setPoint < 10.0) setPoint = 0.0;

  setPoint_R = setPoint + direction;
  setPoint_L = setPoint - direction;

  if (setPoint_R > MAX_SETPOINT) setPoint_R = MAX_SETPOINT;
  if (setPoint_L > MAX_SETPOINT) setPoint_L = MAX_SETPOINT;
  if (setPoint_R < 10.0) setPoint_R = 0.0;
  if (setPoint_L < 10.0) setPoint_L = 0.0;


  //Control PID
  error_R = setPoint_R - rpm_R;
  pidSignal_R = pidSignal1_R + (Kp + Kd / Tm) * error_R + (-Kp + Ki * Tm - 2*Kd / Tm) * error1_R + (Kd / Tm) * error2_R;

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
  
  //Imprimir los numeros internos
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
