/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/basics/ReadAnalogVoltage/
*/

// Teensy - Arduino Code
const int fsrPin1 = A9;
const int fsrPin2 = A8;
const int fsrPin3 = A7;
const int fsrPin4 = A6;
const int fsrPin5 = A5;  // Pino analógico conectado ao FSR

void setup() {
  Serial.begin(115200); // Inicializa comunicação serial
}

void loop() {
  int fsrValue1 = analogRead(fsrPin1);
  int fsrValue2 = analogRead(fsrPin2);
  int fsrValue3 = analogRead(fsrPin3);
  int fsrValue4 = analogRead(fsrPin4);
  int fsrValue5 = analogRead(fsrPin5);

  float voltage1 = fsrValue1 * 3.3 / 1023.0;
  float voltage2 = fsrValue2 * 3.3 / 1023.0;
  float voltage3 = fsrValue3 * 3.3 / 1023.0;
  float voltage4 = fsrValue4 * 3.3 / 1023.0;
  float voltage5 = fsrValue5 * 3.3 / 1023.0;

  // Imprime tudo na mesma linha, separado por tabs
  Serial.print(voltage1);
  Serial.print('\t');
  Serial.print(voltage2);
  Serial.print('\t');
  Serial.print(voltage3);
  Serial.print('\t');
  Serial.print(voltage4);
  Serial.print('\t');
  Serial.println(voltage5);

   // println só no último para nova linha
  delay(1);
  
}

