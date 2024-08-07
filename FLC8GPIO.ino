#include <IBusBM.h>
#include <Fuzzy.h>
IBusBM IBus;

int otomatis = 14;
int motorRightPin1 = 12;
int motorRightPin2 = 13;

int motorLeftPin1 = 18;
int motorLeftPin2 = 19;

// Pompa 1
int relay1Pin = 21;
// Pompa 2
int relay2Pin = 22;
// Fan DC
int relay3Pin = 23;

// Indikator mode manual
int ledKuning = 32;
// Indikator mode otomatis
int ledHijau = 33;

///durasi belok
int turnDuration = 75 00;
//durasi maju
int fwdDuration = 1500;

int motorSpeed;
int motorTurn;

unsigned long lastCommandTime = 0;
unsigned long lastReceiveTime = 0; // Tambahkan variabel untuk waktu terakhir menerima data

unsigned long previousTime = 0;
unsigned long sampleTime = 100; // Interval waktu untuk perhitungan PID (dalam milidetik)

const unsigned long timeoutDuration = 100;
const unsigned long timeoutDurationSTOP = 500;
const unsigned long timeoutDurationError = 500;
const unsigned long stopDuration = 1000; // Durasi untuk berhenti setelah tidak menerima data

String receivedData;
float errorValue;

// Membuat objek Fuzzy/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Fuzzy *fuzzy = new Fuzzy();

// Input membership function untuk error
// Jika error > 15 maka termasuk membership kanan
FuzzySet *Kiri_Tajam = new FuzzySet(-100, -100, -25, -20);
FuzzySet *Kiri = new FuzzySet(-25, -20, -15, -10);
FuzzySet *Lurus = new FuzzySet(-15, -10, 10, 15);
FuzzySet *Kanan = new FuzzySet(10, 15, 20, 25);
FuzzySet *Kanan_Tajam = new FuzzySet(20, 25, 100, 100);

//// Input membership function untuk delta_error
FuzzySet *Delta_Kiri_Tajam = new FuzzySet(-100, -100, -25, -20);
FuzzySet *Delta_Kiri = new FuzzySet(-25, -20, -15, -10);
FuzzySet *Delta_Lurus = new FuzzySet(-15, -10, 10, 15);
FuzzySet *Delta_Kanan = new FuzzySet(10, 15, 20, 25);
FuzzySet *Delta_Kanan_Tajam = new FuzzySet(20, 25, 100, 100);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Output membership function (kecepatan motor kanan)
FuzzySet *RightStop = new FuzzySet(0, 5, 15, 20);
FuzzySet *RightPelan = new FuzzySet(15, 20, 30, 35);
FuzzySet *RightSedang = new FuzzySet(30, 35, 45, 55);
FuzzySet *RightCepat = new FuzzySet(45, 55, 65, 70);
FuzzySet *RightMax = new FuzzySet(65, 70, 80, 85);

// Output membership function (kecepatan motor kiri)
FuzzySet *LeftStop = new FuzzySet(0, 5, 15, 20);
FuzzySet *LeftPelan = new FuzzySet(15, 20, 30, 35);
FuzzySet *LeftSedang = new FuzzySet(30, 35, 45, 55);
FuzzySet *LeftCepat = new FuzzySet(45, 55, 65, 70);
FuzzySet *LeftMax = new FuzzySet(65, 70, 80, 85);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float previousError = 0; // Menyimpan nilai error sebelumnya

// Channel FS
int ch_3, ch_1, ch_5, ch_6, ch_7, ch_8;

void addFuzzyRule(int ruleNumber, FuzzySet* input1, FuzzySet* input2, FuzzySet* output1, FuzzySet* output2) {
  FuzzyRuleAntecedent *antecedent = new FuzzyRuleAntecedent();
  antecedent->joinWithAND(input1, input2);

  FuzzyRuleConsequent *consequent = new FuzzyRuleConsequent();
  consequent->addOutput(output1);
  consequent->addOutput(output2);

  FuzzyRule *fuzzyRule = new FuzzyRule(ruleNumber, antecedent, consequent);
  fuzzy->addFuzzyRule(fuzzyRule);
}
void setup() {
  Serial.begin(115200);
  IBus.begin(Serial2, 1);

  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(ledKuning, OUTPUT);
  pinMode(ledHijau, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  digitalWrite(relay3Pin, LOW);
  digitalWrite(ledKuning, LOW);
  digitalWrite(ledHijau, LOW);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 0);
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 0);

  // Menambahkan FuzzyInput untuk error
  FuzzyInput *error = new FuzzyInput(1);
  error->addFuzzySet(Kiri_Tajam);
  error->addFuzzySet(Kiri);
  error->addFuzzySet(Lurus);
  error->addFuzzySet(Kanan);
  error->addFuzzySet(Kanan_Tajam);
  fuzzy->addFuzzyInput(error);

  // Menambahkan FuzzyInput untuk delta_error
  FuzzyInput *delta_error = new FuzzyInput(2);
  delta_error->addFuzzySet(Delta_Kiri_Tajam);
  delta_error->addFuzzySet(Delta_Kiri);
  delta_error->addFuzzySet(Delta_Lurus);
  delta_error->addFuzzySet(Delta_Kanan);
  delta_error->addFuzzySet(Delta_Kanan_Tajam);
  fuzzy->addFuzzyInput(delta_error);

  // Menambahkan FuzzyOutput untuk motor kanan dan kiri
  FuzzyOutput *MotorKanan = new FuzzyOutput(1); // Output 1 untuk Motor Kanan
  MotorKanan->addFuzzySet(RightStop);
  MotorKanan->addFuzzySet(RightPelan);
  MotorKanan->addFuzzySet(RightSedang);
  MotorKanan->addFuzzySet(RightCepat);
  MotorKanan->addFuzzySet(RightMax);
  fuzzy->addFuzzyOutput(MotorKanan);

  FuzzyOutput *MotorKiri = new FuzzyOutput(2);
  MotorKiri->addFuzzySet(LeftStop);
  MotorKiri->addFuzzySet(LeftPelan);
  MotorKiri->addFuzzySet(LeftSedang);
  MotorKiri->addFuzzySet(LeftCepat);
  MotorKiri->addFuzzySet(LeftMax);
  fuzzy->addFuzzyOutput(MotorKiri);

  // Menambahkan aturan fuzzy menggunakan fungsi
  addFuzzyRule(1, Kiri_Tajam, Delta_Kiri_Tajam, RightMax, LeftPelan); //Kiri_Tajam
  addFuzzyRule(2, Kanan_Tajam, Delta_Kanan_Tajam, RightPelan, LeftMax); //Kanan_Tajam
  //////////////// Lurus
  addFuzzyRule(3, Kiri, Delta_Kanan_Tajam, RightSedang, LeftPelan);
  addFuzzyRule(4, Lurus, Delta_Kiri, RightSedang, LeftSedang);
  addFuzzyRule(5, Lurus, Delta_Lurus, RightSedang, LeftSedang);
  addFuzzyRule(6, Lurus, Delta_Kanan, RightSedang, LeftSedang);
  addFuzzyRule(7, Kanan, Delta_Kiri_Tajam, RightSedang, LeftSedang);
  //////////////// Kiri
  addFuzzyRule(8, Kiri_Tajam, Delta_Kiri, RightMax, LeftPelan); //Belok Kiri Tajam
  addFuzzyRule(9, Kiri_Tajam, Delta_Lurus, RightMax, LeftPelan); //Belok Kiri Tajam
  addFuzzyRule(10, Kiri_Tajam, Delta_Kanan, RightCepat, LeftStop);
  addFuzzyRule(11, Kiri_Tajam, Delta_Kanan_Tajam, RightCepat, LeftStop);
  addFuzzyRule(12, Kiri, Delta_Kiri_Tajam, RightCepat, LeftStop);
  addFuzzyRule(13, Kiri, Delta_Kiri, RightCepat, LeftStop);
  addFuzzyRule(14, Kiri, Delta_Lurus, RightCepat, LeftStop);
  addFuzzyRule(15, Kiri, Delta_Kanan_Tajam, RightCepat, LeftStop);
  addFuzzyRule(16, Lurus, Delta_Kiri_Tajam, RightCepat, LeftStop);
  //////////////// Kanan
  addFuzzyRule(17, Lurus, Delta_Kanan_Tajam, RightStop, LeftCepat);
  addFuzzyRule(18, Kanan, Delta_Kiri, RightStop, LeftCepat);
  addFuzzyRule(19, Kanan, Delta_Lurus, RightStop, LeftCepat);
  addFuzzyRule(20, Kanan, Delta_Kanan, RightStop, LeftCepat);
  addFuzzyRule(21, Kanan, Delta_Kanan_Tajam, RightStop, LeftCepat);
  addFuzzyRule(22, Kanan_Tajam, Delta_Kiri_Tajam, RightStop, LeftCepat);
  addFuzzyRule(23, Kanan_Tajam, Delta_Kiri, RightStop, LeftCepat);
  addFuzzyRule(24, Kanan_Tajam, Delta_Lurus, RightPelan, LeftMax); //belok kanan tajam
  addFuzzyRule(25, Kanan_Tajam, Delta_Kanan, RightPelan, LeftMax); //belok kanan tajam
}

void loop() {
  // Maju mundur
  ch_3 = IBus.readChannel(2);

  // Belok kanan kiri
  ch_1 = IBus.readChannel(0);

  // Auto manual
  ch_5 = IBus.readChannel(4);

  // Fan DC
  ch_6 = IBus.readChannel(5);

  // Pompa 1
  ch_7 = IBus.readChannel(6);

  // Pompa 2
  ch_8 = IBus.readChannel(7);

  motorSpeed = map(ch_3, 1000, 2000, -80, 80);
  motorTurn = map(ch_1, 1000, 2000, -80, 80);

  if (ch_5 > 1500) {
    digitalWrite(otomatis, HIGH);
    digitalWrite(ledKuning, LOW);
    digitalWrite(ledHijau, HIGH);
    ch_1 = 1500;
    ch_3 = 1500;
    ch_5 = 1500;
    ch_6 = 1500;
    ch_7 = 1500;
    ch_8 = 1500;
    //    otomatis();
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - previousTime;

    // Pastikan waktu yang telah berlalu mencapai sampleTime sebelum melakukan perhitungan PID
    if (elapsedTime >= sampleTime) {
      // Reset previousTime ke currentTime
      previousTime = currentTime;

      if (Serial.available() > 0) {
        receivedData = Serial.readStringUntil('\n');
        //      Serial.print("receivedData: ");
        //      Serial.println(receivedData);
        lastReceiveTime = millis();

        if (receivedData.startsWith("error_")) {
          String errorString = receivedData.substring(6); // Mengambil substring setelah "error_"
          float currentError = errorString.toFloat(); // Mengkonversi substring ke nilai float
          //      float deltaError = previousError - 0; // Menghitung delta_error dari nilai previousError dikurangi 0
          float deltaError = currentError - previousError; // Menghitung delta error dari error aktual - error sebelumnya


          Serial.print("error : ");
          Serial.print(currentError);
          if (currentError >= -15 && currentError <= 15){
            Serial.println("(Lurus)");          
          }
          else if(currentError >= 15 && currentError <= 25){
            Serial.println("(Kanan)");
          }
          else if(currentError <= -15 && currentError <= -25){
            Serial.println("(Kiri)");
          }
          else if (currentError >= 25){
            Serial.println("(Kanan tajam)");
          }
          else if (currentError >= -25){
            Serial.println("(Kiri tajam)");
          }
          
          Serial.print("delta error : ");
          Serial.println(deltaError);

          fuzzy->setInput(1, currentError); // Mengatur input fuzzy untuk error
          fuzzy->setInput(2, deltaError); // Mengatur input fuzzy untuk delta_error

          fuzzy->fuzzify(); // Melakukan fuzzifikasi

          // Mendefuzzifikasi output /////////////////////////////////////////////////////////////////////////////////////////////////////////////
          float output1 = fuzzy->defuzzify(1);
          float output2 = fuzzy->defuzzify(2);

          Serial.print("Output fuzzy1: ");
          Serial.println(output1);
          Serial.print("Output fuzzy2: ");
          Serial.println(output2);

          // Mengatur kecepatan motor berdasarkan hasil defuzzifikasi dan aturan fuzzy
          int pwmValueKanan = 0;
          int pwmValueKiri = 0;

          if (output1 >= 70 && output2 >= 20 && output2 <= 35) { // BelokKiriTajam
            Serial.println("Belok Kiri Tajam");
            pwmValueKanan = output1; // Kecepatan motor kanan max
            pwmValueKiri = output2; // Kecepatan motor kiri pelan
            Serial.print("PWM Kanan: ");
            Serial.print(pwmValueKanan);
            Serial.print("PWM Kiri: ");
            Serial.println(pwmValueKiri);
            analogWrite(motorLeftPin1, 0);
            analogWrite(motorLeftPin2, abs(pwmValueKiri));
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 55 && output1 <= 70 && output2 >= 0 && output2 <= 20) { // BelokKiri
            Serial.println("Belok Kiri");
            pwmValueKanan = output1; // Kecepatan motor kanan cepat
            pwmValueKiri = output2; // Kecepatan motor kiri stop
             Serial.print("PWM Kanan: ");
            Serial.print(pwmValueKanan);
            Serial.print("PWM Kiri: ");
            Serial.println(pwmValueKiri);
            analogWrite(motorLeftPin1, 0);
            analogWrite(motorLeftPin2, abs(pwmValueKiri));
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 35 && output1 <= 55 && output2 >= 35 && output2 <= 55) { // MajuLurus
            Serial.println("Maju Lurus");
            pwmValueKanan = output1; // Kecepatan motor kanan sedang
            pwmValueKiri = output2; // Kecepatan motor kiri sedang
            Serial.print("PWM Kanan: ");
            Serial.print(pwmValueKanan);
            Serial.print("PWM Kiri: ");
            Serial.println(pwmValueKiri);
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 0 && output1 <= 20 && output2 >= 55 && output2 <= 70) { // BelokKanan
            Serial.println("Belok Kanan");
            pwmValueKanan = output1; // Kecepatan motor kanan stop
            pwmValueKiri = output2; // Kecepatan motor kiri cepat
            Serial.print("PWM Kanan: ");
            Serial.print(pwmValueKanan);
            Serial.print("PWM Kiri: ");
            Serial.println(pwmValueKiri);
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, 0);
            analogWrite(motorRightPin2, abs(pwmValueKanan));

          } else if (output1 >= 20 && output1 <= 35 && output2 >= 70) { // BelokKananTajam
            Serial.println("Belok Kanan Tajam");
            pwmValueKanan = output1; // Kecepatan motor kanan stop
            pwmValueKiri = output2; // Kecepatan motor kiri cepat
            Serial.print("PWM Kanan: ");
            Serial.print(pwmValueKanan);
            Serial.print("PWM Kiri: ");
            Serial.println(pwmValueKiri);
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, 0);
            analogWrite(motorRightPin2, abs(pwmValueKanan));
          }
          previousError = currentError;
        }

        else if(receivedData.startsWith("MD")) {
          String command1 = receivedData;
          Serial.print("command: ");
          Serial.println(command1);
          majuU();
        }
        else if (receivedData.startsWith("KI")){
          String command2 = receivedData;
          Serial.print("command: ");
          Serial.println(command2);
          kiriUJ();
          delay(turnDuration);
        }
        else if(receivedData.startsWith("KA")){
          String command3 = receivedData;
          Serial.print("command: ");
          Serial.println(command3);
          kananUJ();
          delay(turnDuration);
        }
      }

      // Stop motor jika tidak ada data yang diterima lagi
      if (millis() - lastReceiveTime > stopDuration && lastReceiveTime != 0) {
        stopp();
        delay(100);
      }

    }
  }

  else {
    digitalWrite(ledKuning, HIGH);
    digitalWrite(ledHijau, LOW);
    if (motorTurn > 5) {
      kanan(motorTurn, motorTurn);
    } else if (motorTurn < -5) {
      kiri(motorTurn, motorTurn);
    } else if (motorSpeed < 5 && motorSpeed > -5 && motorTurn > -5 && motorTurn < 5) {
      stopp();
    } else if (motorSpeed > 5 && motorTurn > -5 && motorTurn < 5) {
      maju(motorSpeed, motorSpeed);
    } else if (motorSpeed < 5 && motorTurn > -5 && motorTurn < 5) {
      mundur(motorSpeed, motorSpeed);
    }

    if (ch_7 > 1500) {
      digitalWrite(relay1Pin, HIGH);
    } else {
      digitalWrite(relay1Pin, LOW);
    }

    if (ch_8 > 1500) {
      digitalWrite(relay2Pin, HIGH);
    } else {
      digitalWrite(relay2Pin, LOW);
    }

    if (ch_6 > 1500) {
      digitalWrite(relay3Pin, HIGH);
    } else {
      digitalWrite(relay3Pin, LOW);
    }
  }

  delay(100);
}

void maju(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, abs(leftSpeed));
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, abs(rightSpeed));
  analogWrite(motorRightPin2, 0);
}

void mundur(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, abs(leftSpeed));
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, abs(rightSpeed));
}

void kanan(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, abs(leftSpeed));
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, abs(rightSpeed));
}

void kiri(int leftSpeed, int rightSpeed) {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, abs(leftSpeed));
  analogWrite(motorRightPin1, abs(rightSpeed));
  analogWrite(motorRightPin2, 0);
}

void kananUJ() {
  analogWrite(motorLeftPin1, 70);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 20);
}

void kiriUJ() {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 20);
  analogWrite(motorRightPin1, 70);
  analogWrite(motorRightPin2, 0);
}

void majuU() {
  analogWrite(motorLeftPin1, 30);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 30);
  analogWrite(motorRightPin2, 0);
}

void stopp() {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 0);
}


void manual() {
  if (ch_5 < 1500) {
    stopp();
  }
}
