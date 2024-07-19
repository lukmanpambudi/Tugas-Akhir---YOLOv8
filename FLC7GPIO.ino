  #include <IBusBM.h>
#include <Fuzzy.h>

IBusBM IBus;

int otomatis = 25;

// Pake driver sek iso maju tok
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
int turnDuration = 4000;
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

FuzzySet *Kiri_Tajam = new FuzzySet(-60, -60, -20, -15);
FuzzySet *Kiri = new FuzzySet(-25, -20, -15, -10);
FuzzySet *Lurus = new FuzzySet(-15, -10, 10, 15);
FuzzySet *Kanan = new FuzzySet(10, 15, 20, 25);
FuzzySet *Kanan_Tajam = new FuzzySet(20, 25, 60, 60);

//// Input membership function untuk delta_error
FuzzySet *Delta_Kiri_Tajam = new FuzzySet(-60, -60, -20, -15);
FuzzySet *Delta_Kiri = new FuzzySet(-25, -20, -15, -10);
FuzzySet *Delta_Lurus = new FuzzySet(-15, -10, 10, 15);
FuzzySet *Delta_Kanan = new FuzzySet(10, 15, 20, 25);
FuzzySet *Delta_Kanan_Tajam = new FuzzySet(20, 25, 60, 60);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//// Output membership function (kecepatan motor kanan)
//FuzzySet *RightStop = new FuzzySet(0, 0, 5, 10);
FuzzySet *RightPelan = new FuzzySet(0, 0, 20, 40);
FuzzySet *RightSedang = new FuzzySet(20, 40, 60, 80);
FuzzySet *RightCepat = new FuzzySet(60, 80, 100, 120);


//// Output membership function (kecepatan motor kiri)
//FuzzySet *LeftStop = new FuzzySet(0, 0, 0.5, 1);
FuzzySet *LeftPelan = new FuzzySet(0, 0, 20, 40);
FuzzySet *LeftSedang = new FuzzySet(20, 40, 60, 80);
FuzzySet *LeftCepat = new FuzzySet(60, 80, 100, 120);

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
  MotorKanan->addFuzzySet(RightPelan);
  MotorKanan->addFuzzySet(RightSedang);
  MotorKanan->addFuzzySet(RightCepat);
  fuzzy->addFuzzyOutput(MotorKanan);

  FuzzyOutput *MotorKiri = new FuzzyOutput(2);
  MotorKiri->addFuzzySet(LeftPelan);
  MotorKiri->addFuzzySet(LeftSedang);
  MotorKiri->addFuzzySet(LeftCepat);
  fuzzy->addFuzzyOutput(MotorKiri);

  // Menambahkan aturan fuzzy menggunakan fungsi
  addFuzzyRule(1, Kiri_Tajam, Delta_Kiri_Tajam, RightCepat, LeftPelan); //Kiri_Tajam
  addFuzzyRule(2, Kanan_Tajam, Delta_Kanan_Tajam, RightPelan, LeftCepat); //Kanan_Tajam
  //////////////// Lurus
  addFuzzyRule(3, Kiri, Delta_Kanan_Tajam, RightSedang, LeftSedang);
  addFuzzyRule(4, Lurus, Delta_Kiri, RightSedang, LeftSedang);
  addFuzzyRule(5, Lurus, Delta_Lurus, RightSedang, LeftSedang);
  addFuzzyRule(6, Lurus, Delta_Kanan, RightSedang, LeftSedang);
  addFuzzyRule(7, Kanan, Delta_Kiri_Tajam, RightSedang, LeftSedang);
  //////////////// Kiri
  addFuzzyRule(8, Kiri_Tajam, Delta_Kiri, RightCepat, LeftPelan);
  addFuzzyRule(9, Kiri_Tajam, Delta_Lurus, RightSedang, LeftPelan);
  addFuzzyRule(10, Kiri_Tajam, Delta_Kanan, RightSedang, LeftPelan);
  addFuzzyRule(11, Kiri_Tajam, Delta_Kanan_Tajam, RightSedang, LeftPelan);
  addFuzzyRule(12, Kiri, Delta_Kiri_Tajam, RightSedang, LeftPelan);
  addFuzzyRule(13, Kiri, Delta_Kiri, RightSedang, LeftPelan);
  addFuzzyRule(14, Kiri, Delta_Lurus, RightSedang, LeftPelan);
  addFuzzyRule(15, Kiri, Delta_Kanan_Tajam, RightSedang, LeftPelan);
  addFuzzyRule(16, Lurus, Delta_Kiri_Tajam, RightSedang, LeftPelan);
  //////////////// Kanan
  addFuzzyRule(17, Lurus, Delta_Kanan_Tajam, RightPelan, LeftSedang);
  addFuzzyRule(18, Kanan, Delta_Kiri, RightPelan, LeftSedang);
  addFuzzyRule(19, Kanan, Delta_Lurus, RightPelan, LeftSedang);
  addFuzzyRule(20, Kanan, Delta_Kanan, RightPelan, LeftSedang);
  addFuzzyRule(21, Kanan, Delta_Kanan_Tajam, RightPelan, LeftSedang);
  addFuzzyRule(22, Kanan_Tajam, Delta_Kiri_Tajam, RightPelan, LeftSedang);
  addFuzzyRule(23, Kanan_Tajam, Delta_Kiri, RightPelan, LeftSedang);
  addFuzzyRule(24, Kanan_Tajam, Delta_Lurus, RightPelan, LeftSedang);
  addFuzzyRule(25, Kanan_Tajam, Delta_Kanan, RightPelan, LeftCepat); //belok kanan tajam
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

  if (ch_5 < 1500) {
    digitalWrite(otomatis, HIGH);
    digitalWrite(ledKuning, LOW);
    digitalWrite(ledHijau, HIGH);
    ch_1 = 1500;
    ch_3 = 1500;
//    ch_5 = 1500;
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
          Serial.println(currentError);
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

          if (output1 >= 60 && output2 >= 0 && output2 <= 40) { // BelokKiriTajam
            Serial.println("Belok Kiri Tajam");
            pwmValueKanan = 70; // Kecepatan motor kanan tinggi
            pwmValueKiri = 30; // Kecepatan motor kiri sedang
            analogWrite(motorLeftPin1, 0);
            analogWrite(motorLeftPin2, abs(pwmValueKiri));
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 20 && output1 <= 80 && output2 >= 0 && output2 <= 40) { // BelokKiri
            Serial.println("Belok Kiri");
            pwmValueKanan = 60; // Kecepatan motor kanan sedang
            pwmValueKiri = 15; // Kecepatan motor kiri rendah
            analogWrite(motorLeftPin1, 0);
            analogWrite(motorLeftPin2, abs(pwmValueKiri));
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 20 && output1 <= 80 && output2 >= 20 && output2 <= 80) { // MajuLurus
            Serial.println("Maju Lurus");
            pwmValueKanan = 40; // Kecepatan motor kanan sedang
            pwmValueKiri = 40; // Kecepatan motor kiri sedang
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, abs(pwmValueKanan));
            analogWrite(motorRightPin2, 0);

          } else if (output1 >= 0 && output1 <= 40 && output2 >= 20 && output2 <= 80) { // BelokKanan
            Serial.println("Belok Kanan");
            pwmValueKanan = 15; // Kecepatan motor kanan rendah
            pwmValueKiri = 60; // Kecepatan motor kiri sedang
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, 0);
            analogWrite(motorRightPin2, abs(pwmValueKanan));

          } else if (output1 >= 0 && output1 <= 40 && output2 >= 60) { // BelokKananTajam
            Serial.println("Belok Kanan Tajam");
            pwmValueKanan = 30; // Kecepatan motor kanan sedang
            pwmValueKiri = 70; // Kecepatan motor kiri tinggi
            analogWrite(motorLeftPin1,  abs(pwmValueKiri));
            analogWrite(motorLeftPin2, 0);
            analogWrite(motorRightPin1, 0);
            analogWrite(motorRightPin2, abs(pwmValueKanan));
          }



          // Menulis nilai PWM ke motor driver
          //      analogWrite(enA, pwmValueKanan);
          //      analogWrite(enB, pwmValueKiri);

          // Mengatur arah motor
          //      digitalWrite(in1, HIGH);
          //      digitalWrite(in2, LOW);
          //      digitalWrite(in3, HIGH);
          //      digitalWrite(in4, LOW);

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
  analogWrite(motorLeftPin1, 60);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 0);
  analogWrite(motorRightPin2, 28);
}

void kiriUJ() {
  analogWrite(motorLeftPin1, 0);
  analogWrite(motorLeftPin2, 28);
  analogWrite(motorRightPin1, 60);
  analogWrite(motorRightPin2, 0);
}

void majuU() {
  analogWrite(motorLeftPin1, 45);
  analogWrite(motorLeftPin2, 0);
  analogWrite(motorRightPin1, 45);
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
