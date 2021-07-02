// Стратоспутник 04.06.2021
// ========================
// Serial[0] - USB
// Serial[1] - LoRa Radio
// Serial[2] - UART Logger
// Serial[3] - UART GPS
// OneWire[D10] - DS18B20 #0
// OneWire[D10] - DS18B20 #1

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <MS5611.h>
#include <TinyGPS++.h>
#include <si5351.h>
#include <avr/wdt.h>

boolean
//// == Режим Отладки по Serial == //
  DEBUG = true,

//// == Светодиодная панель == //
  enableLedPanel = true,
  
//// == Генератор частот == //
  enableFrequencyGenerator = true,
  
//// == Нагреватель батареи == //
  enableHeater = true,
  
//// == Пережигание для антенн == //
  enablePerejig = false,
  
//// == LoRa радио == //
  enableLoRaRadio = true,
  
//// == Логгер == //
  enableLogger = true,
  
//// == GPS приемник == //
  enableGPS = true,
  
//// == Датчики температуры == //
  enableTemperatureSensors = true,
  
//// == Инерциальные датчики  == //
  enableIMUsensor = true,
  
//// == Барометр == //
  enableBarometerSensor = true;

//// == == == == == == == == == == == ////

// RF output frequency
#define OUT_FREQ 144125.0
// cw dot length in ms
#define CW_DOT_LEN 80
Si5351 frequencyGenerator;
#define BUTTON_PIN 6
#define PEREJIG_PIN 42
#define HEATER_PIN 11
#define MSHU_PIN 7
#define NEEDED_TEMP 15
#define Kp 17
const uint8_t cwSymbTab[][5] = {
  {1, 2},             // 0  A
  {2, 1, 1, 1},       // 1  B
  {2, 1, 2, 1},       // 2  C
  {2, 1, 1},          // 3  D
  {1},                // 4  E
  {1, 1, 2, 1},       // 5  F
  {2, 2, 1},          // 6  G
  {1, 1, 1, 1},       // 7  H
  {1, 1},             // 8  I
  {1, 2, 2, 2},       // 9  J
  {2, 1, 2},          // 10 K
  {1, 2, 1, 1},       // 11 L
  {2, 2},             // 12 M
  {2, 1},             // 13 N
  {2, 2, 2},          // 14 O
  {1, 2, 2, 1},       // 15 P
  {2, 2, 1, 2},       // 16 Q
  {1, 2, 1},          // 17 R
  {1, 1, 1},          // 18 S
  {2},                // 19 T
  {1, 1, 2},          // 20 U
  {1, 1, 1, 2},       // 21 V
  {1, 2, 2},          // 22 W
  {2, 1, 1, 2},       // 23 X
  {2, 1, 2, 2},       // 24 Y
  {2, 2, 1, 1},       // 25 Z
  {2, 2, 2, 2, 2},    // 26 0
  {1, 2, 2, 2, 2},    // 27 1
  {1, 1, 2, 2, 2},    // 28 2
  {1, 1, 1, 2, 2},    // 29 3
  {1, 1, 1, 1, 2},    // 30 4
  {1, 1, 1, 1, 1},    // 31 5
  {2, 1, 1, 1, 1},    // 32 6
  {2, 2, 1, 1, 1},    // 33 7
  {2, 2, 2, 1, 1},    // 34 8
  {2, 2, 2, 2, 1}     // 35 9
};

TinyGPSPlus gps;
boolean gpsLedState = false;

OneWire oneWire(10);
DallasTemperature temperatureSensors(&oneWire);
DeviceAddress temperatureSensorsAddress;
uint8_t ds18b20_0[8] = { 0x28, 0x49, 0x6B, 0x61, 0x0B, 0x00, 0x00, 0x42 };
uint8_t ds18b20_1[8] = { 0x28, 0x69, 0x56, 0x3F, 0x0D, 0x00, 0x00, 0x80 };
uint8_t temperatureSensorsDeviceCount = 0;
float temp0 = 0.0F, temp1 = 0.0F;

MPU6050 mpu6050;
int16_t ax, ay, az, gx, gy, gz;
float axf, ayf, azf, gxf, gyf, gzf;
float pitch, roll, yaw;

MS5611 ms5611;
double referencePressure, realTemperature;
long realPressure;
float absoluteAltitude, relativeAltitude;

int shim = 0;

unsigned long programStartTime, programExecutingTime;
unsigned long loopsCounter = 0;

boolean mustPerejig = false;

// фильтр бегущее среднее с адаптивным коэффициентом
/*float runningAverage(float newVal) {
  static float filVal = newVal;
  float k;
  // резкость фильтра зависит от модуля разности значений
  if(abs(newVal - filVal) > 1.5F) k = 0.9F;
  else k = 0.03F;
  
  filVal += (newVal - filVal) * k;
  return filVal;
}*/

// выводим показания напряжений основных линий и ток с аккумулятора
void printPowerStatus() {
  float _33V = analogRead(A13) / 1024.0F / (6.8F / (4.7F + 6.8F));
  float _5V = analogRead(A12) / 1024.0F / (4.7F / (6.8F + 4.7F));
  float _12V = analogRead(A14) / 1024.0F / (1.1F / (6.8F + 1.1F));
  float ampers = analogRead(A11) / 1024.0F * 5.0F;
  float batAmpers = 5.0F - ampers;
  Serial.print("[POWER] 3.3V, 5V, 12V, BAT I[A], I[A]: ");
  Serial.print(_33V * 4.6F); Serial.print(", ");
  Serial.print(_5V * 4.6F); Serial.print(", ");
  Serial.print(_12V * 4.6F); Serial.print(", ");
  Serial.print(batAmpers); Serial.print(", ");
  Serial.println(ampers);
}

// посылаем символ генератором частоты
void cwSendSym(uint16_t len) {
  frequencyGenerator.output_enable(SI5351_CLK0, 1);
  digitalWrite(36, HIGH);
  delay(len);
  frequencyGenerator.output_enable(SI5351_CLK0, 0);
  digitalWrite(36, LOW);
}

// посылаем строку генератором частоты
void frequencyGeneratorSend(char* msg) {
  for(byte i = 0; i < strlen(msg); i++) cwTxChar(msg[i]);
}

void cwTxChar(char ch) {
  uint8_t cwSym;
  uint8_t tabIndex;
    
  tabIndex = 255;
  if((ch >= 65) && (ch <= 90)) tabIndex = ch - 65;  // A - Z
  if((ch >= 97) && (ch <= 122)) tabIndex = ch - 97;  // a - z
  if((ch >= 48) && (ch <= 57)) tabIndex = ch - 22;  // 0 - 9

  if(tabIndex == 255) {
    delay(CW_DOT_LEN * 3);
    return;
  }
    
  for(byte i = 0; i < 5; i++) {
    cwSym = cwSymbTab[tabIndex][i];
    if(cwSym == 1) cwSendSym(CW_DOT_LEN);
    else if(cwSym == 2) cwSendSym(CW_DOT_LEN * 3);
    else continue;
    delay(CW_DOT_LEN);
  }

  delay(CW_DOT_LEN * 2);
}

void setup() {
  // Независимо от того, включен ли нагрев -> устанавливаем low статус в D42 (5V CH1)
  pinMode(PEREJIG_PIN, OUTPUT);
  digitalWrite(PEREJIG_PIN, LOW);
  //

  wdt_enable(WDTO_8S);
  
  Serial.begin(9600);
  Serial.println("Stratosputnik");
  Serial.println("=============");

  printPowerStatus();
  
  Serial.print("Led panel - ");
  Serial.println(enableLedPanel ? "ON" : "OFF");
  // светодиодная линия
  if(enableLedPanel) {
    for(uint8_t i = 0; i < 7; i++) {
      pinMode(31 + i, OUTPUT);
    }
  }

  Serial.print("Frequency generator - ");
  Serial.println(enableFrequencyGenerator ? "ON" : "OFF");
  // генератор частоты
  if(enableFrequencyGenerator) {
    bool freqGenInit = frequencyGenerator.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
    
    if(!freqGenInit) Serial.println("SI5351 [Frequency generator] connection failed!");
    
    frequencyGenerator.update_status();
    frequencyGenerator.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
    frequencyGenerator.set_correction(80000, SI5351_PLL_INPUT_XO);
    frequencyGenerator.set_freq(OUT_FREQ * 100000ULL, SI5351_CLK0);
    frequencyGenerator.output_enable(SI5351_CLK0, 0);
  }

  Serial.print("Heater - ");
  Serial.println(enableHeater ? "ON": "OFF");
  // нагреватель (уже провели инициализацию ранее)
  if(enableHeater) {
    pinMode(HEATER_PIN, OUTPUT);
  }

  Serial.print("Perejig - ");
  Serial.println(enablePerejig ? "ON": "OFF");
  // пережигание для антенн
  if(enablePerejig) {
    pinMode(BUTTON_PIN, INPUT);
    pinMode(PEREJIG_PIN, OUTPUT);
  }

  Serial.print("LoRa radio - ");
  Serial.println(enableLoRaRadio ? "ON" : "OFF");
  // радио LoRa
  if(enableLoRaRadio) {
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    Serial1.begin(9600);
    Serial1.println("T+,AX,Y,Z,P,R,Y,Tradio,Tbat,Tms5611,AltiRel,AltiGPS,Lat,Lon,Sats,HDOP,Speed,DDMMYY,HHMMSS,Hsig"); // 89 chars length
    Serial.println("Open new <LoRa> UART with speed 9600 bit/s");
  }

  Serial.print("Logger - ");
  Serial.println(enableLogger ? "ON" : "OFF");
  // Logger
  if(enableLogger) {
    Serial2.begin(9600);
    Serial2.println("[Mizar][Log]T+,aX,aY,aZ,gX,gY,gZ,P,R,Y,Tradio,Tbat,Tms5611,Pres,AltiAbs,AltiRel,Sats,HDOP,Lat,Lon,DDMMYY,HHMMSS,AltiGPS,Speed,Hsig"); // 113 chars length
    Serial.println("Open new <Logger> UART with speed 9600 bit/s");
  }

  Serial.print("GPS - ");
  Serial.println(enableGPS ? "ON" : "OFF");
  // GPS
  if(enableGPS) {
    Serial3.begin(9600);
    Serial.println("Open new <GPS> UART with speed 9600 bit/s");
  }

  Serial.print("Temperature sensors - ");
  Serial.println(enableTemperatureSensors ? "ON" : "OFF");
  // датчики температуры  
  if(enableTemperatureSensors) {
    // получаем адрес и температуру со всех датчиков температуры
    temperatureSensors.begin();
    temperatureSensorsDeviceCount = temperatureSensors.getDeviceCount();
    Serial.print("Founded DS18B20's: ");
    Serial.println(temperatureSensorsDeviceCount);
    for(uint8_t i = 0; i < temperatureSensorsDeviceCount; i++) {
      Serial.print("DS18B20 #");
      Serial.print(i);
      Serial.print(" - ");
      temperatureSensors.getAddress(temperatureSensorsAddress, i);
      for(uint8_t j = 0; j < 8; j++) {
        if(temperatureSensorsAddress[j] < 0x10) Serial.print("0");
        Serial.print(temperatureSensorsAddress[j], HEX);
        if(j < 7) Serial.print(":");
      }
      Serial.println();
    }
  }

  Serial.print("IMU sensor - ");
  Serial.println(enableIMUsensor ? "ON" : "OFF");
  // IMU сенсор
  if(enableIMUsensor) {
    Wire.begin();
    if(!mpu6050.testConnection()) Serial.println("MPU6050 [IMU] connection failed!");
    else {
      mpu6050.initialize();
      mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
      mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
      Serial.println("MPU6050 setup complete. With config: +-2g, +-250d/s");
    }
  }

  Serial.print("Barometer sensor - ");
  Serial.println(enableBarometerSensor ? "ON" : "OFF");
  if(enableBarometerSensor) {
    if(!ms5611.begin()) Serial.println("MS5611 [Barometer] conection failed!");
    else {
      referencePressure = ms5611.readPressure();
      Serial.println("MS5611 setup complete.");
      Serial.println("Reading reference pressure...");
    }
  }
  
  Serial.println();
  Serial.println();
}

uint8_t messageCounter = 0;

void loop() {
  programStartTime = millis();
  // очистка светодиода (радио)
  if(enableLoRaRadio) {
    digitalWrite(35, LOW);
  }
  
  // Послать время миссии (по таймеру мк)
  if(DEBUG) {
    Serial.print("[T+]: ");
    Serial.print(millis() / 1000);
    Serial.println("s");
  }
  
  // светодиодная лента (анимация начала цикла)
  if(enableLedPanel) {
    for(uint8_t i = 0; i < 3; i++) {
      digitalWrite(31 + i, HIGH);
      delay(125);
    }
  }

  wdt_reset();
  
  // генератор частоты
  if(enableFrequencyGenerator) {
    if(DEBUG) {
      Serial.println("[Frequency generator] Sending message by frequency generator started");
    }

    char* s[] = { "RT4D", "RT4D", "123", "KK" };
    if(messageCounter > 3) messageCounter = 0;
    frequencyGeneratorSend(s[messageCounter]);
    
    if(DEBUG) {
      Serial.print("[Frequency generator] Sending message by frequency generator complete ");
      Serial.println(messageCounter);
    }

    messageCounter += 1;
  }

  wdt_reset();

  // нагреватель
  if(enableHeater) {
    if(temp1 < NEEDED_TEMP) {
      shim = Kp * (NEEDED_TEMP - (int)temp1);
      if(shim > 255) shim = 255;
      //shim = map(shim, 0, 511, 0, 255);
      digitalWrite(33, HIGH);
      analogWrite(HEATER_PIN, shim);
    } else {
      shim = 0;
      digitalWrite(33, LOW);
      analogWrite(HEATER_PIN, shim);
    }

    if(DEBUG) {
      Serial.print("[HEATER] Current signal on: ");
      Serial.println(shim);
    }
  }

  // пережигание для антенн (ОДИН РАЗ В ПОЛЕТЕ)
  if(enablePerejig) {
    //
    if(!mustPerejig && digitalRead(BUTTON_PIN) == LOW) {
      mustPerejig = true;
      delay(5000);
      digitalWrite(PEREJIG_PIN, HIGH);
      delay(7000);
      digitalWrite(PEREJIG_PIN, LOW);
      if(DEBUG) {
        Serial.println("[PEREJIG] Successfull!");
      }
    }
  }

  wdt_reset();
  
  // передаем данные по радиоканалу (раз в 115 циклов, примерно раз в 5 минут (если учесть что 1 цикл - 2.6s)
  if(enableLoRaRadio /*&& loopsCounter % 115 == 0*/) {
    // mission time in seconds
    Serial1.print(millis() / 1000); Serial1.print(",");
    // Accel X Y Z
    Serial1.print(axf); Serial1.print(","); Serial1.print(ayf); Serial1.print(","); Serial1.print(azf); Serial1.print(",");
    // Pitch, Roll, Yaw
    Serial1.print(pitch); Serial1.print(","); Serial1.print(roll); Serial1.print(","); Serial1.print(yaw); Serial1.print(",");
    // Radio temp, Battery temp, Barometer temp
    Serial1.print(temp0); Serial1.print(","); Serial1.print(temp1); Serial1.print(","); Serial1.print(realTemperature); Serial1.print(",");
    // Altitude relative, Altitude by GPS
    Serial1.print(relativeAltitude); Serial1.print(","); Serial1.print(gps.altitude.meters()); Serial1.print(",");
    // GPS: Latitude, Longitude
    Serial1.print(gps.location.lat(), 5); Serial1.print(","); Serial1.print(gps.location.lng(), 5); Serial1.print(",");
    // GPS: Sats, HDOP, Speed
    Serial1.print(gps.satellites.value()); Serial1.print(","); Serial1.print(gps.hdop.hdop()); Serial1.print(","); Serial1.print(gps.speed.kmph()); Serial1.print(",");
    // GPS: DDMMYY
    Serial1.print(gps.date.day()); Serial1.print(gps.date.month()); Serial1.print(gps.date.year() % 100); Serial1.print(",");
    // GPS: HHMMSS
    Serial1.print(gps.time.hour()); Serial1.print(gps.time.minute()); Serial1.print(gps.time.second()); Serial1.print(",");
    // Hsig
    Serial1.print(shim);
    Serial1.println();
    digitalWrite(35, HIGH);
    if(DEBUG) {
      Serial.println("[LoRa Radio] Radio packet transmitted");
    }
  }

  wdt_reset();
  
  // запись данных на SD карту
  if(enableLogger) {
    Serial2.print("[Mizar][Log]");
    // mission time in seconds
    Serial2.print(millis() / 1000);  Serial2.print(",");
    // accel X, Y, Z
    Serial2.print(axf); Serial2.print(","); Serial2.print(ayf); Serial2.print(","); Serial2.print(azf); Serial2.print(",");
    // gyroscope X, Y, Z
    Serial2.print(gxf); Serial2.print(","); Serial2.print(gyf); Serial2.print(","); Serial2.print(gzf); Serial2.print(",");
    // Pitch, Roll, Yaw
    Serial2.print(pitch); Serial2.print(","); Serial2.print(roll); Serial2.print(","); Serial2.print(yaw); Serial2.print(",");
    // Radio temp, Battery temp, Barometer temp
    Serial2.print(temp0); Serial2.print(","); Serial2.print(temp1); Serial2.print(","); Serial2.print(realTemperature); Serial2.print(",");
    // Pressure, Altitude absolute, Altitude relative
    Serial2.print(realPressure); Serial2.print(","); Serial2.print(absoluteAltitude); Serial2.print(","); Serial2.print(relativeAltitude); Serial2.print(",");
    // GPS: Sats, HDOP
    Serial2.print(gps.satellites.value()); Serial2.print(","); Serial2.print(gps.hdop.hdop()); Serial2.print(",");
    // GPS: Latitude, Longitude
    Serial2.print(gps.location.lat(), 5); Serial2.print(","); Serial2.print(gps.location.lng(), 5); Serial2.print(",");
    // GPS: Date DDMMYY
    Serial2.print(gps.date.day()); Serial2.print(gps.date.month()); Serial2.print(gps.date.year() % 100); Serial2.print(",");
    // GPS: Time HHMMSS
    Serial2.print(gps.time.hour()); Serial2.print(gps.time.minute()); Serial2.print(gps.time.second()); Serial2.print(",");
    // GPS: Altitude by GPS
    Serial2.print(gps.altitude.meters()); Serial2.print(",");
    // GPS: Speed in km/h
    Serial2.print(gps.speed.kmph()); Serial2.print(",");
    // Hsig
    Serial2.print(shim);
    Serial2.println();
    if(DEBUG) {
      Serial.println("[Logger] Writed data to logger");
    }
  }

  wdt_reset();

  // получаем данные GPS
  if(enableGPS) {
    if(DEBUG) {
      Serial.print("[GPS] Sats, HDOP, lat, lon, age, date, altitude[m], course[°], speed[km/h]: ");
      Serial.print(gps.satellites.value()); Serial.print(", ");
      Serial.print(gps.hdop.hdop()); Serial.print(", ");
      Serial.print(gps.location.lat(), 5); Serial.print(", ");
      Serial.print(gps.location.lng(), 5); Serial.print(", ");
      Serial.print(gps.location.age()); Serial.print(", [");
      Serial.print(gps.date.day()); Serial.print(","); Serial.print(gps.date.month()); Serial.print(","); Serial.print(gps.date.year()); Serial.print("], [");
      Serial.print(gps.time.hour()); Serial.print(":"); Serial.print(gps.time.minute()); Serial.print(":"); Serial.print(gps.time.second()); Serial.print("], ");
      Serial.print(gps.altitude.meters()); Serial.print(", ");
      Serial.print(gps.course.deg()); Serial.print(", ");
      Serial.println(gps.speed.kmph());
    }
    
    long gpsTimeout = 1000;
    unsigned long startTime = millis();
    do {
      while(Serial3.available()) {
        gps.encode(Serial3.read());
      }
      digitalWrite(37, gpsLedState ? HIGH : LOW);
      gpsLedState = !gpsLedState;
    } while(millis() - startTime < gpsTimeout);
  }

  wdt_reset();
  
  // послать запрос на все датчики температуры
  if(enableTemperatureSensors) {
    temperatureSensors.requestTemperatures();
  }
  
  // получаем температуру со всех датчиков температуры
  if(enableTemperatureSensors) {
    temp0 = temperatureSensors.getTempC(ds18b20_0);
    temp1 = temperatureSensors.getTempC(ds18b20_1);
    if(DEBUG) {
      Serial.print("[DS18B20] Temperature [Outer sensor, Inner battery sensor]: ");
      Serial.print(temp0);
      Serial.print("°C, ");
      Serial.print(temp1);
      Serial.println("°C");
    }
  }

  // получаем сырые данные с MPU и преобразовываем их в единицы СИ
  if(enableIMUsensor) {
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    axf = ((float)ax / 32768.0F * 16.0F) * 9.81F;
    ayf = ((float)ay / 32768.0F * 16.0F) * 9.81F;
    azf = ((float)az / 32768.0F * 16.0F) * 9.81F;
    gxf = ((float)gx / 32768.0F * 2000.0F);
    gyf = ((float)gy / 32768.0F * 2000.0F);
    gzf = ((float)gz / 32768.0F * 2000.0F);
    // фильтрация
    //axf = runningAverage(axf, 0);
    //ayf = runningAverage(ayf, 1);
    //azf = runningAverage(azf);
    /*gxf = runningAverage(gxf, 3);
    gyf = runningAverage(gyf, 4);
    gzf = runningAverage(gzf, 5);*/
    //
    pitch = atan2(ayf, sqrt(axf*axf + azf*azf)) * 180.0F / 3.14F;
    roll = atan2(-axf, sqrt(ayf*ayf + azf*azf)) * 180.0F / 3.14F;
    yaw = gzf;
    //yaw += (int)floor(gzf * (millis() - programStartTime) * 10e-3); // gzf * dt
    //yaw -= (int)floor(gyf * (millis() - programStartTime) * 10e-3);
    //if(yaw >= 360.0F || yaw <= -360.0F) yaw = 0.0F;
    if(DEBUG) {
      Serial.print("[MPU6050] Accelerometer [x, y, z], Gyroscope [x, y, z], Pitch, Roll, Yaw: ");
      Serial.print(axf); Serial.print(", ");
      Serial.print(ayf); Serial.print(", ");
      Serial.print(azf); Serial.print(", ");
      Serial.print(gxf); Serial.print(", ");
      Serial.print(gyf); Serial.print(", ");
      Serial.print(gzf); Serial.print(", ");
      Serial.print(pitch); Serial.print(", ");
      Serial.print(roll); Serial.print(", ");
      Serial.println(yaw);
    }
    //
  }

  // получаем данные с барометра
  if(enableBarometerSensor) {
    realTemperature = ms5611.readTemperature();
    realPressure = ms5611.readPressure();
    // фильтрация
    //realPressure = runningAverage(realPressure);
    //
    absoluteAltitude = ms5611.getAltitude(realPressure);
    relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);
    if(DEBUG) {
      Serial.print("[MS5611] Temperature [Inner barometer sensor], pressure, altitude [absolute, relative]: ");
      Serial.print(realTemperature); Serial.print("°C, ");
      Serial.print(realPressure); Serial.print(", ");
      Serial.print(absoluteAltitude); Serial.print(", ");
      Serial.println(relativeAltitude);
    }
  }

  wdt_reset();

  // светодиодная лента (анимация конца цикла)
  if(enableLedPanel) {
    for(uint8_t i = 0; i < 3; i++) {
      digitalWrite(31 + i, LOW);
      delay(62);
    }
  }
  programExecutingTime = millis() - programStartTime;
  loopsCounter += 1;
  if(DEBUG) {
    Serial.print("Takes: ");
    Serial.print(programExecutingTime / 1000.0F);
    Serial.println(" s");
  }
}
