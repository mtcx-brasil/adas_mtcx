/*
 * MTCX GPS - Sistema de Monitoramento GPS com Sensores MPU6050
 * 
 * Autor: Anderson Scaloni
 * 
 * Este projeto é open source e está disponível para uso livre por qualquer pessoa,
 * sem qualquer vínculo ou restrição. Você pode usar, modificar e distribuir este
 * código livremente para fins pessoais, educacionais ou comerciais.
 * 
 * Licença: MIT License
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 */

#include <Wire.h>
#include <U8g2lib.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// ===================== Display U8g2 =====================
// Construtor com buffer de página (usa menos RAM)
U8G2_SSD1306_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
bool displayAvailable = false;  // Controla se o display está funcionando

// ===================== Controle de Telas =====================
#define BUTTON_PIN 3        // Botão no pino D3
#define NUM_SCREENS 5       // 5 telas diferentes
uint8_t currentScreen = 0;  // Tela atual (0-3)
bool lastButtonState = HIGH;
bool buttonPressed = false;
unsigned long lastButtonTime = 0;
#define DEBOUNCE_DELAY 200  // 200ms debounce

// ===================== GPS Timeout =====================
unsigned long lastGPSUpdate = 0;
#define GPS_TIMEOUT 2000    // 2 segundos sem GPS = timeout (reduzido para detecção mais rápida)
bool gpsTimeout = false;

// ===================== GPS =====================
SoftwareSerial gpsPorts(2, 4);  // RX=D2 (Arduino), TX=D4
TinyGPS gps;

// Buffer circular para dados GPS (reduzido)
#define GPS_BUFFER_SIZE 16
char gpsBuffer[GPS_BUFFER_SIZE];
uint8_t gpsBufferIndex = 0;

// ===================== MPU-6050 =====================
const uint8_t MPU = 0x68;

// Dados crus
int16_t RawAccX, RawAccY, RawAccZ;
int16_t RawGyrX, RawGyrY, RawGyrZ;
int16_t RawTemp;

// Convertidos
float AccX_g, AccY_g, AccZ_g;
float GyrX_dps, GyrY_dps, GyrZ_dps;
float Temp_C;

// Dados calculados
float totalG_force = 0;      // Força G total
float lateralG_force = 0;    // Força G lateral (curvas)
float longitudinalG_force = 0; // Força G longitudinal (aceleração/frenagem)
float lastSpeedKmh = 0;      // Para calcular aceleração
float acceleration_ms2 = 0;   // Aceleração instantânea

// ===================== Calibragem MPU6050 =====================
// Offsets de calibragem
float AccX_offset = 0, AccY_offset = 0, AccZ_offset = 0;
float GyrX_offset = 0, GyrY_offset = 0, GyrZ_offset = 0;
bool mpuCalibrated = false;

// ===================== Filtro Kalman Simplificado =====================
struct KalmanFilter {
  float Q; // Ruído do processo
  float R; // Ruído da medição
  float P; // Erro estimado
  float K; // Ganho Kalman
  float X; // Valor filtrado
};

// Filtros Kalman para cada eixo
KalmanFilter kalman_AccX = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_AccY = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_AccZ = {0.01, 0.1, 1, 0, 0};
KalmanFilter kalman_GyrX = {0.01, 0.5, 1, 0, 0};
KalmanFilter kalman_GyrY = {0.01, 0.5, 1, 0, 0};
KalmanFilter kalman_GyrZ = {0.01, 0.5, 1, 0, 0};

// Bitmap removido para economizar memória RAM

// ===================== GPS Configuration =====================
void configureGPS() {
  gpsPorts.begin(9600);
  delay(500);
  gpsPorts.println("$PMTK220,200*2C"); // 5Hz
  delay(200);
}

// Processamento simples de dados GPS
bool processGPSData() {
  bool newData = false;
  
  while (gpsPorts.available()) {
    char c = gpsPorts.read();
    
    // Buffer circular simples
    gpsBuffer[gpsBufferIndex] = c;
    gpsBufferIndex = (gpsBufferIndex + 1) % GPS_BUFFER_SIZE;
    
    if (gps.encode(c)) {
      newData = true;
    }
  }
  
  return newData;
}

// ===================== Helpers =====================
float kalmanUpdate(KalmanFilter* kf, float measurement) {
  // Predição
  kf->P = kf->P + kf->Q;
  
  // Atualização
  kf->K = kf->P / (kf->P + kf->R);
  kf->X = kf->X + kf->K * (measurement - kf->X);
  kf->P = (1 - kf->K) * kf->P;
  
  return kf->X;
}

void calibrateMPU() {
  long sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  long sumGyrX = 0, sumGyrY = 0, sumGyrZ = 0;
  
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU, 14, true);

    sumAccX += (Wire.read() << 8) | Wire.read();
    sumAccY += (Wire.read() << 8) | Wire.read();
    sumAccZ += (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temp
    sumGyrX += (Wire.read() << 8) | Wire.read();
    sumGyrY += (Wire.read() << 8) | Wire.read();
    sumGyrZ += (Wire.read() << 8) | Wire.read();
    
    delay(5);
  }
  
  AccX_offset = (sumAccX / 100) / 2048.0f;
  AccY_offset = (sumAccY / 100) / 2048.0f;
  AccZ_offset = ((sumAccZ / 100) / 2048.0f) - 1.0f;
  GyrX_offset = (sumGyrX / 100) / 16.4f;
  GyrY_offset = (sumGyrY / 100) / 16.4f;
  GyrZ_offset = (sumGyrZ / 100) / 16.4f;
  
  mpuCalibrated = true;
}

void calculateDerivedData(float speedKmh) {
  // Calcular força G total (magnitude do vetor aceleração)
  totalG_force = sqrt(AccX_g * AccX_g + AccY_g * AccY_g + AccZ_g * AccZ_g);
  
  // Força G lateral (curvas) - eixo Y
  lateralG_force = abs(AccY_g);
  
  // Força G longitudinal (aceleração/frenagem) - eixo X
  longitudinalG_force = abs(AccX_g);
  
  // Calcular aceleração instantânea baseada na mudança de velocidade
  static unsigned long lastSpeedTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastSpeedTime > 500) { // A cada 500ms
    float deltaSpeed = speedKmh - lastSpeedKmh;
    float deltaTime = (currentTime - lastSpeedTime) / 1000.0f; // em segundos
    
    if (deltaTime > 0) {
      acceleration_ms2 = (deltaSpeed * 0.277778f) / deltaTime; // km/h para m/s²
    }
    
    lastSpeedKmh = speedKmh;
    lastSpeedTime = currentTime;
  }
}

void drawGPSIcon(bool hasSignal, bool isSearching) {
  // Ícone GPS no canto superior direito
  int x = 105;
  int y = 8;
  
  display.setFont(u8g2_font_4x6_tf);
  
  if (hasSignal) {
    // GPS OK - ícone sólido
    display.setCursor(x, y);
    display.print(F("GPS"));
  } else if (isSearching) {
    // GPS buscando - ícone piscando
    static bool blink = false;
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      blink = !blink;
      lastBlink = millis();
    }
    
    if (blink) {
      display.setCursor(x, y);
      display.print(F("GPS"));
    } else {
      display.setCursor(x, y);
      display.print(F("..."));
    }
  } else {
    // GPS timeout - X
    display.setCursor(x, y);
    display.print(F("GPS"));
    display.drawLine(x, y-6, x+15, y);
    display.drawLine(x, y, x+15, y-6);
  }
}

void drawTemperatureIcon(int x, int y) {
  // Desenhar termômetro simples
  // Bulbo do termômetro (círculo na base)
  display.drawDisc(x + 3, y + 15, 4);
  
  // Tubo do termômetro (retângulo vertical)
  display.drawBox(x + 1, y, 4, 12);
  
  // Marcações de temperatura (linhas pequenas)
  display.drawHLine(x + 6, y + 2, 3);
  display.drawHLine(x + 6, y + 5, 3);
  display.drawHLine(x + 6, y + 8, 3);
  display.drawHLine(x + 6, y + 11, 3);
}

void checkButton() {
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == LOW && millis() - lastButtonTime > DEBOUNCE_DELAY) {
      currentScreen = (currentScreen + 1) % NUM_SCREENS;
      lastButtonTime = millis();
    }
    lastButtonState = currentButtonState;
  }
}
bool mpuInit(uint8_t accelFS = 0b11, uint8_t gyroFS = 0b11) {
  // Verificar se o MPU está conectado
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU Error");
    return false;
  }

  // Acordar o MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // acorda
  Wire.endTransmission(true);

  // Configurar Gyro FS
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write((gyroFS & 0x03) << 3);
  Wire.endTransmission(true);

  // Configurar Accel FS
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write((accelFS & 0x03) << 3);
  Wire.endTransmission(true);

  return true;
}

void mpuReadOnce() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU, 14, true);

  RawAccX = (Wire.read() << 8) | Wire.read();
  RawAccY = (Wire.read() << 8) | Wire.read();
  RawAccZ = (Wire.read() << 8) | Wire.read();
  RawTemp = (Wire.read() << 8) | Wire.read();
  RawGyrX = (Wire.read() << 8) | Wire.read();
  RawGyrY = (Wire.read() << 8) | Wire.read();
  RawGyrZ = (Wire.read() << 8) | Wire.read();

  // Conversões (±16g e ±2000 °/s)
  float rawAccX_g = RawAccX / 2048.0f;
  float rawAccY_g = RawAccY / 2048.0f;
  float rawAccZ_g = RawAccZ / 2048.0f;
  
  float rawGyrX_dps = RawGyrX / 16.4f;
  float rawGyrY_dps = RawGyrY / 16.4f;
  float rawGyrZ_dps = RawGyrZ / 16.4f;

  // Aplicar calibragem se disponível
  if (mpuCalibrated) {
    rawAccX_g -= AccX_offset;
    rawAccY_g -= AccY_offset;
    rawAccZ_g -= AccZ_offset;
    
    rawGyrX_dps -= GyrX_offset;
    rawGyrY_dps -= GyrY_offset;
    rawGyrZ_dps -= GyrZ_offset;
  }
  
  // Aplicar filtro Kalman
  AccX_g = kalmanUpdate(&kalman_AccX, rawAccX_g);
  AccY_g = kalmanUpdate(&kalman_AccY, rawAccY_g);
  AccZ_g = kalmanUpdate(&kalman_AccZ, rawAccZ_g);
  
  GyrX_dps = kalmanUpdate(&kalman_GyrX, rawGyrX_dps);
  GyrY_dps = kalmanUpdate(&kalman_GyrY, rawGyrY_dps);
  GyrZ_dps = kalmanUpdate(&kalman_GyrZ, rawGyrZ_dps);

  // Temperatura (não precisa de filtro)
  Temp_C = RawTemp / 340.0f + 36.53f;
}

void drawSplash() {
  if (!displayAvailable) return;
  display.firstPage();
  do {
    display.setFont(u8g2_font_ncenB10_tr);
    display.setCursor(30, 35);
    display.print(F("MTCX GPS"));
  } while (display.nextPage());
}

// ===================== Telas do Sistema =====================
// Tela 1: Velocidade + Aceleração
void drawScreen1_Speed(float speedKmh, bool hasGPS, bool searchingGPS) {
  if (!displayAvailable) return;
  display.firstPage();
  do {
    // Indicador de tela (canto superior esquerdo)
    display.setFont(u8g2_font_4x6_tf);
    display.setCursor(0, 8);
    display.print(F("1/5"));
    
    // Velocidade gigante no centro
    display.setFont(u8g2_font_inb30_mn);
    display.setCursor(10, 45);
    display.print((int)speedKmh);
    
    // km/h menor embaixo
    display.setFont(u8g2_font_ncenB10_tr);
    display.setCursor(45, 60);
    display.print(F("km/h"));
    
    // Aceleração embaixo à esquerda
    display.setFont(u8g2_font_6x10_tf);
    display.setCursor(0, 64);
    display.print(F("A:"));
    display.print(acceleration_ms2, 1);
    
    // Ícone GPS
    drawGPSIcon(hasGPS, searchingGPS);
    
  } while (display.nextPage());
}

// Tela 2: Geolocalização com direção
void drawScreen2_Navigation(double latDeg, double lonDeg, float speedKmh, bool hasGPS, bool searchingGPS) {
  if (!displayAvailable) return;
  display.firstPage();
  do {
    display.setFont(u8g2_font_6x10_tf);
    
    // Coordenadas (ou mensagem se sem GPS)
    if (hasGPS) {
      display.setCursor(0, 10);
      display.print(F("LA:"));
      display.print(latDeg, 5);
      
      display.setCursor(0, 22);
      display.print(F("LO:"));
      display.print(lonDeg, 5);
    } else {
      display.setCursor(0, 10);
      display.print(F("LA: ---.-----"));
      display.setCursor(0, 22);
      display.print(F("LO: ---.-----"));
    }
    
    // Velocidade e direção baseada no giroscópio
    display.setCursor(0, 40);
    display.print(F("Vel: "));
    display.print((int)speedKmh);
    display.print(F(" km/h"));
    
    // Direção baseada no giroscópio Z (rotação)
    display.setCursor(0, 52);
    display.print(F("Dir: "));
    if (GyrZ_dps > 10) {
      display.print(F("Direita"));
    } else if (GyrZ_dps < -10) {
      display.print(F("Esquerda"));
    } else {
      display.print(F("Reto"));
    }
    
    // Ícone GPS
    drawGPSIcon(hasGPS, searchingGPS);
    
    // Indicador de tela
    display.setFont(u8g2_font_4x6_tf);
    display.setCursor(0, 64);
    display.print(F("2/5"));
    
  } while (display.nextPage());
}

// Tela 3: Inclinômetro estilo carro
void drawScreen3_Inclinometer() {
  if (!displayAvailable) return;
  display.firstPage();
  do {
    // Centro da tela
    int centerX = 64;
    int centerY = 32;
    
    // Calcular inclinação baseada no acelerômetro
    int tiltX = (int)(AccX_g * 20); // Escala para pixels
    int tiltY = (int)(AccY_g * 15);
    
    // Limitar valores
    tiltX = constrain(tiltX, -30, 30);
    tiltY = constrain(tiltY, -20, 20);
    
    // Desenhar círculo base (horizonte)
    display.drawCircle(centerX, centerY, 25);
    
    // Desenhar "bolinha" do inclinômetro
    display.drawDisc(centerX + tiltX, centerY - tiltY, 3);
    
    // Linhas de referência (cruz)
    display.drawHLine(centerX - 30, centerY, 60);
    display.drawVLine(centerX, centerY - 25, 50);
    
    // Forças G
    display.setFont(u8g2_font_4x6_tf);
    display.setCursor(90, 8);
    display.print(F("G:"));
    display.print(totalG_force, 1);
    
    display.setCursor(90, 16);
    display.print(F("Lat:"));
    display.print(lateralG_force, 1);
    
    display.setCursor(90, 24);
    display.print(F("Lon:"));
    display.print(longitudinalG_force, 1);
    
    // Valores acelerômetro (menores)
    display.setCursor(0, 8);
    display.print(F("X:"));
    display.print(AccX_g, 1);
    
    display.setCursor(0, 16);
    display.print(F("Y:"));
    display.print(AccY_g, 1);
    
    display.setCursor(0, 24);
    display.print(F("Z:"));
    display.print(AccZ_g, 1);
    
    // Indicador de tela
    display.setCursor(0, 64);
    display.print(F("3/5"));
    
  } while (display.nextPage());
}

// Tela 4: Dados completos Giroscópio + Acelerômetro
void drawScreen4_SensorsData(bool hasGPS, bool searchingGPS) {
  if (!displayAvailable) return;
  display.firstPage();
  do {
    display.setFont(u8g2_font_6x10_tf);
    
    // Título
    display.setCursor(25, 10);
    display.print(F("SENSORES"));
    
    // Acelerômetro
    display.setCursor(0, 25);
    display.print(F("Acelerometro (g):"));
    display.setCursor(0, 35);
    display.print(F("X:"));
    display.print(AccX_g, 2);
    display.print(F(" Y:"));
    display.print(AccY_g, 2);
    display.print(F(" Z:"));
    display.print(AccZ_g, 2);
    
    // Giroscópio
    display.setCursor(0, 48);
    display.print(F("Giroscopio (dps):"));
    display.setCursor(0, 58);
    display.print(F("X:"));
    display.print((int)GyrX_dps);
    display.print(F(" Y:"));
    display.print((int)GyrY_dps);
    display.print(F(" Z:"));
    display.print((int)GyrZ_dps);
    
    // Ícone GPS
    drawGPSIcon(hasGPS, searchingGPS);
    
    // Indicador de tela
    display.setFont(u8g2_font_4x6_tf);
    display.setCursor(0, 64);
    display.print(F("4/5"));
    
  } while (display.nextPage());
}

// Tela 5: Temperatura + Altitude
void drawScreen5_Temperature(bool hasGPS, bool searchingGPS, float altM) {
  if (!displayAvailable) return;
  display.firstPage();
  do {
    // Indicador de tela (canto superior esquerdo)
    display.setFont(u8g2_font_4x6_tf);
    display.setCursor(0, 8);
    display.print(F("5/5"));
    
    // Ícone de termômetro à esquerda
    drawTemperatureIcon(15, 25);
    
    // Temperatura gigante centralizada
    display.setFont(u8g2_font_inb30_mn);
    display.setCursor(45, 45);
    display.print((int)Temp_C);
    
    // Altitude embaixo à esquerda
    display.setFont(u8g2_font_6x10_tf);
    display.setCursor(0, 64);
    display.print(F("Alt:"));
    display.print((int)altM);
    display.print(F("m"));
    
    // Ícone GPS
    drawGPSIcon(hasGPS, searchingGPS);
    
  } while (display.nextPage());
}

// drawNoFix removida - não é mais usada

// Função principal de desenho - chama a tela apropriada
void drawData(double latDeg, double lonDeg, float speedKmh, float altM, bool hasGPS, bool searchingGPS) {
  if (!displayAvailable) return;
  
  // Calcular dados derivados
  calculateDerivedData(speedKmh);
  
  switch (currentScreen) {
    case 0:
      drawScreen1_Speed(speedKmh, hasGPS, searchingGPS);
      break;
    case 1:
      drawScreen2_Navigation(latDeg, lonDeg, speedKmh, hasGPS, searchingGPS);
      break;
    case 2:
      drawScreen3_Inclinometer();
      break;
    case 3:
      drawScreen4_SensorsData(hasGPS, searchingGPS);
      break;
    case 4:
      drawScreen5_Temperature(hasGPS, searchingGPS, altM);
      break;
  }
}

// ===================== Setup/Loop =====================
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Aguarda Serial estar pronto
  Serial.println("Init...");
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(100000);
  delay(20);
  
  display.begin();
  displayAvailable = true;
  drawSplash();
  delay(200);
  
  configureGPS();
  
  if (mpuInit(0b11, 0b11)) {
    delay(300);
    calibrateMPU();
  }
  
  Serial.println("Ready!");
}

void loop() {
  // Verificar botão
  checkButton();
  
  // Processar dados GPS com função otimizada
  bool gpsDataReceived = processGPSData();

  // Ler dados do MPU6050 (apenas se não há dados GPS pendentes para máxima prioridade)
  if (!gpsPorts.available()) {
    mpuReadOnce();
  }

  // Obter posição GPS
  long rawLat, rawLon;
  unsigned long age;
  gps.get_position(&rawLat, &rawLon, &age);

  double latDeg = (rawLat == TinyGPS::GPS_INVALID_ANGLE) ? NAN : (rawLat / 100000.0);
  double lonDeg = (rawLon == TinyGPS::GPS_INVALID_ANGLE) ? NAN : (rawLon / 100000.0);

  float speedKmh = gps.f_speed_kmph();
  float altM = gps.f_altitude();
  
  // Verificar se dados GPS são válidos e recentes
  bool validPosition = !isnan(latDeg) && !isnan(lonDeg);
  bool recentData = (age < 5000); // Dados com menos de 5 segundos
  
  // Atualizar timestamp se temos dados GPS válidos e recentes
  if (validPosition && recentData) {
    lastGPSUpdate = millis();
    gpsTimeout = false;
  } else if (millis() - lastGPSUpdate > GPS_TIMEOUT) {
    gpsTimeout = true;
  }

  // Determinar status do GPS de forma mais estável
  bool hasGPS = validPosition && recentData && !gpsTimeout;
  bool searchingGPS = !hasGPS && !gpsTimeout;
  
  // Atualizar display a cada 100ms para acompanhar GPS 10Hz
  static uint32_t lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 100) {
    lastDisplayUpdate = millis();
    
    // Sempre mostrar as telas principais, apenas com ícone GPS
    if (isnan(speedKmh)) speedKmh = 0.0f;
    if (isnan(altM) || altM > 100000.0f) altM = 0.0f;
    
    // Se não tem GPS, usar valores padrão para coordenadas
    double displayLat = hasGPS ? latDeg : 0.0;
    double displayLon = hasGPS ? lonDeg : 0.0;
    
    drawData(displayLat, displayLon, speedKmh, altM, hasGPS, searchingGPS);
  }

  static uint32_t lastLog = 0;
  if (millis() - lastLog > 2000) {
    lastLog = millis();
    
    // Debug detalhado para identificar problema
    Serial.print("GPS: ");
    if (!isnan(latDeg) && !isnan(lonDeg)) {
      Serial.print(latDeg, 4);
      Serial.print(",");
      Serial.print(lonDeg, 4);
      Serial.print(" ");
      Serial.print((int)speedKmh);
      Serial.print("km/h");
    } else {
      Serial.print("INVALID");
    }
    Serial.print(" | hasGPS:");
    Serial.print(hasGPS ? "YES" : "NO");
    Serial.print(" valid:");
    Serial.print(validPosition ? "YES" : "NO");
    Serial.print(" recent:");
    Serial.print(recentData ? "YES" : "NO");
    Serial.print(" timeout:");
    Serial.print(gpsTimeout ? "YES" : "NO");
    Serial.print(" age:");
    Serial.println(age);
  }

  delay(5);   // Otimizado para GPS 10Hz - máxima responsividade
}
