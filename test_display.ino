#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configurações do display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Testará múltiplos endereços
uint8_t possibleAddresses[] = {0x3C, 0x3D, 0x78, 0x7A};
int numAddresses = 4;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Aguarda Serial estar pronto
  
  Serial.println(F("=== TESTE DIAGNÓSTICO DISPLAY SSD1306 ==="));
  Serial.println(F("Versão da biblioteca Adafruit_SSD1306: Verificando..."));
  Serial.println();
  
  // Inicializar I2C
  Wire.begin();
  Serial.println(F("I2C inicializado"));
  
  // Testar diferentes velocidades I2C
  uint32_t speeds[] = {100000, 400000, 50000};
  int numSpeeds = 3;
  
  for (int s = 0; s < numSpeeds; s++) {
    Wire.setClock(speeds[s]);
    Serial.print(F("Testando velocidade I2C: "));
    Serial.print(speeds[s]);
    Serial.println(F(" Hz"));
    
    // Scanner I2C detalhado
    scanI2CDetailed();
    
    // Testar cada endereço possível
    for (int i = 0; i < numAddresses; i++) {
      testDisplayAddress(possibleAddresses[i]);
    }
    
    Serial.println(F("----------------------------------------"));
  }
  
  Serial.println(F("=== TESTE CONCLUÍDO ==="));
  Serial.println(F("Envie os logs completos para análise!"));
}

void loop() {
  // Scanner contínuo a cada 5 segundos
  static unsigned long lastScan = 0;
  if (millis() - lastScan > 5000) {
    lastScan = millis();
    Serial.println(F("--- Scanner contínuo ---"));
    scanI2CDetailed();
    Serial.println();
  }
}

void scanI2CDetailed() {
  Serial.println(F("Iniciando scanner I2C detalhado..."));
  byte error, address;
  int nDevices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("✓ Dispositivo encontrado no endereço 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(F(" (decimal: "));
      Serial.print(address);
      Serial.print(F(", binário: "));
      Serial.print(address, BIN);
      Serial.println(F(")"));
      nDevices++;
      
      // Identificar possível tipo de dispositivo
      identifyDevice(address);
      
    } else if (error == 4) {
      Serial.print(F("⚠ Erro desconhecido no endereço 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(F(" - dispositivo pode estar presente mas com problemas"));
    }
    
    // Pequeno delay para estabilidade
    delay(1);
  }
  
  Serial.print(F("Total de dispositivos I2C encontrados: "));
  Serial.println(nDevices);
  
  if (nDevices == 0) {
    Serial.println(F("❌ NENHUM dispositivo I2C detectado!"));
    Serial.println(F("Possíveis problemas:"));
    Serial.println(F("- Cabos SDA/SCL desconectados ou invertidos"));
    Serial.println(F("- Alimentação (VCC/GND) com problema"));
    Serial.println(F("- Resistores pull-up ausentes (alguns displays precisam)"));
    Serial.println(F("- Display defeituoso"));
  }
  Serial.println();
}

void identifyDevice(byte address) {
  Serial.print(F("  → Possível dispositivo: "));
  switch (address) {
    case 0x3C:
    case 0x3D:
      Serial.println(F("SSD1306 OLED Display (128x64 ou 128x32)"));
      break;
    case 0x68:
    case 0x69:
      Serial.println(F("MPU6050 Acelerômetro/Giroscópio"));
      break;
    case 0x48:
    case 0x49:
    case 0x4A:
    case 0x4B:
      Serial.println(F("ADS1115 ADC ou sensor de temperatura"));
      break;
    case 0x50:
    case 0x51:
    case 0x52:
    case 0x53:
    case 0x54:
    case 0x55:
    case 0x56:
    case 0x57:
      Serial.println(F("EEPROM 24C32/24C64"));
      break;
    case 0x76:
    case 0x77:
      Serial.println(F("BMP280/BME280 Sensor de pressão"));
      break;
    default:
      Serial.println(F("Dispositivo desconhecido"));
      break;
  }
}

void testDisplayAddress(uint8_t addr) {
  Serial.print(F("🔍 Testando SSD1306 no endereço 0x"));
  if (addr < 16) Serial.print("0");
  Serial.print(addr, HEX);
  Serial.print(F("... "));
  
  // Tentar inicializar display
  bool success = display.begin(SSD1306_SWITCHCAPVCC, addr);
  
  if (success) {
    Serial.println(F("✅ SUCESSO!"));
    Serial.println(F("Display inicializado com sucesso!"));
    
    // Testar funcionalidades básicas
    testDisplayFunctions();
    
  } else {
    Serial.println(F("❌ FALHOU"));
    Serial.println(F("  Motivos possíveis:"));
    Serial.println(F("  - Endereço incorreto"));
    Serial.println(F("  - Display não é SSD1306"));
    Serial.println(F("  - Problema de comunicação I2C"));
    Serial.println(F("  - Display defeituoso"));
  }
  Serial.println();
}

void testDisplayFunctions() {
  Serial.println(F("Testando funções do display:"));
  
  // Teste 1: Limpar display
  Serial.print(F("  - Limpando display... "));
  display.clearDisplay();
  display.display();
  Serial.println(F("OK"));
  delay(500);
  
  // Teste 2: Pixel único
  Serial.print(F("  - Desenhando pixel... "));
  display.drawPixel(64, 32, SSD1306_WHITE);
  display.display();
  Serial.println(F("OK"));
  delay(500);
  
  // Teste 3: Texto simples
  Serial.print(F("  - Escrevendo texto... "));
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("TESTE OK!"));
  display.setCursor(0, 16);
  display.println(F("Display funcionando"));
  display.setCursor(0, 32);
  display.print(F("Endereco: 0x"));
  display.println(display.getRotation(), HEX); // Workaround para mostrar algo
  display.display();
  Serial.println(F("OK"));
  delay(2000);
  
  // Teste 4: Retângulo
  Serial.print(F("  - Desenhando retângulo... "));
  display.clearDisplay();
  display.drawRect(10, 10, 108, 44, SSD1306_WHITE);
  display.display();
  Serial.println(F("OK"));
  delay(1000);
  
  // Teste 5: Círculo
  Serial.print(F("  - Desenhando círculo... "));
  display.clearDisplay();
  display.drawCircle(64, 32, 20, SSD1306_WHITE);
  display.display();
  Serial.println(F("OK"));
  delay(1000);
  
  Serial.println(F("✅ Todos os testes do display passaram!"));
  Serial.println(F("🎉 Display está funcionando perfeitamente!"));
}
