# MTCX MTCX ADAS - Sistema de Monitoramento GPS com Sensores

![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat&logo=Arduino&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B&logoColor=white)

## 📖 Sobre o Projeto

O **MTCX ADAS** é um sistema completo de monitoramento GPS com sensores inerciais desenvolvido por **Anderson Scaloni**. Este projeto combina localização GPS precisa com dados de acelerômetro e giroscópio para criar um dispositivo de telemetria automotiva ou de atividades esportivas.

### ✨ Características Principais

- **5 Telas Interativas**: Navegue entre diferentes visualizações usando um botão
- **Monitoramento GPS**: Localização, velocidade e altitude em tempo real
- **Sensores Inerciais**: Acelerômetro e giroscópio MPU6050 com filtro Kalman
- **Análise de Força G**: Medição de forças laterais e longitudinais
- **Inclinômetro Visual**: Interface gráfica estilo automotivo
- **Temperatura**: Monitoramento térmico do sensor MPU6050
- **Interface Intuitiva**: Display OLED com ícones e indicadores visuais

## 🛠️ Hardware Necessário

### Componentes Obrigatórios

| Componente | Especificação | Quantidade | Preço Estimado (BR) |
|------------|---------------|------------|---------------------|
| **Arduino Uno/Nano** | ATmega328P | 1x | R$ 25-45 |
| **Display OLED** | SSD1306 128x64 I2C | 1x | R$ 15-25 |
| **Módulo GPS** | Compatível NMEA (ex: NEO-6M) | 1x | R$ 20-35 |
| **Sensor MPU6050** | Acelerômetro + Giroscópio I2C | 1x | R$ 8-15 |
| **Botão Push** | Botão táctil 6x6mm | 1x | R$ 1-3 |
| **Resistor Pull-up** | 10kΩ | 1x | R$ 0,50 |

### Componentes Opcionais

| Componente | Finalidade | Preço Estimado |
|------------|------------|----------------|
| **Case/Caixa** | Proteção do circuito | R$ 10-20 |
| **Protoboard** | Montagem sem solda | R$ 8-15 |
| **Jumpers** | Conexões | R$ 5-10 |
| **Fonte Externa** | Alimentação independente | R$ 15-25 |
 
**💰 Custo Total Estimado: R$ 70-160**

## 🔌 Diagrama de Conexões

### Pinagem Arduino Uno/Nano

```
┌─────────────────────────────────────────────────────────────┐
│                        ARDUINO UNO                          │
│                                                             │
│  Digital Pins              Analog Pins        Power         │
│  ┌─────────────┐          ┌──────────┐     ┌─────────┐      │
│  │ D13 │ D12   │          │ A5 │ A4  │     │ 5V │GND │      │
│  │ D11 │ D10   │          │ A3 │ A2  │     │ 3V3│VIN │      │
│  │ D9  │ D8    │          │ A1 │ A0  │     └─────────┘      │
│  │ D7  │ D6    │          └──────────┘                      │
│  │ D5  │ D4 ◄──┼── GPS TX                                   │
│  │ D3 ◄┼─BOTÃO │                                            │
│  │ D2 ◄┼─GPS RX│                                            │
│  │ D1  │ D0    │                                            │
│  └─────────────┘                                            │
│                                                             │
│              I2C Bus                                        │
│              ┌─────────────────┐                            │
│              │ SDA ◄─── A4     │                            │
│              │ SCL ◄─── A5     │                            │
│              └─────────────────┘                            │
└─────────────────────────────────────────────────────────────┘

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│   DISPLAY OLED   │  │    MPU6050       │  │   MÓDULO GPS     │
│   SSD1306        │  │                  │  │                  │
│                  │  │                  │  │                  │
│ VCC ──── 5V      │  │ VCC ──── 5V      │  │ VCC ──── 5V      │
│ GND ──── GND     │  │ GND ──── GND     │  │ GND ──── GND     │
│ SDA ──── A4      │  │ SDA ──── A4      │  │ TX  ──── D2      │
│ SCL ──── A5      │  │ SCL ──── A5      │  │ RX  ──── D4      │
└──────────────────┘  └──────────────────┘  └──────────────────┘

┌──────────────────┐
│      BOTÃO       │
│                  │
│ Terminal 1 ── D3 │
│ Terminal 2 ── GND│
│                  │
│ (Resistor pull-up│
│  interno ativo)  │
└──────────────────┘
```

### Detalhes das Conexões

#### 🖥️ Display OLED SSD1306 (I2C)
- **VCC** → 5V (ou 3.3V)
- **GND** → GND
- **SDA** → A4 (Arduino Uno) / A4 (Arduino Nano)
- **SCL** → A5 (Arduino Uno) / A5 (Arduino Nano)

#### 🧭 MPU6050 (I2C)
- **VCC** → 5V (ou 3.3V)
- **GND** → GND
- **SDA** → A4 (compartilhado com display)
- **SCL** → A5 (compartilhado com display)

#### 📡 Módulo GPS
- **VCC** → 5V
- **GND** → GND
- **TX** → D2 (RX do Arduino)
- **RX** → D4 (TX do Arduino)

#### 🔘 Botão de Navegação
- **Terminal 1** → D3
- **Terminal 2** → GND
- **Pull-up interno** ativado no código

## 📚 Bibliotecas Necessárias

Instale as seguintes bibliotecas através do Arduino IDE (Sketch → Include Library → Manage Libraries):

```cpp
// Bibliotecas obrigatórias
#include <Wire.h>           // I2C (já incluída no Arduino IDE)
#include <U8g2lib.h>        // Display OLED - versão 2.34.x ou superior
#include <SoftwareSerial.h> // Comunicação serial (já incluída)
#include <TinyGPS.h>        // Processamento GPS - versão 13.x ou superior
```

### 📦 Como Instalar as Bibliotecas

1. **U8g2lib** (para display OLED):
   - Arduino IDE → Tools → Manage Libraries
   - Busque por "U8g2"
   - Instale "U8g2" by oliver

2. **TinyGPS** (para processamento GPS):
   - Arduino IDE → Tools → Manage Libraries
   - Busque por "TinyGPS"
   - Instale "TinyGPS" by Mikal Hart

## 🚀 Como Usar

### 1. Montagem do Hardware
- Conecte todos os componentes conforme o diagrama de pinagem
- Verifique todas as conexões antes de energizar
- Certifique-se de que os dispositivos I2C não tenham conflito de endereços

### 2. Instalação e Configuração do Arduino IDE

#### 📥 Download e Instalação
1. **Baixe o Arduino IDE**:
   - Acesse: [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)
   - Escolha a versão para seu sistema operacional (Windows, macOS, Linux)
   - **Versão recomendada**: Arduino IDE 2.x (mais recente)
   - Instale seguindo as instruções do seu sistema operacional

#### ⚙️ Configuração Inicial
2. **Configure a Placa Arduino**:
   - Abra o Arduino IDE
   - Vá em **Tools** → **Board** → **Arduino AVR Boards**
   - Selecione sua placa:
     - **Arduino Uno** (se usar Arduino Uno)
     - **Arduino Nano** (se usar Arduino Nano)

3. **Configure a Porta Serial**:
   - Conecte seu Arduino ao computador via cabo USB
   - Vá em **Tools** → **Port**
   - Selecione a porta onde seu Arduino está conectado:
     - **Windows**: COM3, COM4, COM5, etc.
     - **macOS**: /dev/cu.usbmodem ou /dev/cu.usbserial
     - **Linux**: /dev/ttyUSB0, /dev/ttyACM0, etc.

#### 📚 Instalação das Bibliotecas
4. **Instale as Bibliotecas Necessárias**:
   
   **U8g2lib (Display OLED):**
   - No Arduino IDE: **Tools** → **Manage Libraries**
   - Digite "U8g2" na busca
   - Encontre "U8g2 by oliver"
   - Clique em **Install**
   - Aguarde a instalação completa
   
   **TinyGPS (Processamento GPS):**
   - No Arduino IDE: **Tools** → **Manage Libraries**
   - Digite "TinyGPS" na busca
   - Encontre "TinyGPS by Mikal Hart"
   - Clique em **Install**
   - Aguarde a instalação completa

### 3. Compilação e Upload do Programa

#### 📁 Preparação dos Arquivos
1. **Baixe o Código**:
   ```bash
   # Opção 1: Clone via Git
   git clone https://github.com/mtcx-brasil/adas_mtcx.git
   
   # Opção 2: Download direto do GitHub
   # Acesse: https://github.com/mtcx-brasil/adas_mtcx
   # Clique em "Code" → "Download ZIP"
   # Extraia o arquivo ZIP
   ```

2. **Abra o Projeto**:
   - Navegue até a pasta do projeto
   - **Duplo-clique** no arquivo `arduino.ino`
   - O Arduino IDE abrirá automaticamente com o projeto

#### 🔧 Compilação e Verificação
3. **Compile o Código**:
   - No Arduino IDE, clique no ícone **✓ Verify** (ou Ctrl+R)
   - Aguarde a compilação terminar
   - **Se houver erros**:
     - Verifique se todas as bibliotecas estão instaladas
     - Verifique se a placa está selecionada corretamente
     - Verifique a sintaxe do código

4. **Informações de Compilação**:
   ```
   Sketch uses XXXXX bytes (XX%) of program storage space.
   Global variables use XXXX bytes (XX%) of dynamic memory.
   ```
   - **Programa**: Deve usar menos de 32KB (Arduino Uno)
   - **Memória RAM**: Deve usar menos de 2KB (Arduino Uno)

#### 📤 Upload para o Arduino
5. **Faça o Upload**:
   - Certifique-se de que o Arduino está conectado
   - Verifique se a porta está selecionada corretamente
   - Clique no ícone **→ Upload** (ou Ctrl+U)
   - **Aguarde o processo**:
     ```
     Compiling sketch...
     Uploading...
     Done uploading.
     ```

6. **Verificação do Upload**:
   - O LED do Arduino piscará durante o upload
   - Após completar, o programa iniciará automaticamente
   - Você verá a tela de splash "MTCX GPS" no display

#### 🔍 Monitoramento e Debug
7. **Monitor Serial** (Opcional):
   - Clique em **Tools** → **Serial Monitor** (ou Ctrl+Shift+M)
   - Configure para **9600 baud**
   - Você verá mensagens de debug do GPS e sensores:
     ```
     Init...
     Ready!
     GPS: INVALID | hasGPS:NO valid:NO recent:NO timeout:NO age:4294967295
     GPS: 25.2634,-53.8251 45km/h | hasGPS:YES valid:YES recent:YES timeout:NO age:150
     ```

#### ⚠️ Solução de Problemas de Compilação

**Erro: "Library not found"**
- Reinstale as bibliotecas U8g2lib e TinyGPS
- Reinicie o Arduino IDE

**Erro: "Board not found"**
- Verifique se selecionou a placa correta
- Reinstale os drivers do Arduino

**Erro: "Port not available"**
- Verifique o cabo USB
- Tente outra porta USB
- Reinstale os drivers CH340/CP2102 (Arduino clones)

**Erro: "Sketch too big"**
- Use Arduino Uno (mais memória que o Nano)
- Otimize o código removendo funcionalidades desnecessárias

#### 📊 Especificações de Compilação
- **Plataforma**: Arduino AVR (ATmega328P)
- **Frequência**: 16MHz
- **Bootloader**: Optiboot
- **Memória Flash**: ~28KB usados de 32KB
- **SRAM**: ~1.5KB usados de 2KB

### 4. Operação do Sistema
- **Ligar**: O sistema mostra uma tela de splash "MTCX"
- **Navegação**: Pressione o botão para alternar entre as 5 telas
- **GPS**: Aguarde alguns minutos para aquisição do sinal GPS
- **Calibração**: O MPU6050 é calibrado automaticamente na inicialização

## 📱 Interface do Sistema

### Tela 1: Velocidade Principal
- **Velocidade**: Fonte grande no centro (km/h)
- **Aceleração**: Valor instantâneo (m/s²)
- **Status GPS**: Ícone no canto superior direito

### Tela 2: Navegação GPS
- **Coordenadas**: Latitude e Longitude (5 casas decimais)
- **Velocidade**: Valor atual
- **Direção**: Baseada no giroscópio (Esquerda/Direita/Reto)

### Tela 3: Inclinômetro
- **Visualização**: Círculo com "bolinha" mostrando inclinação
- **Forças G**: Total, Lateral e Longitudinal
- **Valores XYZ**: Dados brutos do acelerômetro

### Tela 4: Dados dos Sensores
- **Acelerômetro**: Valores X, Y, Z em g
- **Giroscópio**: Valores X, Y, Z em graus/segundo
- **Precisão**: 2 casas decimais

### Tela 5: Temperatura e Altitude
- **Temperatura**: Do sensor MPU6050 em °C
- **Altitude**: Do GPS em metros
- **Ícone**: Termômetro gráfico

## 🔧 Configurações Avançadas

### Calibração do MPU6050
O sistema calibra automaticamente o MPU6050 na inicialização:
- Mantenha o dispositivo **imóvel** durante os primeiros 5 segundos
- A calibração compensa offsets de fábrica
- 100 amostras são coletadas para precisão

### Configuração GPS
- **Taxa de atualização**: 5Hz (200ms)
- **Timeout**: 2 segundos sem dados
- **Protocolo**: NMEA padrão
- **Baudrate**: 9600 bps

### Filtro Kalman
O sistema usa filtros Kalman individuais para cada eixo:
- **Ruído do processo (Q)**: 0.01
- **Ruído da medição (R)**: 0.1 (acelerômetro), 0.5 (giroscópio)
- **Reduz**: Vibrações e ruídos elétricos

## 🔍 Solução de Problemas

### Display não funciona
- ✅ Verifique conexões I2C (SDA/SCL)
- ✅ Teste diferentes endereços I2C (0x3C, 0x3D)
- ✅ Use o arquivo `test_display.ino` para diagnóstico
- ✅ Verifique alimentação (3.3V ou 5V)

### GPS não adquire sinal
- ✅ Use em área externa com visão do céu
- ✅ Aguarde 2-5 minutos para cold start
- ✅ Verifique conexões TX/RX (podem estar invertidas)
- ✅ Monitore Serial para mensagens NMEA

### MPU6050 não responde
- ✅ Verifique endereço I2C (0x68 padrão)
- ✅ Teste com scanner I2C
- ✅ Verifique alimentação estável
- ✅ Mantenha imóvel durante calibração

### Botão não responde
- ✅ Verifique conexão no pino D3
- ✅ Teste continuidade do botão
- ✅ Pull-up interno está ativado no código

## 📊 Especificações Técnicas

### Performance
- **Taxa de atualização**: 10Hz (100ms)
- **Precisão GPS**: ±3-5 metros
- **Range acelerômetro**: ±16g
- **Range giroscópio**: ±2000°/s
- **Resolução temperatura**: 0.1°C

### Consumo de Energia
- **Arduino Uno**: ~20mA
- **Display OLED**: ~15mA
- **MPU6050**: ~3mA
- **GPS**: ~25mA
- **Total estimado**: ~65mA @ 5V

### Limitações
- GPS funciona apenas em áreas externas
- Precisão afetada por vibração excessiva
- Temperatura do MPU6050 (não ambiente)
- Altitude GPS tem precisão limitada

## 🤝 Contribuições

Este é um projeto **open source** sob licença MIT. Contribuições são bem-vindas!

### Como Contribuir
1. Fork o projeto
2. Crie uma branch para sua feature (`git checkout -b feature/MinhaFeature`)
3. Commit suas mudanças (`git commit -m 'Adiciona MinhaFeature'`)
4. Push para a branch (`git push origin feature/MinhaFeature`)
5. Abra um Pull Request

### Ideias para Melhorias
- [ ] Suporte a cartão SD para logging
- [ ] Interface Bluetooth para smartphone
- [ ] Alarmes configuráveis
- [ ] Modo economia de energia
- [ ] Suporte a múltiplos tipos de GPS
- [ ] Interface web via WiFi

## 📄 Licença

```
MIT License

Copyright (c) 2024 Anderson Scaloni

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 📞 Suporte

- **Autor**: Anderson Scaloni
- **Issues**: Use o sistema de issues do GitHub para reportar bugs
- **Documentação**: Este README contém todas as informações necessárias

---

**⭐ Se este projeto foi útil para você, considere dar uma estrela no GitHub!**

**🚗 Desenvolvido com ❤️ para entusiastas de eletrônica e automobilismo**
