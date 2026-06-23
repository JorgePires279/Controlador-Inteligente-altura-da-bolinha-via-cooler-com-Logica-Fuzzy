# 🏐 Levitação de Bolinha — Controlador Fuzzy

> Sistema embarcado de levitação pneumática com Inteligência Artificial rodando em ESP32 + FreeRTOS. A bolinha flutua dentro de um tubo PVC e é mantida em uma altura-alvo definida pelo operador — sem intervenção humana.

**Autor:** Jorge Luiz Madeira Pires  
**Disciplina:** Inteligência Artificial — 2026  
**Técnica de IA:** Lógica Fuzzy Incremental (Método Mamdani — Velocity Form)

---

## 📹 Vídeos do Sistema em Funcionamento

> ⚠️ O hardware físico é necessário para executar o projeto. Acesse os vídeos demonstrativos do sistema funcionando no link abaixo:

**🎬 [Clique aqui para assistir os vídeos](https://drive.google.com/drive/folders/10K1VTOatXDoch2NS3IJtij4NFJMhp7fN)**

---

## 🏗️ Arquitetura do Sistema

```
┌─────────────────────────────────────────────────────┐
│                    ESP32 (Dual-Core 240MHz)          │
│                                                      │
│  Core 0                    Core 1                    │
│  ┌──────────────┐          ┌──────────────────────┐  │
│  │ Task Sensor  │          │   Task Controle      │  │
│  │   (60ms)     │─filaSensor─►  Fuzzy Mamdani     │  │
│  │  HC-SR04 ISR │          │   (60ms)             │  │
│  └──────────────┘          └──────────┬───────────┘  │
│  ┌──────────────┐                     │mutexESC       │
│  │ Task Botões  │                     ▼               │
│  │   (20ms)     │          ┌──────────────────────┐  │
│  │  5 botões    │          │   Task Serial/LCD    │  │
│  └──────────────┘          │      (400ms)         │  │
│                            └──────────────────────┘  │
└─────────────────────────────────────────────────────┘
         │                          │
    Motor ESC (PWM)           LCD I²C 16x2
    Pin 18 / 50Hz
```

---

## 🔧 Hardware Necessário

| Componente | Quantidade | Observação |
|---|---|---|
| **ESP32 DevKit** | 1 | Qualquer variante com 38 pinos |
| **Motor Brushless** | 1 | Ex.: 2212 ou similar (drone) |
| **ESC (Electronic Speed Controller)** | 1 | Compatível com o motor, ex.: 30A |
| **HC-SR04 (Sensor Ultrassônico)** | 1 | Instalado no topo do tubo |
| **LCD 16x2 com módulo I²C** | 1 | Endereço padrão 0x27 |
| **Tubo PVC** | 1 | Aprox. 40cm comprimento |
| **Potenciômetro 10kΩ** | 1 | Para calibração |
| **LED** | 1 | Indicador de altura-alvo atingida |
| **Botões de pressão** | 5 | ARM, CAL, UP, DOWN, TURB |
| **Fonte 5V / Bateria LiPo** | 1 | Para alimentar o sistema |

---

## 📌 Mapeamento de Pinos (ESP32)

| Pino GPIO | Função |
|---|---|
| **34** | Potenciômetro (calibração) |
| **18** | Sinal PWM para o ESC |
| **25** | HC-SR04 — TRIG |
| **27** | HC-SR04 — ECHO |
| **4** | Botão ARM / DESARMAR |
| **0** | Botão CALIBRAÇÃO (avança etapa) |
| **32** | Botão + (aumenta setpoint 5cm) |
| **33** | Botão − (diminui setpoint 5cm) |
| **15** | Botão TURBULÊNCIA |
| **14** | LED indicador de alvo atingido |
| **21** | LCD I²C — SDA |
| **22** | LCD I²C — SCL |

---

## 💻 Software — Como Programar o ESP32

### Pré-requisitos

1. **Arduino IDE 2.x** (ou PlatformIO)
2. **Suporte ao ESP32** na IDE:
   - Abra: *Arquivo → Preferências*
   - Em "URLs adicionais do gerenciador de placas", adicione:
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Vá em *Ferramentas → Placa → Gerenciador de placas* e instale **esp32 by Espressif Systems**

### Bibliotecas necessárias

Instale pelo Gerenciador de Bibliotecas da Arduino IDE (*Sketch → Incluir Biblioteca → Gerenciar Bibliotecas*):

| Biblioteca | Autor | Versão |
|---|---|---|
| **LiquidCrystal_I2C** | Frank de Brabander | ≥ 1.1.2 |

> As bibliotecas FreeRTOS, `Wire.h` e `Arduino.h` já estão incluídas no pacote ESP32.

### Passos para gravar

```
1. Conecte o ESP32 ao computador via USB
2. Abra o arquivo: Controlador_Fuzzy_altitude_da_bolinha.ino
3. Selecione a placa: Ferramentas → Placa → ESP32 Dev Module
4. Selecione a porta COM correspondente ao ESP32
5. Clique em "Carregar" (botão de seta →)
6. Aguarde a mensagem "Done uploading"
```

---

## 🚀 Como Operar o Sistema

### 1. Primeira vez — Calibração

O sistema possui um menu de calibração de 6 etapas acionado pelo **Botão CAL (Pin 0)** com o motor **desarmado**:

| Etapa | Ação |
|---|---|
| **Passo 1 — Mantem** | Ajuste o potenciômetro até a bolinha flutuar parada |
| **Passo 2 — Subida** | Ajuste até a bolinha subir suavemente |
| **Passo 3 — Descida** | Ajuste até a bolinha descer suavemente |
| **Passo 4 — Arrancada** | Ajuste o pulso de partida do fundo do tubo |
| **Passo 5 — Máx. (cm)** | Defina o comprimento máximo do tubo |
| **Passo 6 — Mín. (cm)** | Defina a margem mínima do fundo |

> Em cada etapa, gire o potenciômetro e pressione o **Botão CAL** para salvar e avançar.

### 2. Operação Normal

```
1. Pressione Botão ARM (Pin 4) para armar o motor
   → O sistema enviará pulso max + min para inicializar o ESC (aguardar ~4s)
   → LED apagado = aguardando bolinha atingir o alvo

2. A Lógica Fuzzy assume o controle automaticamente
   → LED aceso = bolinha dentro de ±1.5cm do setpoint

3. Ajuste a altura-alvo:
   → Botão UP  (Pin 32): +5cm por clique
   → Botão DOWN (Pin 33): −5cm por clique

4. Botão TURBULÊNCIA (Pin 15): aplica perturbação de 2s para testar recuperação
5. Botão ARM novamente: desarma o motor
```

### 3. Monitor Serial

Abra o Monitor Serial a **115200 baud** para acompanhar a telemetria em tempo real:

```
--- MODO OPERACAO ---
Status     : ARMADO
Alvo (cm)  : 20.0
Altura (cm): 19.7
Erro (cm)  : 0.3
PWM Atual  : 3960
```

---

## 🧠 A Lógica Fuzzy (resumo técnico)

O controlador usa **Fuzzy Incremental** (Velocity Form / Mamdani), onde a saída é uma **variação de PWM (ΔPwm)** que se acumula ciclo a ciclo — dando ao sistema memória natural sem precisar de termo integral explícito.

### Entradas
| Variável | Descrição | Faixas |
|---|---|---|
| **Erro** | setpoint − altura_real | −15 a +15 cm |
| **ΔErro** | variação do erro / Δt | −10 a +10 cm/s |

### Base de Regras (3×3)

| Erro ╲ ΔErro | Subindo (N) | Parado (Z) | Caindo (P) |
|---|---|---|---|
| **Acima alvo (N)** | FN (−12) | N (−4) | Z (0) |
| **No alvo (Z)** | N (−4) | Z (0) | P (+4) |
| **Abaixo alvo (P)** | Z (0) | P (+4) | FP (+12) |

### Saída — ΔPwm crisp
`PWM(t) = PWM(t−1) + ΔPwm`

---

## ⚠️ Observações Importantes

- Os valores de calibração (**pwmMantem, pwmSubida, pwmDescida, pwmArrancada**) são salvos apenas em RAM. Desligue o sistema → valores voltam ao padrão. Para torná-los permanentes, adicione gravação em **EEPROM/NVS**.
- Nunca ligue o motor brushless sem hélice/propulsor. Apenas o fluxo de ar dentro do tubo sustenta a bolinha.
- O sensor HC-SR04 deve estar **fixo no topo do tubo**, apontando para baixo.
- Ajuste `setpointGlobal` no código (linha 62) para mudar a altura padrão ao ligar (padrão: 20.0 cm).

---

## 📁 Estrutura do Repositório

```
levitacao-bolinha-fuzzy/
│
└── Controlador_Fuzzy_altitude_da_bolinha.ino   # Código-fonte completo
```

---

## 📄 Licença

Projeto acadêmico — Disciplina de Inteligência Artificial, 2026.
