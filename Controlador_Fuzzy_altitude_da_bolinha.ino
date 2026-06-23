// PROJETO: Sistema de Levitação de Bolinha - ESP32 + FreeRTOS
// Aluno  : Jorge Luiz Madeira Pires
// Matéria: Inteligência Artificial
// v0.17    Adicionado botões de + e - altura setada, turbulencia e display LCD.

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

//PINAGEM ESP 32
#define PIN_POT       34   // potenciometro usado para controlar velocidade do motor no modo calibração 
#define PIN_ESC       18   // saida controlador esc do motor
#define PIN_BOTAO_ARM  4   // Botão para armar/desarmar
#define PIN_BOTAO_CAL  0   // Botão para avançar calibração
#define PIN_TRIG      25 // sensor utrassonico
#define PIN_ECHO      27 // sensor utrassonico
#define PIN_BOTAO_UP   32 // Botão para + 5 cm altura setada
#define PIN_BOTAO_DOWN 33 // Botão para - 5 cm altura setada
#define PIN_BOTAO_TURB 15 // Botão gerar turbulencia para reiniciar controle de aaltura
#define PIN_LED_ALVO  14  // saída indicação de acerto da altura alvo com 1,5 cm de tolerância 
                          // LCD na I2C pinos 21 e 22

//PWM DO ESC
#define LEDC_FREQUENCIA  50
#define LEDC_RESOLUCAO   16
#define PWM_ESC_MIN    3276    // Mínimo (1000us)
#define PWM_ESC_MAX    6553    // Máximo (2000us)
#define PWM_ESC_ARM    3276

// VARIAVEIS SENSOR ULTRASSÔNICO
#define ECHO_TIMEOUT_US 25000UL
#define FATOR_SOM        58.0f
#define SENSOR_JANELA    10

// MAQUINA DE ESTADO
enum ModoOperacao {
  MODO_NORMAL,
  CALIB_MANTEM,
  CALIB_SUBIDA,
  CALIB_DESCIDA,
  CALIB_ARRANCADA, 
  CALIB_MAX_CM,
  CALIB_MIN_CM
};

// VARIÁVEIS MODO CALIBRAÇÃO
float tuboMax = 40.0f;
float tuboMin = 2.0f;
int   pwmMantem  = 3960;   // velocidade base para manter bolinha parada
int   pwmSubida  = 4060;  // velocidade base para subir bolinha devagar
int   pwmDescida = 3940;  // velocidade base para descer bolinha devagar
int   pwmArrancada = 4060;  // velocidade base para bolinha arrancar do ponto 0 cm

// CONFIGURAÇÕES GERAIS
float passoCm = 5.0f;               
int   tempoTurbMs = 2000;   
int   pwmTurb = 4100;        
volatile float setpointGlobal = 20.0f;     

volatile bool   turbulenciaAtiva = false;
volatile uint32_t tempoTurbIniciado = 0;

//PARAMETROS DO CONTROLE FUZZY
#define FUZZY_DT         0.06f  // Intervalo de tempo da task (60ms)
#define LIMITE_ERRO      15.0f  // Limite para o erro máximo
#define LIMITE_DERIVADA  10.0f  // Limite para a variação do erro

// Constantes de saída para o Delta PWM
#define OUT_FN  -12.0f 
#define OUT_N   -4.0f  
#define OUT_Z     0.0f 
#define OUT_P    4.0f  
#define OUT_FP   12.0f 

//Variaveis globais
typedef struct {
  float distancia_filtrada_cm;
  float altura_cm;
} DadosSensor_t;

typedef struct {
  ModoOperacao modo_atual;
  bool  armado;
  float setpoint_cm;
  float altura_cm;
  float erro_cm;
  int   pwm_atual;
  int   pot_raw;
} StatusSistema_t;

// Recursos RTOS
QueueHandle_t     filaSensor;
QueueHandle_t     filaStatus;
SemaphoreHandle_t mutexESC;

LiquidCrystal_I2C lcd(0x27, 16, 2); 

static volatile bool escArmado = false;
static volatile int  pwmAtual  = PWM_ESC_ARM;
static volatile ModoOperacao modoAtual = MODO_NORMAL;

volatile uint32_t echoInicio = 0;
volatile uint32_t echoDuracao = 0;
volatile bool     echoPronto = false;

void IRAM_ATTR echoISR() {
  if (digitalRead(PIN_ECHO) == HIGH) {
    echoInicio = micros();
  } else {
    echoDuracao = micros() - echoInicio;
    echoPronto = true;
  }
}

// MOTOR DE INFERÊNCIA FUZZY
float pertinenciaTriangulo(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0f;
  if (x == b) return 1.0f;
  if (x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

float fuzzyMin(float a, float b) {
  return (a < b) ? a : b;
}

float calcularFuzzy(float erro, float delta_erro) {
  
  // 1. FUZZIFICAÇÃO DO ERRO
  float err_N = pertinenciaTriangulo(erro, -LIMITE_ERRO * 2, -LIMITE_ERRO, 0);
  float err_Z = pertinenciaTriangulo(erro, -LIMITE_ERRO, 0, LIMITE_ERRO);
  float err_P = pertinenciaTriangulo(erro, 0, LIMITE_ERRO, LIMITE_ERRO * 2);

  // 2. FUZZIFICAÇÃO DA VARIAÇÃO DO ERRO 
  // (dErr_N = indo para CIMA, dErr_Z = Parado, dErr_P = caindo)
  float dErr_N = pertinenciaTriangulo(delta_erro, -LIMITE_DERIVADA * 2, -LIMITE_DERIVADA, 0);
  float dErr_Z = pertinenciaTriangulo(delta_erro, -LIMITE_DERIVADA, 0, LIMITE_DERIVADA);
  float dErr_P = pertinenciaTriangulo(delta_erro, 0, LIMITE_DERIVADA, LIMITE_DERIVADA * 2);

  // 3. BASE DE REGRAS MAMDANI PARA CONTROLE INCREMENTAL
  float r1 = fuzzyMin(err_N, dErr_N); // Acima do alvo e subindo -> Corta motor forte
  float r2 = fuzzyMin(err_N, dErr_Z); // Acima do alvo e parado -> Corta motor suave
  float r3 = fuzzyMin(err_N, dErr_P); // Acima do alvo e descendo -> Zero (já está indo pro alvo, não faz nada)
  
  float r4 = fuzzyMin(err_Z, dErr_N); // No alvo mas subindo -> Corta suave (freia para não passar)
  float r5 = fuzzyMin(err_Z, dErr_Z); // No alvo e parado -> Zero (Mantém a força exata atual)
  float r6 = fuzzyMin(err_Z, dErr_P); // No alvo mas caindo -> Aumenta suave (segura a queda)
  
  float r7 = fuzzyMin(err_P, dErr_N); // Abaixo do alvo e subindo -> Zero (já está subindo rumo ao alvo)
  float r8 = fuzzyMin(err_P, dErr_Z); // Abaixo do alvo e parado -> Aumenta motor suave
  float r9 = fuzzyMin(err_P, dErr_P); // Abaixo do alvo e caindo -> Aumenta motor forte (Impede a queda)

  float numerador = 
    (r1 * OUT_FN) + 
    (r2 * OUT_N)  + 
    (r3 * OUT_Z)  + 
    (r4 * OUT_N)  + 
    (r5 * OUT_Z)  + 
    (r6 * OUT_P)  + 
    (r7 * OUT_Z)  + 
    (r8 * OUT_P)  + 
    (r9 * OUT_FP);

  float denominador = r1 + r2 + r3 + r4 + r5 + r6 + r7 + r8 + r9;

  if (denominador == 0) return 0.0f;

  return numerador / denominador;
}
// Controlador ESC's
void escSetPWM(int valor) {
  valor = constrain(valor, PWM_ESC_MIN, PWM_ESC_MAX);
  if (xSemaphoreTake(mutexESC, pdMS_TO_TICKS(10)) == pdTRUE) {
    ledcWrite(PIN_ESC, valor);
    pwmAtual = valor;
    xSemaphoreGive(mutexESC);
  }
}
// Utrassonico
float medirDistancia() {
  echoPronto = false;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  uint32_t t0 = micros();
  while (!echoPronto) {
    if ((micros() - t0) > ECHO_TIMEOUT_US) return -1.0f;
    taskYIELD();
  }
  if (echoDuracao == 0) return -1.0f;
  return (float)echoDuracao / FATOR_SOM;
}
// leitura e tratamento botôes e maquina de estado
void TaskBotoes(void *pvParameters) {
  bool ultArm = HIGH, ultCal = HIGH, ultUp = HIGH, ultDown = HIGH, ultTurb = HIGH;

  while (true) {
    bool btnArm = digitalRead(PIN_BOTAO_ARM);
    bool btnCal = digitalRead(PIN_BOTAO_CAL);
    bool btnUp  = digitalRead(PIN_BOTAO_UP);
    bool btnDown = digitalRead(PIN_BOTAO_DOWN);
    bool btnTurb = digitalRead(PIN_BOTAO_TURB);

    // Lógica para Armar / Desarmar
    if (ultArm == HIGH && btnArm == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50));
      if (modoAtual == MODO_NORMAL) {
        if (!escArmado) {
          Serial.println(F("\n[SISTEMA] Armanda ESC..."));
          ledcWrite(PIN_ESC, PWM_ESC_MAX);
          vTaskDelay(pdMS_TO_TICKS(2000));
          ledcWrite(PIN_ESC, PWM_ESC_ARM);
          vTaskDelay(pdMS_TO_TICKS(2000));
          escArmado = true;
          Serial.println(F("[ESC] Motor pronto para uso."));
        } else {
          escArmado = false;
          escSetPWM(PWM_ESC_ARM);
          Serial.println(F("\n[SISTEMA] Motor desligado!"));
        }
      }
    }

    // Navegação do menu de calibração
    if (ultCal == HIGH && btnCal == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50));
      if (!escArmado || modoAtual != MODO_NORMAL) {
        switch (modoAtual) {
          case MODO_NORMAL:   modoAtual = CALIB_MANTEM;   escArmado = true; break; 
          case CALIB_MANTEM:  modoAtual = CALIB_SUBIDA;   break;
          case CALIB_SUBIDA:  modoAtual = CALIB_DESCIDA;  break;
          case CALIB_DESCIDA: modoAtual = CALIB_ARRANCADA; break; 
          case CALIB_ARRANCADA: modoAtual = CALIB_MAX_CM;  break;
          case CALIB_MAX_CM:  modoAtual = CALIB_MIN_CM;   break;
          case CALIB_MIN_CM:  
            modoAtual = MODO_NORMAL; 
            escArmado = false; 
            escSetPWM(PWM_ESC_ARM);
            Serial.println(F("\n[CALIB] Parametros salvos. Motor desarmado."));
            break;
        }
      } else {
         Serial.println(F("[AVISO] Desarme o motor para calibrar!"));
      }
    }

    // Ajuste de setpoint manual (+)
    if (ultUp == HIGH && btnUp == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50));
      setpointGlobal += passoCm;
      if (setpointGlobal > tuboMax) setpointGlobal = tuboMax;
    }

    // Ajuste de setpoint manual (-)
    if (ultDown == HIGH && btnDown == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50));
      setpointGlobal -= passoCm;
      if (setpointGlobal < tuboMin) setpointGlobal = tuboMin;
    }

    // Ativação da perturbação externa (Turbulência)
    if (ultTurb == HIGH && btnTurb == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50));
      if (modoAtual == MODO_NORMAL && escArmado && !turbulenciaAtiva) {
        turbulenciaAtiva = true;
        tempoTurbIniciado = millis();
        Serial.println(F("\n[TESTE] Turbulencia acionada!"));
      }
    }

    ultArm = btnArm; ultCal = btnCal; ultUp = btnUp; ultDown = btnDown; ultTurb = btnTurb;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// leitura e filtro sensor utrassonico
void TaskUltrasonico(void *pvParameters) {
  float bufSensor[SENSOR_JANELA] = {0};
  uint8_t idx = 0;
  DadosSensor_t pacote;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    float bruta = medirDistancia();
    if (bruta > 0 && bruta < 400.0f) {
      bufSensor[idx] = bruta;
      idx = (idx + 1) % SENSOR_JANELA;
      float soma = 0;
      for (int i = 0; i < SENSOR_JANELA; i++) soma += bufSensor[i];
      float filtrada = soma / SENSOR_JANELA;
      
      float altura = tuboMax - filtrada; 
      altura = constrain(altura, 0.0f, tuboMax);

      pacote.distancia_filtrada_cm = filtrada;
      pacote.altura_cm = altura;
    }
    xQueueOverwrite(filaSensor, &pacote);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60));
  }
}

// controlador baseado em regras fuzzy
void TaskControle(void *pvParameters) {
  DadosSensor_t sensor;
  StatusSistema_t status;
  
  float erroAntes = 0;
  bool primeiroCiclo = true;
  float pwmAcumulado = PWM_ESC_ARM; 
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true) {
    int pot = analogRead(PIN_POT);
    float alturaReal = 0;
    
    if (xQueuePeek(filaSensor, &sensor, 0) == pdTRUE) {
      alturaReal = sensor.altura_cm;
    }

    int pwmAlvo = PWM_ESC_ARM;

    if (modoAtual == MODO_NORMAL) {
      
      float setpoint = setpointGlobal;
      float erro = setpoint - alturaReal;

      if (primeiroCiclo) {
        erroAntes = erro;
        pwmAcumulado = pwmMantem; 
        primeiroCiclo = false;
      }

      if (escArmado) {
        // Controle do LED indicador de meta atingida
        if (erro >= -1.5f && erro <= 1.5f) {
            digitalWrite(PIN_LED_ALVO, HIGH);
        } else {
            digitalWrite(PIN_LED_ALVO, LOW);
        }

        float delta_erro = (erro - erroAntes) / FUZZY_DT;
        erroAntes = erro;

        float deltaPWM = calcularFuzzy(erro, delta_erro);
        pwmAcumulado += deltaPWM;
        pwmAcumulado = constrain(pwmAcumulado, pwmDescida - 100, pwmSubida + 250);
        pwmAlvo = (int)pwmAcumulado;
        
        // vencer o atrtito inicial
        int tetoMaximo = pwmSubida;
        if (alturaReal <= (tuboMin + 1.5f) && erro > 1.5f) {
          if (pwmAlvo < pwmArrancada) {
            pwmAlvo = pwmArrancada; 
            pwmAcumulado = pwmArrancada; 
          }
          if (pwmArrancada > tetoMaximo) {
            tetoMaximo = pwmArrancada;
          }
        }

        pwmAlvo = constrain(pwmAlvo, pwmDescida, tetoMaximo);

        //pulso forçado
        if (turbulenciaAtiva) {
          if ((millis() - tempoTurbIniciado) < tempoTurbMs) {
            pwmAlvo = pwmTurb; 
            erroAntes = erro; 
            pwmAcumulado = pwmTurb; 
          } else {
            turbulenciaAtiva = false;
            Serial.println(F("[TESTE] Perturbacao finalizada."));
          }
        }

        pwmAlvo = constrain(pwmAlvo, PWM_ESC_MIN, PWM_ESC_MAX);
        escSetPWM(pwmAlvo);

      } else {
        erroAntes = erro;
        pwmAcumulado = pwmMantem; 
        escSetPWM(PWM_ESC_ARM);
        digitalWrite(PIN_LED_ALVO, LOW); 
      }

      status.setpoint_cm = setpoint;
      status.erro_cm = erro;
    } 

    // modo calibração (pouco funcional)
    else {
      erroAntes = 0; 
      pwmAcumulado = pwmMantem;
      digitalWrite(PIN_LED_ALVO, LOW); 

      if (modoAtual == CALIB_MANTEM || modoAtual == CALIB_SUBIDA || modoAtual == CALIB_DESCIDA || modoAtual == CALIB_ARRANCADA) {
        pwmAlvo = map(pot, 0, 4095, pwmDescida - 50, pwmArrancada + 50);
        pwmAlvo = constrain(pwmAlvo, PWM_ESC_MIN, PWM_ESC_MAX);
        escSetPWM(pwmAlvo);
        
        if (modoAtual == CALIB_MANTEM)    pwmMantem    = pwmAlvo;
        if (modoAtual == CALIB_SUBIDA)    pwmSubida    = pwmAlvo;
        if (modoAtual == CALIB_DESCIDA)   pwmDescida   = pwmAlvo;
        if (modoAtual == CALIB_ARRANCADA) pwmArrancada = pwmAlvo; 
      }
      else if (modoAtual == CALIB_MAX_CM) {
        tuboMax = map(pot, 0, 4095, 10, 100);
        escSetPWM(PWM_ESC_ARM); 
      }
      else if (modoAtual == CALIB_MIN_CM) {
        tuboMin = map(pot, 0, 4095, 0, 20); 
        escSetPWM(PWM_ESC_ARM);
      }
      
      status.setpoint_cm = 0;
      status.erro_cm = 0;
    }

    status.modo_atual = modoAtual;
    status.armado = escArmado;
    status.altura_cm = alturaReal;
    status.pwm_atual = pwmAtual;
    status.pot_raw = pot;
    
    xQueueOverwrite(filaStatus, &status);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60));
  }
}

// serial e display lcd
void TaskSerial(void *pvParameters) {
  StatusSistema_t st;
  char buf1[17]; 
  char buf2[17];

  while (true) {
    if (xQueuePeek(filaStatus, &st, pdMS_TO_TICKS(100)) == pdTRUE) {
      
      Serial.println(F("\n----------------------------------------"));
      if (st.modo_atual == MODO_NORMAL) {
        Serial.println(F("--- MODO OPERACAO ---"));
        Serial.print(F("Status: ")); Serial.println(st.armado ? F("ARMADO") : F("DESARMADO"));
        if(turbulenciaAtiva) {
            Serial.println(F("[!] Alerta: Turbulencia ativa"));
        }
        Serial.print(F("Alvo (cm)   : ")); Serial.println(st.setpoint_cm, 1);
        Serial.print(F("Altura (cm) : ")); Serial.println(st.altura_cm, 1);
        Serial.print(F("Erro (cm)   : ")); Serial.println(st.erro_cm, 1);
        Serial.print(F("PWM Atual   : ")); Serial.println(st.pwm_atual);

        sprintf(buf1, "S:%4.1f A:%4.1f ", st.setpoint_cm, st.altura_cm);
        lcd.setCursor(0, 0); 
        lcd.print(buf1);
        
        sprintf(buf2, "E:%4.1f Est:%c   ", st.erro_cm, st.armado ? 'A' : 'D');
        lcd.setCursor(0, 1); 
        lcd.print(buf2);

      } 
      else {
        Serial.println(F("--- ROTINA DE CALIBRACAO ---"));
        Serial.println(F("Ajuste o trimpot e aperte o B2 para avançar."));
        
        switch(st.modo_atual) {
          case CALIB_MANTEM:
            Serial.println(F("Passo 1: Ajuste a flutuação central."));
            Serial.print(F("PWM Salvo: ")); Serial.println(pwmMantem);
            break;
          case CALIB_SUBIDA:
            Serial.println(F("Passo 2: Ajuste o limite de subida."));
            Serial.print(F("PWM Salvo: ")); Serial.println(pwmSubida);
            break;
          case CALIB_DESCIDA:
            Serial.println(F("Passo 3: Ajuste a descida controlada."));
            Serial.print(F("PWM Salvo: ")); Serial.println(pwmDescida);
            break;
          case CALIB_ARRANCADA:
            Serial.println(F("Passo 4: Ajuste o pulso de partida inicial."));
            Serial.print(F("PWM Salvo: ")); Serial.println(pwmArrancada);
            break;
          case CALIB_MAX_CM:
            Serial.println(F("Passo 5: Definir o limite superior do tubo."));
            Serial.print(F("Max (cm): ")); Serial.println(tuboMax);
            break;
          case CALIB_MIN_CM:
            Serial.println(F("Passo 6: Definir o limite inferior do tubo."));
            Serial.print(F("Min (cm): ")); Serial.println(tuboMin);
            break;
        }
        Serial.print(F("Leitura Pot: ")); Serial.println(st.pot_raw);

        lcd.setCursor(0, 0); 
        lcd.print("CALIBRACAO      ");
        sprintf(buf2, "Etapa: %d       ", st.modo_atual);
        lcd.setCursor(0, 1); 
        lcd.print(buf2);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(400));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Iniciando...");

  Serial.println(F("\n-----------------------------------------"));
  Serial.println(F(" SISTEMA DE LEVITACAO - CONTROLADOR FUZZY "));
  Serial.println(F("-----------------------------------------"));

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ledcAttach(PIN_ESC, LEDC_FREQUENCIA, LEDC_RESOLUCAO);
  ledcWrite(PIN_ESC, PWM_ESC_ARM);

  pinMode(PIN_BOTAO_ARM, INPUT_PULLUP);
  pinMode(PIN_BOTAO_CAL, INPUT_PULLUP);
  pinMode(PIN_BOTAO_UP, INPUT_PULLUP);
  pinMode(PIN_BOTAO_DOWN, INPUT_PULLUP);
  pinMode(PIN_BOTAO_TURB, INPUT_PULLUP);
  
  pinMode(PIN_LED_ALVO, OUTPUT);
  digitalWrite(PIN_LED_ALVO, LOW);

  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), echoISR, CHANGE);

  mutexESC   = xSemaphoreCreateMutex();
  filaSensor = xQueueCreate(1, sizeof(DadosSensor_t));
  filaStatus = xQueueCreate(1, sizeof(StatusSistema_t));

  // Inicialização das Tasks no FreeRTOS
  xTaskCreatePinnedToCore(TaskUltrasonico, "Sensor", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskBotoes,      "Botoes", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskControle,    "Fuzzy",  4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskSerial,      "Serial", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}