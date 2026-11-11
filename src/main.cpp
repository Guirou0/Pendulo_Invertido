/*
  Pêndulo invertido com roda de reação — controle em malha fechada
  Motor:    Gimbal 2804 (12N14P -> 7 pares de polos)
  Encoder:  AS5600 (I2C bus 0 / Wire), no EIXO DO MOTOR/RODA
  IMU:      GY-521 / MPU6050 (I2C bus 1 / Wire1), fixado na HASTE
  Driver:   SimpleFOCmini (DRV8313, 3PWM)
  Placa:    ESP32 Doit DevKit V1
  Build:    PlatformIO (env esp32doit-devkit-v1 em platformio.ini)
  Telemetria e comandos: Bluetooth Classic (SPP), sem fio ao notebook

  CONVENÇÃO DE EIXOS DO GY-521 (ver explicação completa na resposta)
  --------------------------------------------------------------------
  O GY-521 é fixado à HASTE com o plano da placa alinhado ao PLANO DE
  OSCILAÇÃO do pêndulo, e o eixo Z do sensor apontando para FORA desse
  plano, paralelo ao eixo de rotação do pivô.
    - Ângulo da haste em relação à vertical: angle = atan2(ax, ay)
    - Velocidade angular correspondente:     gz (°/s)
  Se o sinal medido vier invertido no seu teste (a haste cai para um
  lado mas o ângulo soma no sentido errado), troque ANGLE_SIGN ou
  GYRO_SIGN de +1.0f para -1.0f logo abaixo.

  MODOS (trocar via Bluetooth, comando "M0" ou "M1")
  --------------------------------------------------------------------
  M0 = teste em malha aberta (onda quadrada de tensão), como antes.
  M1 = balanceamento por PID usando o ângulo do GY-521 (padrão).

  COMANDOS BLUETOOTH (digite no app de terminal serial BT, Enter após):
    A<valor>  amplitude da onda quadrada no modo M0 (V)
    P<valor>  período da onda quadrada no modo M0 (ms)
    M<0 ou 1> troca de modo
    K<valor>  Kp do PID
    I<valor>  Ki do PID
    D<valor>  Kd do PID
    S<valor>  setpoint do PID, em graus (0 = vertical)
    R         inverte o sentido do torque do motor (alterna +1/-1)
    L<seg>    pede o log por <seg> segundos (ex.: L10). "L" sozinho = 5 s, L0 para.
              (o log NÃO é mais automático)
    F         formato do log: F1 = CSV (planilha), F0 = texto legível, "F" alterna
    T<hz>     taxa do log (1 a 50 Hz)
    B         calibra o offset do giroscópio (haste PARADA, 2 s, motor desligado)
    Q         mostra parâmetros atuais, taxa do loop e motivo do último reset
    H         legenda de cada coluna do log + lista de comandos
    J<volts>  limite do termo integral (padrão 1, máx. 4)
    M2        PID + pulsos: amplitude do pulso vinda do PID (mínimo C, máximo V), sentido = sinal do PID
    C<volts>  M2: amplitude mínima do pulso | N<graus> zona morta | O<ms> tempo mínimo entre reversões
    X<volts>  kick de partida (vale em M1 e M2; padrão 2 V): recua a roda e inverte para pegar o pico (X0 desliga)
    G<ms>     kick: tempo parado antes de disparar (padrão 120) | U<dps> |gz| que conta como parado (15) | E<ms> recuo (120)
    #<volts>  tensão de alimentação real (6 a 16 V), ex.: #12.4 — meça com multímetro; W salva
    W         SALVA os parâmetros na memória (carregados a cada boot) | Y apaga o que foi salvo
    V<volts>  limite de tensão do motor/PID (padrão 4, máx. 8). Suba aos poucos.
    Z         setpoint = ângulo atual (segure a haste no ponto de equilíbrio antes)

  AVISO: BluetoothSerial (SPP clássico) conecta com Android normalmente
  via apps como "Serial Bluetooth Terminal". iPhone/iOS não suporta SPP
  clássico com apps genéricos — se for usar iOS, me avise para trocar
  para BLE.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "BluetoothSerial.h"
#include <esp_system.h>   // esp_reset_reason()
#include <Preferences.h>  // memória não volátil (NVS) para salvar parâmetros

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth não habilitado no build. Rode `idf.py menuconfig`/verifique o framework ESP32 Arduino.
#endif

// ---------- Pinos do driver (SimpleFOCmini) ----------
// Evita strapping pins (0,2,12,15), flash SPI (6-11) e pinos só-entrada (34-39).
#define PIN_PWM_A   25
#define PIN_PWM_B   26
#define PIN_PWM_C   27
#define PIN_ENABLE  33

// ---------- I2C bus 0: AS5600 no eixo do motor/roda ----------
// Pinos padrão do ESP32 (Wire): SDA=21, SCL=22

// ---------- I2C bus 1: GY-521/MPU6050 na haste ----------
// Barramento separado (Wire1) para não compartilhar tráfego/colisões com o
// encoder, que precisa de leitura rápida e constante para o FOC.
#define MPU_SDA 16
#define MPU_SCL 17
#define MPU_ADDR 0x68

// Alimentação
#define SUPPLY_VOLTAGE 13.70f  // tensão da fonte do driver (dentro de 7.4-16V do motor)
float VOLTAGE_LIMIT_MAX = 4.0f;   // ajustável em tempo real pelo comando V (máx. 8 V) // trava de segurança (não passe disso sem revisar o motor)

// ---------- Hardware: motor/driver/encoder ----------
BLDCMotor motor = BLDCMotor(7);                     // 2804 12N14P -> 7 pares de polos
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ---------- Bluetooth (substitui o Serial para telemetria/comandos) ----------
// ---- "Kick" de partida (comando X): recua a roda e depois inverte para pegar o pico de torque ----
// Dispara quando a haste fica parada (|gz| < U) com erro grande (> KICK_ERR_DEG) por G ms. Recuo = E ms, pulso = KICK_T2_MS.
#define KICK_ERR_DEG     3.0f
#define KICK_T2_MS       200    // fase 2: inverte para o lado da subida (pico de torque)
#define KICK_COOLDOWN_MS 250
#define TELEM_HZ 20   // taxa do log via Bluetooth (Hz). Suba para 50 só se o app aguentar.
BluetoothSerial SerialBT;
// ---- Interpretador de comandos próprio (Bluetooth) ----
// O Commander do SimpleFOC só guarda 20 comandos; registrar mais que isso corrompe a memória
// (foi o que quebrou M e R). Aqui o limite é MAX_CMDS. Aceita "K30", "k30", "K 30"; Enter = \n ou \r\n.
#define MAX_CMDS 40
struct CmdEntry { char id; void (*fn)(char*); };
CmdEntry cmd_tab[MAX_CMDS];
int cmd_n = 0;

void cmdAdd(char id, void (*fn)(char*), const char* label) {
  if (cmd_n < MAX_CMDS) { cmd_tab[cmd_n].id = id; cmd_tab[cmd_n].fn = fn; cmd_n++; }
}

// Se veio argumento, grava; sempre mostra o valor atual.
void cmdScalar(float* v, char* arg) {
  if (arg != NULL && arg[0] != '\0') *v = atof(arg);
  SerialBT.println(*v, 3);
}

void cmdRun() {
  static char buf[32];
  static uint8_t len = 0;
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        char id = (char)toupper((unsigned char)buf[0]);
        char* arg = &buf[1];
        while (*arg == ' ') arg++;
        bool achou = false;
        for (int i = 0; i < cmd_n; i++) {
          if (cmd_tab[i].id == id) { cmd_tab[i].fn(arg); achou = true; break; }
        }
        if (!achou) { SerialBT.print("comando desconhecido: "); SerialBT.println(buf); }
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;   // linha longa demais: descarta
    }
  }
}

// ============================================================
//  Driver mínimo do MPU6050, em Wire1 (bus I2C separado)
// ============================================================
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_CONFIG       0x1A
// Filtro digital do MPU6050 (reduz vibração do motor no acelerômetro/giro):
// 0 = desligado (260 Hz) | 1 = 184 Hz | 2 = 94 Hz | 3 = 44 Hz | 4 = 21 Hz
#define MPU_DLPF_CFG 3
#define MPU_REG_ACCEL_XOUT_H 0x3B

const float ACCEL_SENS = 16384.0f; // LSB/g   (faixa ±2g, default)
float gyro_bias_z = 0;             // °/s, offset do giro Z (comando B calibra)
const float GYRO_SENS  = 131.0f;   // LSB/°/s (faixa ±250°/s, default)

bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  return Wire1.endTransmission() == 0;
}

bool mpuInit() {
  Wire1.begin(MPU_SDA, MPU_SCL, 400000);
  bool ok = true;
  ok &= mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00);   // sai do modo sleep
  delay(50);
  ok &= mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00); // força ±2g
  ok &= mpuWriteReg(MPU_REG_CONFIG, MPU_DLPF_CFG);   // filtro digital (ver MPU_DLPF_CFG)
  ok &= mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);  // força ±250°/s
  return ok;
}

struct MpuSample {
  float ax, ay, az;  // g
  float gx, gy, gz;  // °/s
  bool  ok;
};

MpuSample mpuRead() {
  MpuSample s{0, 0, 0, 0, 0, 0, false};
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(MPU_REG_ACCEL_XOUT_H);
  if (Wire1.endTransmission(false) != 0) return s;
  uint8_t n = Wire1.requestFrom((int)MPU_ADDR, 14, (int)true);
  if (n < 14) return s;

  int16_t raw_ax = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_ay = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_az = (Wire1.read() << 8) | Wire1.read();
  Wire1.read(); Wire1.read();   // temperatura (descartada)
  int16_t raw_gx = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_gy = (Wire1.read() << 8) | Wire1.read();
  int16_t raw_gz = (Wire1.read() << 8) | Wire1.read();

  s.ax = raw_ax / ACCEL_SENS;
  s.ay = raw_ay / ACCEL_SENS;
  s.az = raw_az / ACCEL_SENS;
  s.gx = raw_gx / GYRO_SENS;
  s.gy = raw_gy / GYRO_SENS;
  s.gz = raw_gz / GYRO_SENS - gyro_bias_z;
  s.ok = true;
  return s;
}

// ============================================================
//  Estimativa de ângulo: filtro complementar (acelerômetro + giro)
// ============================================================
#define ANGLE_SIGN 1.0f   // troque para -1.0f se o ângulo sair invertido
#define GYRO_SIGN  1.0f   // troque para -1.0f se a taxa sair invertida

float angle_est = 0;              // rad, ângulo filtrado da haste
unsigned long t_prev_imu = 0;
const float ALPHA_COMP = 0.98f;   // peso do giroscópio no filtro complementar

void updateAngle(const MpuSample &s) {
  unsigned long now = micros();
  float dt = (t_prev_imu == 0) ? 0.001f : (now - t_prev_imu) * 1e-6f;
  t_prev_imu = now;

  float angle_accel    = ANGLE_SIGN * atan2(s.ax, s.ay);     // rad, só acelerômetro
  float gyro_rate_rads = GYRO_SIGN * s.gz * DEG_TO_RAD;       // rad/s

  // Giro para a dinâmica rápida, acelerômetro para corrigir o desvio (drift) do giro
  angle_est = ALPHA_COMP * (angle_est + gyro_rate_rads * dt)
            + (1.0f - ALPHA_COMP) * angle_accel;
}

// ============================================================
//  PID de balanceamento
// ============================================================
float Kp = 3.0f, Ki = 2.0f, Kd = 0.18f;   // ponto de partida — precisa ser ajustado na bancada
float setpoint_deg = 0.0f;                 // 0° = vertical
float pid_integral = 0;                    // termo integral já em Volts (Ki incluso)
unsigned long t_prev_pid = 0;
const float SAFETY_ANGLE_DEG = 70.0f;      // acima disso, considera "caído" e desliga
float I_MAX_V = 1.0f;   // ajustável pelo comando J                // limite do termo integral (anti-windup), em V
// ---- Variáveis de depuração (preenchidas pelo PID, lidas pelo log) ----
float dbg_err_deg = 0;      // erro = setpoint - ângulo (graus)
float dbg_P = 0, dbg_I = 0, dbg_D = 0;   // contribuição de cada termo (V)
float dbg_pre = 0;          // soma P+I+D ANTES de limitar (V)
bool  dbg_sat = false;      // true se a saída foi cortada pelo limite de tensão
bool  dbg_fallen = false;   // true se |ângulo| > SAFETY_ANGLE_DEG (PID desligado)
float loop_hz = 0;          // voltas do loop() por segundo
uint32_t mpu_falhas = 0;    // leituras do MPU que falharam
bool  fmt_csv = true;       // formato do log: true = CSV, false = texto legível
float telem_hz = TELEM_HZ;  // taxa do log (Hz)
const char* reset_motivo = "?";
uint32_t log_ate_ms = 0;    // fim do log pedido (comando L)
bool     log_ativo  = false;
// ---- Modo M2: PID + pulsos (amplitude do PID, mínimo = kick) ----
float bb_V = 2.0f;          // amplitude do pulso (V)               comando C
float bb_db_deg = 0.8f;     // zona morta da função de chaveamento (graus)  comando N
float bb_dwell_ms = 60.0f;  // tempo mínimo entre reversões (ms)     comando O
// Tensão de alimentação usada pelo driver para converter volts em PWM (comando #). Meça a fonte/bateria
// com um multímetro e ajuste: se o valor real for menor que este, o motor recebe menos tensão que a pedida.
float supply_V = SUPPLY_VOLTAGE;

// Tempos do kick (ajustáveis): quanto menores, mais rápido ele tenta de novo depois de bater no limite
float kick_wait_ms = 120.0f;   // tempo parado antes de disparar o kick (ms)      comando G
float kick_gz_dps  = 15.0f;    // "parado" = |gz| abaixo disso (graus/s)          comando U
float kick_t1_ms   = 120.0f;   // duração do recuo da roda (ms)                   comando E
float    kick_V     = 2.0f;    // tensão do kick (V). 0 = desligado (comando X)
// calibração do offset do giroscópio (comando B)
bool bias_cal = false; double bias_soma = 0; uint32_t bias_cnt = 0, bias_fim_ms = 0;

bool paramsSalvos();   // definida mais abaixo (usa a memória NVS)

const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on (ligou a energia)";
    case ESP_RST_EXT:       return "pino EN / reset externo";
    case ESP_RST_SW:        return "software (ESP.restart)";
    case ESP_RST_PANIC:     return "PANIC / excecao (crash do codigo)";
    case ESP_RST_INT_WDT:   return "watchdog de interrupcao";
    case ESP_RST_TASK_WDT:  return "watchdog de tarefa";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (queda de tensao na alimentacao)";
    default:                return "outro / desconhecido";
  }
}

float dir_motor = 1.0f;                    // +1 / -1: sentido do torque (comando R inverte)

void resetPID() {
  pid_integral = 0;
  t_prev_pid = 0;
}

float computePID(float angle_rad, float gyro_rate_rads) {
  unsigned long now = micros();
  float dt = (t_prev_pid == 0) ? 0.005f : (now - t_prev_pid) * 1e-6f;
  t_prev_pid = now;
  dt = constrain(dt, 1e-4f, 0.05f);        // protege contra dt zero ou saltos grandes

  if (fabs(angle_rad) * RAD_TO_DEG > SAFETY_ANGLE_DEG) {
    pid_integral = 0;    // pêndulo caído: desliga e não acumula erro
    dbg_fallen = true;
    dbg_err_deg = setpoint_deg - angle_rad * RAD_TO_DEG;
    dbg_P = dbg_I = dbg_D = dbg_pre = 0;
    dbg_sat = false;
    return 0;
  }

  float error = setpoint_deg * DEG_TO_RAD - angle_rad;
  dbg_fallen  = false;
  dbg_err_deg = error * RAD_TO_DEG;

  // P
  float upk = Kp * error;
  dbg_P = upk;

  // I com anti-windup: limitado a ±I_MAX_V e só integra se a saída não estiver saturada
  // empurrando no mesmo sentido do erro
  pid_integral += Ki * error * dt;
  pid_integral = constrain(pid_integral, -I_MAX_V, I_MAX_V);

  // D: usa a taxa do giroscópio (mais limpa que derivar o ângulo). d(erro)/dt = -taxa
  float udk = Kd * (-gyro_rate_rads);
  dbg_D = udk;

  float output = upk + pid_integral + udk;
  float sat = constrain(output, -VOLTAGE_LIMIT_MAX, VOLTAGE_LIMIT_MAX);
  if (sat != output && (output * error) > 0) {
    pid_integral -= Ki * error * dt;       // desfaz a integração se saturou a favor do erro
  }

  dbg_I   = pid_integral;
  dbg_pre = output;
  dbg_sat = (sat != output);
  return dir_motor * sat;                  // dir_motor inverte o sentido do torque
}

// ============================================================
//  Modo de teste em malha aberta (herdado do teste anterior)
// ============================================================
float amplitude_V = 1.0f;
float periodo_ms  = 700;
unsigned long t_ultima_troca = 0;
int sentido = 1;

// ============================================================
//  Comandos Bluetooth
// ============================================================
float modo = 1.0f;   // 0 = onda quadrada (M0) | 1 = PID de balanceamento (M1)

void onAmp(char* cmd)  { cmdScalar(&amplitude_V, cmd); }
void onPer(char* cmd)  { cmdScalar(&periodo_ms, cmd); }
void onModo(char* cmd) { cmdScalar(&modo, cmd); resetPID(); }

// R: inverte o sentido do torque do motor (use se ele reagir para o lado errado)
void onInv(char* cmd) {
  dir_motor = -dir_motor;
  resetPID();
  SerialBT.print("sentido motor: ");
  SerialBT.println(dir_motor > 0 ? "normal (+1)" : "invertido (-1)");
}

void printHeader() {
  SerialBT.println("t_ms,modo,ang_deg,ang_acc_deg,gz_dps,erro_deg,P_V,I_V,D_V,pid_V,saida_V,sat,caido,vel_roda_rads,ax_g,ay_g,az_g,clip,loop_hz,mpu_falhas");
}

// F: troca o formato do log. F1 = CSV (para planilha/gráfico), F0 = texto legível. "F" alterna.
void onFmt(char* cmd) {
  fmt_csv = (cmd == NULL || cmd[0] == '\0') ? !fmt_csv : (atof(cmd) > 0.5f);
  SerialBT.println(fmt_csv ? "formato do log: CSV" : "formato do log: texto legivel");
}

// T<hz>: taxa do log (1..50 Hz)
void onTelem(char* cmd) {
  float v = (cmd != NULL) ? atof(cmd) : 0;
  if (v >= 1.0f && v <= 50.0f) telem_hz = v;
  SerialBT.printf("taxa do log: %.0f Hz\n", telem_hz);
}

// B: calibra o offset do giroscópio (haste PARADA). Motor desligado durante 2 s.
void onBias(char* cmd) {
  bias_soma = 0; bias_cnt = 0; bias_fim_ms = millis() + 2000; bias_cal = true;
  resetPID();
  SerialBT.println("calibrando bias do giro por 2 s: segure a haste PARADA (motor desligado)...");
}

// Q: mostra todos os parâmetros atuais
void onStatus(char* cmd) {
  SerialBT.printf("modo=M%d  sentido_motor=%+d  setpoint=%.2f deg\n", modo < 0.5f ? 0 : (modo > 1.5f ? 2 : 1), (int)dir_motor, setpoint_deg);
  SerialBT.printf("Kp=%.3f (= %.3f V por grau)  Ki=%.3f  Kd=%.3f\n", Kp, Kp * DEG_TO_RAD, Ki, Kd);
  SerialBT.printf("limite_saida=%.1f V  I_MAX=%.1f V  corte_queda=%.0f deg  fonte=%.2f V\n",
                  VOLTAGE_LIMIT_MAX, I_MAX_V, SAFETY_ANGLE_DEG, supply_V);
  SerialBT.printf("onda quadrada (M0): amp=%.2f V  periodo=%.0f ms\n", amplitude_V, periodo_ms);
  SerialBT.printf("log: %s a %.0f Hz, %s\n", fmt_csv ? "CSV" : "texto", telem_hz, log_ativo ? "ATIVO" : "parado");
  SerialBT.printf("loop=%.0f Hz  mpu_falhas=%lu  bias_gz=%.3f dps  DLPF=%d\n",
                  loop_hz, (unsigned long)mpu_falhas, gyro_bias_z, (int)MPU_DLPF_CFG);
  SerialBT.printf("kick=%.1f V (0 = desligado)  C=%.2f V  N=%.2f deg  O=%.0f ms\n", kick_V, bb_V, bb_db_deg, bb_dwell_ms);
  SerialBT.printf("kick tempos: G(espera)=%.0f ms  U(|gz| parado)=%.0f dps  E(recuo)=%.0f ms\n", kick_wait_ms, kick_gz_dps, kick_t1_ms);
  SerialBT.printf("memoria: %s\n", paramsSalvos() ? "ha parametros salvos (carregados no boot)" : "nada salvo (valores padrao do codigo)");
  SerialBT.printf("ultimo reset: %s\n", reset_motivo);
}

// H: legenda do log e lista de comandos
void onHelp(char* cmd) {
  SerialBT.println("--- COLUNAS DO LOG (CSV) ---");
  SerialBT.println("t_ms        tempo desde o boot (ms)");
  SerialBT.println("modo        0 = onda quadrada, 1 = PID");
  SerialBT.println("ang_deg     angulo da haste filtrado (graus, 0 = vertical) <- o que o PID usa");
  SerialBT.println("ang_acc_deg angulo so do acelerometro (graus), compare com ang_deg");
  SerialBT.println("gz_dps      velocidade angular da haste (graus/s), bias ja removido");
  SerialBT.println("erro_deg    setpoint - ang_deg (graus)");
  SerialBT.println("P_V I_V D_V contribuicao de cada termo do PID (V)");
  SerialBT.println("pid_V       P+I+D antes de limitar (V)");
  SerialBT.println("saida_V     tensao enviada ao motor (V, ja com limite e sentido R)");
  SerialBT.println("sat         1 = pid_V foi cortado pelo limite de tensao");
  SerialBT.println("caido       1 = |angulo| > corte, PID desligado");
  SerialBT.println("vel_roda    velocidade da roda (rad/s)");
  SerialBT.println("ax,ay,az    acelerometro (g). ay~+1 e ax~0 com a haste em pe");
  SerialBT.println("clip        1 = acelerometro no fundo de escala (2 g)");
  SerialBT.println("loop_hz     voltas do loop por segundo (ideal > 500)");
  SerialBT.println("mpu_falhas  leituras do MPU que falharam");
  SerialBT.println("--- COMANDOS ---");
  SerialBT.println("M0/M1 modo | K I D ganhos | S setpoint | R inverte motor | A P onda quadrada");
  SerialBT.println("L<seg> log (L0 para) | F formato | T<hz> taxa | B bias giro | Q status | H ajuda");
  SerialBT.println("M2 = PID + pulsos (K I D J S R valem): C = amplitude MINIMA do pulso | N = zona morta (graus) | O = dwell (ms)");
  SerialBT.println("M1 = PID simples (X<volts> liga o kick de partida) | J V Z B Q | W salva parametros | Y apaga salvos | #<V> tensao da fonte");
}

// L<seg>: pede o log por <seg> segundos (padrão 5). L0 para.
void onLog(char* cmd) {
  float seg = (cmd != NULL && cmd[0] != '\0') ? atof(cmd) : 5.0f;
  if (seg <= 0) {
    log_ativo = false;
    SerialBT.println("log parado");
    return;
  }
  if (fmt_csv) printHeader();
  log_ate_ms = millis() + (uint32_t)(seg * 1000.0f);
  log_ativo  = true;
}
void onKp(char* cmd)   { cmdScalar(&Kp, cmd); }
void onKi(char* cmd)   { cmdScalar(&Ki, cmd); }
void onKd(char* cmd)   { cmdScalar(&Kd, cmd); }
void onSet(char* cmd)  { cmdScalar(&setpoint_deg, cmd); resetPID(); }

// V<volts>: limite de tensão do motor (e da saída do PID). Máx. 8 V. Sem argumento só mostra.
// Cuidado: mais tensão = mais corrente e aquecimento no 2804. Suba aos poucos.
void onVolt(char* cmd) {
  if (cmd != NULL && cmd[0] != '\0') {
    VOLTAGE_LIMIT_MAX = constrain(atof(cmd), 0.0f, 8.0f);
    motor.voltage_limit = VOLTAGE_LIMIT_MAX;
  }
  SerialBT.printf("limite de tensao = %.1f V (max permitido 8 V)\n", VOLTAGE_LIMIT_MAX);
}

// Parâmetros do modo M2 (PID + pulsos). Sem argumento só mostram o valor atual.
void onBBV(char* cmd)    { cmdScalar(&bb_V, cmd);        bb_V = constrain(bb_V, 0.0f, 8.0f); }
void onBBDb(char* cmd)   { cmdScalar(&bb_db_deg, cmd);   bb_db_deg = constrain(bb_db_deg, 0.0f, 10.0f); }
void onBBDwell(char* cmd){ cmdScalar(&bb_dwell_ms, cmd); bb_dwell_ms = constrain(bb_dwell_ms, 5.0f, 1000.0f); }

// #<volts>: tensão de alimentação (6 a 16 V). Ex.: "#12.4". Sem argumento só mostra. W salva.
void onSupply(char* cmd) {
  if (cmd != NULL && cmd[0] != '\0') {
    supply_V = constrain((float)atof(cmd), 6.0f, 16.0f);
    driver.voltage_power_supply = supply_V;
    driver.voltage_limit = supply_V;
  }
  SerialBT.printf("tensao de alimentacao = %.2f V\n", supply_V);
}

// Tempos do kick (valem em M1 e M2). Sem argumento só mostram o valor atual.
void onKWait(char* cmd) { cmdScalar(&kick_wait_ms, cmd); kick_wait_ms = constrain(kick_wait_ms, 20.0f, 2000.0f); }
void onKGz(char* cmd)   { cmdScalar(&kick_gz_dps, cmd);  kick_gz_dps  = constrain(kick_gz_dps, 1.0f, 200.0f); }
void onKT1(char* cmd)   { cmdScalar(&kick_t1_ms, cmd);   kick_t1_ms   = constrain(kick_t1_ms, 20.0f, 500.0f); }

// ---- Parâmetros persistentes (NVS do ESP32) ----
// W salva os parâmetros abaixo; no próximo boot eles são carregados. Y apaga o que foi salvo.
// O modo (M0/M1/M2) NÃO é salvo: o pêndulo sempre liga em M1.
Preferences prefs;

void loadParams() {
  prefs.begin("pend", true);                       // somente leitura
  if (prefs.isKey("Kp")) {
    Kp = prefs.getFloat("Kp", Kp);   Ki = prefs.getFloat("Ki", Ki);   Kd = prefs.getFloat("Kd", Kd);
    setpoint_deg = prefs.getFloat("set", setpoint_deg);
    dir_motor = prefs.getFloat("dir", dir_motor) < 0 ? -1.0f : 1.0f;
    VOLTAGE_LIMIT_MAX = constrain(prefs.getFloat("vmax", VOLTAGE_LIMIT_MAX), 0.0f, 8.0f);
    I_MAX_V = constrain(prefs.getFloat("imax", I_MAX_V), 0.0f, 4.0f);
    supply_V = constrain(prefs.getFloat("vcc", supply_V), 6.0f, 16.0f);
    kick_V = constrain(prefs.getFloat("kick", kick_V), 0.0f, 8.0f);
    kick_wait_ms = constrain(prefs.getFloat("kw", kick_wait_ms), 20.0f, 2000.0f);
    kick_gz_dps  = constrain(prefs.getFloat("kg", kick_gz_dps), 1.0f, 200.0f);
    kick_t1_ms   = constrain(prefs.getFloat("kt1", kick_t1_ms), 20.0f, 500.0f);
    bb_V = constrain(prefs.getFloat("c", bb_V), 0.0f, 8.0f);
    bb_db_deg = constrain(prefs.getFloat("n", bb_db_deg), 0.0f, 10.0f);
    bb_dwell_ms = constrain(prefs.getFloat("o", bb_dwell_ms), 5.0f, 1000.0f);
    gyro_bias_z = prefs.getFloat("bias", gyro_bias_z);
    telem_hz = constrain(prefs.getFloat("thz", telem_hz), 1.0f, 50.0f);
    fmt_csv = prefs.getBool("csv", fmt_csv);
  }
  prefs.end();
}

bool paramsSalvos() {
  prefs.begin("pend", true);
  bool tem = prefs.isKey("Kp");
  prefs.end();
  return tem;
}

void onSave(char* cmd) {
  prefs.begin("pend", false);
  prefs.putFloat("Kp", Kp);  prefs.putFloat("Ki", Ki);  prefs.putFloat("Kd", Kd);
  prefs.putFloat("set", setpoint_deg);
  prefs.putFloat("dir", dir_motor);
  prefs.putFloat("vmax", VOLTAGE_LIMIT_MAX);
  prefs.putFloat("imax", I_MAX_V);
  prefs.putFloat("vcc", supply_V);
  prefs.putFloat("kick", kick_V);
  prefs.putFloat("kw", kick_wait_ms);
  prefs.putFloat("kg", kick_gz_dps);
  prefs.putFloat("kt1", kick_t1_ms);
  prefs.putFloat("c", bb_V);
  prefs.putFloat("n", bb_db_deg);
  prefs.putFloat("o", bb_dwell_ms);
  prefs.putFloat("bias", gyro_bias_z);
  prefs.putFloat("thz", telem_hz);
  prefs.putBool("csv", fmt_csv);
  prefs.end();
  SerialBT.println("parametros SALVOS na memoria (K I D S R V J X C N O bias T F). Serao carregados no boot.");
}

void onClearSaved(char* cmd) {
  prefs.begin("pend", false);
  prefs.clear();
  prefs.end();
  SerialBT.println("memoria apagada: no proximo boot volta aos valores padrao do codigo.");
}

// J<volts>: limite do termo integral (anti-windup). Padrão 1 V, máx. 4 V.
void onImax(char* cmd) {
  if (cmd != NULL && cmd[0] != '\0') I_MAX_V = constrain(atof(cmd), 0.0f, 4.0f);
  SerialBT.printf("I_MAX = %.2f V\n", I_MAX_V);
}

// X<volts>: liga o kick de partida com essa tensão (X0 desliga).
void onKick(char* cmd) {
  if (cmd != NULL && cmd[0] != '\0') kick_V = constrain(atof(cmd), 0.0f, 8.0f);
  if (kick_V > 0.01f) SerialBT.printf("kick LIGADO: %.1f V (limitado por V=%.1f)\n", kick_V, VOLTAGE_LIMIT_MAX);
  else                SerialBT.println("kick desligado");
}

// Z: usa o ângulo ATUAL da haste como setpoint (segure a haste exatamente no ponto de equilíbrio)
void onZero(char* cmd) {
  setpoint_deg = angle_est * RAD_TO_DEG;
  resetPID();
  SerialBT.printf("setpoint = %.2f deg (angulo atual)\n", setpoint_deg);
}

void setup() {
  Serial.begin(115200);   // só para debug via USB, se estiver plugado
  loadParams();   // carrega parâmetros salvos (comando W), se existirem
  reset_motivo = resetReasonStr();
  Serial.printf("Motivo do ultimo reset: %s\n", reset_motivo);
  SerialBT.begin("ESP32_Pendulo");
  delay(300);

  // Encoder AS5600 (I2C bus 0, pinos padrão 21/22)
  Wire.begin();
  sensor.init();
  motor.linkSensor(&sensor);

  // GY-521 (I2C bus 1, pinos 16/17)
  if (!mpuInit()) {
    SerialBT.println("AVISO: falha ao iniciar o MPU6050 (verifique fiação/endereço 0x68).");
  }

  // Driver
  driver.voltage_power_supply = supply_V;
  driver.voltage_limit = supply_V;
  driver.init();
  motor.linkDriver(&driver);

  // Controle por torque (tensão), sem sensor de corrente
  motor.voltage_limit = VOLTAGE_LIMIT_MAX;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.useMonitoring(SerialBT);

  motor.init();
  motor.initFOC();

  cmdAdd('A', onAmp,  "amplitude onda quadrada (V) [modo M0]");
  cmdAdd('P', onPer,  "periodo onda quadrada (ms) [modo M0]");
  cmdAdd('M', onModo, "modo: 0=onda quadrada, 1=PID");
  cmdAdd('K', onKp,   "Kp");
  cmdAdd('I', onKi,   "Ki");
  cmdAdd('D', onKd,   "Kd");
  cmdAdd('S', onSet,  "setpoint (graus)");
  cmdAdd('R', onInv,  "inverte sentido do torque");
  cmdAdd('L', onLog,  "pede log por N segundos (L0 para)");
  cmdAdd('F', onFmt,  "formato do log (F1=CSV, F0=texto)");
  cmdAdd('T', onTelem,"taxa do log (Hz)");
  cmdAdd('B', onBias, "calibra bias do giro (haste parada)");
  cmdAdd('Q', onStatus,"status/parametros atuais");
  cmdAdd('H', onHelp, "legenda do log e comandos");
  cmdAdd('Z', onZero, "setpoint = angulo atual");
  cmdAdd('V', onVolt, "limite de tensao (V, max 8)");
  cmdAdd('J', onImax, "limite do integrador (V)");
  cmdAdd('C', onBBV,    "M2: amplitude do pulso (V)");
  cmdAdd('N', onBBDb,   "M2: zona morta (graus)");
  cmdAdd('O', onBBDwell,"M2: tempo minimo entre reversoes (ms)");
  cmdAdd('X', onKick, "kick de partida (V, X0 desliga)");
  cmdAdd('#', onSupply, "tensao de alimentacao (V)");
  cmdAdd('G', onKWait, "kick: tempo parado antes de disparar (ms)");
  cmdAdd('U', onKGz,   "kick: limite de |gz| para considerar parado (graus/s)");
  cmdAdd('E', onKT1,   "kick: duracao do recuo (ms)");
  cmdAdd('W', onSave, "salva parametros na memoria");
  cmdAdd('Y', onClearSaved, "apaga parametros salvos");

  t_ultima_troca = millis();
}

void loop() {
  motor.loopFOC();

  // taxa real do loop (diagnóstico)
  static uint32_t loops = 0, t_loop = 0;
  loops++;
  if (millis() - t_loop >= 1000) {
    loop_hz = loops * 1000.0f / (millis() - t_loop);
    loops = 0; t_loop = millis();
  }

  MpuSample s = mpuRead();
  if (s.ok) updateAngle(s); else mpu_falhas++;

  float out_V = 0;

  if (modo < 0.5f) {
    // ---- Modo M0: teste em malha aberta (onda quadrada) ----
    if (millis() - t_ultima_troca > (unsigned long)periodo_ms) {
      sentido *= -1;
      t_ultima_troca = millis();
    }
    out_V = sentido * constrain(amplitude_V, 0, VOLTAGE_LIMIT_MAX);
    dbg_err_deg = 0; dbg_P = dbg_I = dbg_D = dbg_pre = 0; dbg_sat = false; dbg_fallen = false;
  } else if (modo > 1.5f) {
    // ---- Modo M2: PID + pulsos (amplitude vinda do PID, com mínimo = kick) ----
    // O PID calcula u = P+I+D (mesmos K, I, D, J, S, R do M1). O motor recebe um pulso:
    //   sentido   = sinal de u (com tempo mínimo O entre reversões -> cada reversão dá o pico de torque)
    //   amplitude = |u|, mas nunca menor que C (kick base, vence o atrito) e nunca maior que V
    //   zona morta: se |u| < Kp * N graus, a saída é zero
    float gyro_rate_rads = GYRO_SIGN * s.gz * DEG_TO_RAD;
    float u = computePID(angle_est, gyro_rate_rads);      // já inclui sentido R e limite V
    float mag  = fabsf(u);
    float dead = Kp * bb_db_deg * DEG_TO_RAD;
    float want = (mag <= dead) ? 0.0f : (u > 0 ? 1.0f : -1.0f);
    static float pm_dir = 0;
    static uint32_t pm_t0 = 0;
    uint32_t tn = millis();
    if (want != pm_dir && (tn - pm_t0) >= (uint32_t)bb_dwell_ms) { pm_dir = want; pm_t0 = tn; }
    float vmin = min(bb_V, VOLTAGE_LIMIT_MAX);
    float amp  = constrain(mag, vmin, VOLTAGE_LIMIT_MAX);
    out_V = pm_dir * amp;
  } else {
    // ---- Modo M1: balanceamento por PID ----
    float gyro_rate_rads = GYRO_SIGN * s.gz * DEG_TO_RAD;
    out_V = computePID(angle_est, gyro_rate_rads);
  }

  // ---- Kick de partida (comando X), vale para M1 E M2 ----
  // Se a haste fica parada longe do setpoint (ex.: encostada no limite), a roda já está na velocidade
  // máxima da tensão e não gera mais torque. O kick recua a roda (sentido oposto) e depois inverte
  // para o lado da subida, aproveitando o pico de torque da reversão.
  if (modo >= 0.5f && kick_V > 0.01f && !bias_cal) {
    static uint8_t kst = 0;                 // 0 = normal, 1 = recuo, 2 = pulso
    static uint32_t kt0 = 0, tstuck = 0;
    static float ksign = 1;                 // sentido da SUBIDA (mesmo sinal da saída do PID)
    uint32_t tn = millis();
    if (kst == 0) {
      bool travado = !dbg_fallen && fabsf(dbg_err_deg) > KICK_ERR_DEG && fabsf(s.gz) < kick_gz_dps;
      if (!travado) tstuck = 0;
      else if (tstuck == 0) tstuck = tn;
      else if (tn - tstuck > (uint32_t)kick_wait_ms && tn - kt0 > KICK_COOLDOWN_MS) {
        ksign = ((dbg_err_deg > 0) ? 1.0f : -1.0f) * dir_motor;
        kst = 1; kt0 = tn; tstuck = 0;
        SerialBT.println("KICK");
      }
    }
    if (kst == 1) {
      out_V = -ksign * kick_V;
      if (tn - kt0 > (uint32_t)kick_t1_ms) { kst = 2; kt0 = tn; }
    } else if (kst == 2) {
      out_V = ksign * kick_V;
      if (tn - kt0 > KICK_T2_MS) { kst = 0; kt0 = tn; resetPID(); }
    }
  }

  // Calibração do offset do giroscópio (comando B): motor desligado, haste parada
  if (bias_cal) {
    out_V = 0;
    if (s.ok) { bias_soma += s.gz + gyro_bias_z; bias_cnt++; }
    if ((int32_t)(millis() - bias_fim_ms) >= 0) {
      bias_cal = false;
      if (bias_cnt > 50) gyro_bias_z = (float)(bias_soma / bias_cnt);
      SerialBT.printf("bias gz = %.3f dps (%lu amostras)\n", gyro_bias_z, (unsigned long)bias_cnt);
      resetPID();
    }
  }

  motor.move(out_V);

  // Telemetria via Bluetooth (CSV), limitada a TELEM_HZ e SEM bloquear o controle:
  // uma única escrita por linha, e se o buffer BT estiver cheio a linha é descartada.
  static uint32_t t_telem = 0;
  uint32_t now_ms = millis();
  if (log_ativo && (int32_t)(now_ms - log_ate_ms) >= 0) {
    log_ativo = false;
    SerialBT.println("fim do log");
  }
  if (log_ativo && now_ms - t_telem >= (uint32_t)(1000.0f / telem_hz)) {
    t_telem = now_ms;
    // (availableForWrite() retorna 0 no BluetoothSerial, por isso não é usado aqui)
    if (SerialBT.hasClient()) {
      float ang_deg = angle_est * RAD_TO_DEG;
      float ang_acc = ANGLE_SIGN * atan2f(s.ax, s.ay) * RAD_TO_DEG;
      int   clip    = (fabsf(s.ax) > 1.99f || fabsf(s.ay) > 1.99f || fabsf(s.az) > 1.99f) ? 1 : 0;
      int   m       = (modo < 0.5f) ? 0 : (modo > 1.5f) ? 2 : 1;
      char buf[320];
      int n;
      if (fmt_csv) {
        // colunas: ver comando H (e o cabeçalho enviado no início do log)
        n = snprintf(buf, sizeof(buf),
                     "%lu,%d,%.2f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2f,%.3f,%.3f,%.3f,%d,%.0f,%lu\n",
                     (unsigned long)now_ms, m, ang_deg, ang_acc, s.gz, dbg_err_deg,
                     dbg_P, dbg_I, dbg_D, dbg_pre, out_V, (int)dbg_sat, (int)dbg_fallen,
                     motor.shaft_velocity, s.ax, s.ay, s.az, clip, loop_hz, (unsigned long)mpu_falhas);
      } else {
        n = snprintf(buf, sizeof(buf),
                     "t=%.2fs M%d | ang=%+.2f acc=%+.2f gz=%+.1f erro=%+.2f | P=%+.3f I=%+.3f D=%+.3f pid=%+.3f OUT=%+.3fV SAT=%d CAIDO=%d | roda=%+.2f rad/s | a=(%+.2f,%+.2f,%+.2f)g CLIP=%d | loop=%.0fHz falhas=%lu\n",
                     now_ms / 1000.0f, m, ang_deg, ang_acc, s.gz, dbg_err_deg,
                     dbg_P, dbg_I, dbg_D, dbg_pre, out_V, (int)dbg_sat, (int)dbg_fallen,
                     motor.shaft_velocity, s.ax, s.ay, s.az, clip, loop_hz, (unsigned long)mpu_falhas);
      }
      if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
      SerialBT.write((const uint8_t*)buf, n);
    }
  }

  cmdRun();
}