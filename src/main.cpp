/*
  Pêndulo invertido com roda de reação — FreeRTOS, modo desligado, PID/pulsos/LQI e auto-ajuste
  Motor:    Gimbal 2804 (12N14P -> 7 pares de polos)
  Encoder:  AS5600 (I2C bus 0 / Wire), no EIXO DO MOTOR/RODA
  IMU:      GY-521 / MPU6050 (I2C bus 1 / Wire1), fixado na HASTE
  Driver:   SimpleFOCmini (DRV8313, 3PWM)
  Placa:    ESP32 Doit DevKit V1
  Build:    PlatformIO (env esp32doit-devkit-v1 em platformio.ini)
  Telemetria e comandos: Bluetooth Classic (SPP), sem fio ao notebook

  ARQUITETURA (FreeRTOS, dois núcleos)
  --------------------------------------------------------------------
  Núcleo 1 | loop()      FOC: motor.loopFOC() + motor.move() o mais rápido possível (lê o AS5600).
                         Só executa com o motor LIGADO; senão dorme.
  Núcleo 0 | ctrlTask    (prioridade alta, 500 Hz) lê o IMU (Wire1), filtro, PID / M2 / M3 / kick,
                         proteções. Publica a tensão desejada em u_cmd.
  Núcleo 0 | comTask     (prioridade baixa) Bluetooth: comandos, log, economia de energia e
                         auto-ajuste. Se o Bluetooth travar, o controle NÃO é afetado.
  Cada barramento I2C é usado por UMA tarefa só (Wire = FOC, Wire1 = controle).

  ESTADOS
  --------------------------------------------------------------------
  STANDBY  (estado ao ligar a energia) motor DESLIGADO (driver em alta impedância), IMU lendo a 50 Hz.
           Dá para usar B, Z, L, Q etc. sem o motor energizado.
  ECONOMIA entra sozinha após $<s> segundos (padrão 120) sem comandos no STANDBY: IMU em sleep, CPU a
           80 MHz. QUALQUER comando recebido pelo Bluetooth acorda. (O Bluetooth fica ativo.)
  ATIVO    motor ligado e controlando. O primeiro '@' alinha o encoder (a roda se mexe um pouco).

  MODOS (valem no estado ATIVO)
  --------------------------------------------------------------------
  M0 = teste em malha aberta (onda quadrada de tensão)
  M1 = balanceamento por PID (padrão), com kick de partida (X)
  M2 = PID + pulsos (amplitude do pulso vinda do PID, mínimo C, máximo V)
  M3 = balanceamento por LQI (realimentação de estados + integrador na velocidade da roda):
         u = -(k1*theta + k2*dtheta + k3*w_roda + k4*z),  z = integral de (0 - w_roda)
       Ganhos calculados no lqi_ganhos.py (modelo do Fusion). Ajuste fino: $1..$10. Saída em PULSOS (reversões de sentido, como o M2) e kick adaptativo (X) quando trava.

  COMANDOS BLUETOOTH (digite no app de terminal serial BT, Enter após):
    !         DESLIGA o motor (vai para STANDBY).  !1 = desliga e entra em ECONOMIA já.
    @         LIGA o motor no modo atual.          @0 a @3 = liga já no modo escolhido.
    ^<s>      segundos sem comando até entrar em economia (padrão 120; 0 = nunca)
    $         ganhos do LQI (M3): "$" mostra | $1 k1 | $2 k2 | $3 k3 | $4 k4 | $5 limite integral (V) | $6 zona morta (V) | $7 kick roda (rad/s) | $8 pulsos 1/0 | $9 auto-trim | $10 dwell (ms)
    %<s>      desliga SOZINHO se ficar longe do setpoint (> 4 graus) por mais de <s> s (padrão 20; 0 = nunca)
    ~         AUTO-AJUSTE (twiddle) com o motor ligado: M1 = Kp,Kd,Ki | M2 = Kp,Kd,Ki,C | M3 = escalas dos ganhos k1..k4.  ~<s> = janela de <s> s por teste.
              ~0 cancela (mantém os melhores valores achados até ali). Depois use W para salvar.
    A<valor>  amplitude da onda quadrada no modo M0 (V)
    P<valor>  período da onda quadrada no modo M0 (ms)
    M<0,1,2,3> troca de modo
    K<valor>  Kp do PID
    I<valor>  Ki do PID
    D<valor>  Kd do PID
    S<valor>  setpoint do PID, em graus (0 = vertical)
    R         inverte o sentido do torque do motor (alterna +1/-1)
    L<seg>    pede o log por <seg> segundos (ex.: L10). "L" sozinho = 5 s, L0 para.
    F         formato do log: F1 = CSV (planilha), F0 = texto legível, "F" alterna
    T<hz>     taxa do log (1 a 50 Hz)
    B         calibra o offset do giroscópio (haste PARADA, 2 s). Só com o motor desligado.
    Q         mostra parâmetros atuais, taxas e motivo do último reset
    H         legenda de cada coluna do log + lista de comandos
    J<volts>  limite do termo integral (padrão 1, máx. 4)
    C<volts>  M2: amplitude mínima do pulso | N<graus> zona morta | O<ms> tempo mínimo entre reversões
    X<volts>  kick de partida (vale em M1, M2 e M3; padrão 2 V; X0 desliga)
    G<ms>     kick: tempo parado antes de disparar (padrão 120) | U<dps> |gz| que conta como parado (15) | E<ms> recuo (120)
    #<volts>  tensão de alimentação real (6 a 16 V), ex.: #12.4 — meça com multímetro; W salva
    W         SALVA os parâmetros na memória (carregados a cada boot) | Y apaga o que foi salvo
    V<volts>  limite de tensão do motor/PID (padrão 4, máx. 8). Suba aos poucos.
    Z         setpoint = ângulo atual (segure a haste no ponto de equilíbrio antes)

  AVISO: BluetoothSerial (SPP clássico) conecta com Android normalmente
  via apps como "Serial Bluetooth Terminal". iPhone/iOS não suporta SPP.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "BluetoothSerial.h"
#include <esp_system.h>   // esp_reset_reason()
#include <Preferences.h>  // memória não volátil (NVS) para salvar parâmetros
#include <string.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth não habilitado no build. Rode `idf.py menuconfig`/verifique o framework ESP32 Arduino.
#endif

// ---------- Pinos do driver (SimpleFOCmini) ----------
// Evita strapping pins (0,2,12,15), flash SPI (6-11) e pinos só-entrada (34-39).
#define PIN_PWM_A   25
#define PIN_PWM_B   26
#define PIN_PWM_C   27
#define PIN_ENABLE  33

// ---------- I2C bus 0: AS5600 no eixo do motor/roda (Wire, pinos padrão SDA=21, SCL=22) ----------
// ---------- I2C bus 1: GY-521/MPU6050 na haste (Wire1) ----------
#define MPU_SDA 16
#define MPU_SCL 17
#define MPU_ADDR 0x68

// ---------- Tarefas FreeRTOS ----------
#define CTRL_PERIOD_MS     2     // período do controle no estado ATIVO (2 ms = 500 Hz). Tente 1 para 1 kHz.
#define STANDBY_PERIOD_MS  20    // leitura do IMU em STANDBY (50 Hz)
#define ECO_PERIOD_MS      100   // período das tarefas em ECONOMIA
#define CTRL_PRIO          6     // prioridade da tarefa de controle (núcleo 0)
#define COM_PRIO           2     // prioridade da tarefa de comunicação (núcleo 0)
#define CTRL_STACK         6144
#define COM_STACK          8192
#define CPU_MHZ_NORMAL     240
#define CPU_MHZ_ECO        80    // o Bluetooth exige no mínimo 80 MHz

// Alimentação
#define SUPPLY_VOLTAGE 13.70f  // tensão da fonte do driver (dentro de 7.4-16V do motor)
float VOLTAGE_LIMIT_MAX = 4.0f;   // ajustável em tempo real pelo comando V (máx. 8 V)

// ---------- Kick de partida (comando X) ----------
// Dispara quando a haste fica parada (|gz| < U) com erro grande (> KICK_ERR_DEG) por G ms. Recuo = E ms, pulso = KICK_T2_MS.
#define KICK_ERR_DEG     3.0f
#define KICK_T2_MS       200
#define KICK_COOLDOWN_MS 250
// Auto-desligar (comando %): considera "longe" se |erro| > PERDA_ERR_DEG
#define PERDA_ERR_DEG    4.0f
#define TELEM_HZ 20   // taxa do log via Bluetooth (Hz). Suba para 50 só se o app aguentar.

// ---------- Hardware: motor/driver/encoder ----------
BLDCMotor motor = BLDCMotor(7);                     // 2804 12N14P -> 7 pares de polos
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_A, PIN_PWM_B, PIN_PWM_C, PIN_ENABLE);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BluetoothSerial SerialBT;

// ============================================================
//  Estado compartilhado entre as tarefas
//  (floats/ints de 32 bits são lidos/escritos de forma atômica no ESP32;
//   estruturas maiores são copiadas dentro de seção crítica)
// ============================================================
enum : uint8_t { EST_ECO = 0, EST_STANDBY = 1, EST_ATIVO = 2 };
volatile uint8_t  estado    = EST_STANDBY;   // começa DESLIGADO (motor sem energia)
volatile float    wheel_vel = 0;             // velocidade da roda (rad/s), escrita pelo FOC, lida pelo controle
volatile float    u_cmd     = 0;             // tensão desejada (V): escrita pelo controle, lida pelo FOC
volatile bool     motor_on  = false;         // true = driver habilitado (só o loop() FOC desliga)
volatile uint32_t foc_count = 0;             // voltas do loop FOC (para medir a taxa)
bool foc_ok = false;                         // motor.init()/initFOC() já executados (feito no 1º '@')

volatile bool ctrl_reset_req    = false;     // pede ao controle: zerar PID/estados
volatile bool angle_resync_req  = false;     // pede ao controle: ressincronizar o ângulo com o acelerômetro
volatile bool mpu_sleep_req     = false;
volatile bool mpu_wake_req      = false;
volatile bool mpu_awake         = true;
volatile uint8_t auto_off_reason = 0;        // 1 = longe do setpoint por muito tempo, 2 = falha do MPU
volatile uint32_t kick_evt      = 0;         // conta disparos do kick
volatile bool bias_msg          = false;

// ============================================================
//  Auto-ajuste: busca por coordenadas ("twiddle") em escala multiplicativa.
//  Lógica pura (sem Arduino) para poder ser testada no PC.
// ============================================================
// TWIDDLE-BEGIN
struct Twiddle {
  static const int MAXP = 4;
  int   n;
  float best[MAXP], cur[MAXP], dp[MAXP], lo[MAXP], hi[MAXP];
  float best_cost;
  int   idx, phase, sweeps, trials;     // phase: 1 = testa para cima, 2 = testa para baixo
  bool  done, have_base;
  float margin, grow, shrink, dp_min, dp_max;
  int   max_sweeps;

  Twiddle() : n(0), best_cost(0), idx(0), phase(1), sweeps(0), trials(0), done(true), have_base(false),
              margin(0.03f), grow(1.25f), shrink(0.6f), dp_min(0.05f), dp_max(0.9f), max_sweeps(8) {
    for (int i = 0; i < MAXP; i++) best[i] = cur[i] = dp[i] = lo[i] = hi[i] = 0;
  }

  void begin(int np, const float* init, const float* lo_, const float* hi_, float dp0) {
    n = (np > MAXP) ? MAXP : np;
    for (int i = 0; i < n; i++) { best[i] = cur[i] = init[i]; lo[i] = lo_[i]; hi[i] = hi_[i]; dp[i] = dp0; }
    best_cost = 0; idx = 0; phase = 1; sweeps = 0; trials = 0;
    have_base = false; done = (n == 0);
  }

  // Próximo conjunto de parâmetros a testar (o primeiro é o ponto de partida, para medir o custo base).
  // Depois de chamar, confira 'done': se true, não há mais testes e o resultado está em best[].
  const float* trial() {
    if (done) return best;
    if (!have_base) { memcpy(cur, best, sizeof(cur)); return cur; }
    for (int guard = 0; guard < 4 * MAXP + 4 && !done; guard++) {
      memcpy(cur, best, sizeof(cur));
      float f = (phase == 1) ? (1.0f + dp[idx]) : (1.0f - dp[idx]);
      float v = best[idx] * f;
      cur[idx] = (v < lo[idx]) ? lo[idx] : ((v > hi[idx]) ? hi[idx] : v);
      float dif = cur[idx] - best[idx]; if (dif < 0) dif = -dif;
      float ref = best[idx] < 0 ? -best[idx] : best[idx];
      if (dif > 1e-4f * ref + 1e-6f) return cur;     // candidato diferente do melhor
      rejectCurrent();                               // preso no limite: descarta sem medir
    }
    memcpy(cur, best, sizeof(cur));
    return cur;
  }

  // Informe o custo medido para o último conjunto devolvido por trial().
  void report(float cost) {
    trials++;
    if (!have_base) { best_cost = cost; have_base = true; phase = 1; idx = 0; return; }
    if (cost < best_cost * (1.0f - margin)) {
      memcpy(best, cur, sizeof(best));
      best_cost = cost;
      dp[idx] *= grow; if (dp[idx] > dp_max) dp[idx] = dp_max;
      advance();
    } else {
      rejectCurrent();
    }
  }

  void rejectCurrent() {
    if (phase == 1) { phase = 2; }
    else { dp[idx] *= shrink; advance(); }
  }

  void advance() {
    phase = 1; idx++;
    if (idx >= n) {
      idx = 0; sweeps++;
      float m = 0; for (int i = 0; i < n; i++) if (dp[i] > m) m = dp[i];
      if (m < dp_min || sweeps >= max_sweeps) done = true;
    }
  }
};
// TWIDDLE-END

// ============================================================
//  Driver mínimo do MPU6050, em Wire1 (bus I2C separado) — usado SÓ pela tarefa de controle
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

bool mpuConfigure() {
  bool ok = true;
  ok &= mpuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00);     // força ±2g
  ok &= mpuWriteReg(MPU_REG_CONFIG, MPU_DLPF_CFG);   // filtro digital (ver MPU_DLPF_CFG)
  ok &= mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);      // força ±250°/s
  return ok;
}

bool mpuInit() {                                     // chamado no setup(), antes das tarefas
  Wire1.begin(MPU_SDA, MPU_SCL, 400000);
  bool ok = mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00);   // sai do modo sleep
  delay(50);
  ok &= mpuConfigure();
  return ok;
}

void mpuSleep() { mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x40); }   // bit SLEEP: ~5 uA
void mpuWake() {
  mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x00);
  vTaskDelay(pdMS_TO_TICKS(50));
  mpuConfigure();
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

volatile float angle_est = 0;     // rad, ângulo filtrado da haste (escrito só pelo controle)
unsigned long t_prev_imu = 0;
const float ALPHA_COMP = 0.98f;   // peso do giroscópio no filtro complementar

void updateAngle(const MpuSample &s) {
  unsigned long now = micros();
  float dt = (t_prev_imu == 0) ? 0.001f : (now - t_prev_imu) * 1e-6f;
  t_prev_imu = now;
  if (dt > 0.1f) dt = 0.1f;

  float angle_accel    = ANGLE_SIGN * atan2f(s.ax, s.ay);     // rad, só acelerômetro
  float gyro_rate_rads = GYRO_SIGN * s.gz * DEG_TO_RAD;       // rad/s

  // Giro para a dinâmica rápida, acelerômetro para corrigir o desvio (drift) do giro
  angle_est = ALPHA_COMP * (angle_est + gyro_rate_rads * dt)
            + (1.0f - ALPHA_COMP) * angle_accel;
}

// ============================================================
//  Parâmetros (ajustáveis por Bluetooth) e variáveis de depuração
// ============================================================
float Kp = 3.0f, Ki = 2.0f, Kd = 0.18f;   // ponto de partida — precisa ser ajustado na bancada
float setpoint_deg = 0.0f;                 // 0° = vertical
float pid_integral = 0;                    // termo integral já em Volts (Ki incluso)
unsigned long t_prev_pid = 0;
const float SAFETY_ANGLE_DEG = 70.0f;      // acima disso, considera "caído" e desliga a saída
float I_MAX_V = 1.0f;                      // limite do termo integral (anti-windup), em V — comando J
float dir_motor = 1.0f;                    // +1 / -1: sentido do torque (comando R inverte)
float modo = 1.0f;                         // 0 = M0 | 1 = M1 PID | 2 = M2 PID+pulsos | 3 = M3 LQI
int modoIdx(float m) { return (m < 0.5f) ? 0 : (m < 1.5f ? 1 : (m < 2.5f ? 2 : 3)); }

// ---- Variáveis de depuração (preenchidas pelo PID, lidas pelo log) ----
float dbg_err_deg = 0;      // erro = setpoint - ângulo (graus)
float dbg_P = 0, dbg_I = 0, dbg_D = 0;   // contribuição de cada termo (V)
float dbg_pre = 0;          // soma P+I+D ANTES de limitar (V)
bool  dbg_sat = false;      // true se a saída foi cortada pelo limite de tensão
bool  dbg_fallen = false;   // true se |ângulo| > SAFETY_ANGLE_DEG (PID desligado)
volatile float ctrl_hz = 0; // voltas da tarefa de controle por segundo
volatile float foc_hz  = 0; // voltas do loop FOC por segundo
volatile uint32_t mpu_falhas = 0;
uint16_t mpu_fail_run = 0;
bool  fmt_csv = true;       // formato do log: true = CSV, false = texto legível
float telem_hz = TELEM_HZ;  // taxa do log (Hz)
const char* reset_motivo = "?";
uint32_t log_ate_ms = 0;    // fim do log pedido (comando L)
bool     log_ativo  = false;

// ---- Modo M2: PID + pulsos (amplitude do PID, mínimo = kick) ----
float bb_V = 2.0f;          // amplitude mínima do pulso (V)                comando C
float bb_db_deg = 0.8f;     // zona morta (graus)                           comando N
float bb_dwell_ms = 60.0f;  // tempo mínimo entre reversões (ms)            comando O

// ---- Modo M3: LQI ----
// Ganhos calculados (lqi_ganhos.py) com o CAD novo (FreeCAD, 23/11/2026): massa 279,17 g | CM a 118,85 mm do pivô |
// I_pivô ≈ 4.930.000 g·mm² e I_roda ≈ 343.000 g·mm² (ESTIMADOS: troque pelos valores do CAD) | motor 2804 220 KV, 2,3 ohm de fase.
// Limites (Bryson): ângulo 2°, haste 1 rad/s, roda 60 rad/s, integral 20 rad, tensão 4 V.
// Recalcule com lqi_ganhos.py se o pêndulo mudar fisicamente. Na bancada, ajuste pelo comando $.
float lqi_k1 = 154.69f;   // V/rad        ângulo da haste (≈ 2,70 V/grau)
float lqi_k2 = 13.189f;   // V/(rad/s)    velocidade da haste (giroscópio)
float lqi_k3 = -0.2969f;  // V/(rad/s)    velocidade da roda (encoder) — negativo é normal
float lqi_k4 = 0.200f;    // V/rad        integral da velocidade da roda
float lqi_zmax_V = 2.0f;  // limite do termo integral k4*z (anti-windup), em V
// M3 em modo PULSOS (padrão): o sentido vem do sinal do LQI, a amplitude é |u| limitada entre C e V, com tempo mínimo O entre
// reversões. Assim a roda gira pouco e trabalha em reversões (pico de torque), como no M2. $8 0 = LQI contínuo.
float lqi_dead_V = 0.3f;  // zona morta: se |u| < isto (V), a saída é zero            comando $6
float lqi_wk     = 30.0f; // kick do M3: o recuo dura até a roda atingir esta velocidade (rad/s) comando $7
float lqi_pulse  = 1.0f;  // 1 = pulsos (padrão) | 0 = contínuo                       comando $8
float lqi_dwell_ms = 30.0f; // tempo mínimo entre reversões no M3 (o LQI é rápido: 60 ms é lento demais) comando $10
// Auto-trim do setpoint: se o setpoint estiver errado, a roda precisa de torque constante e acelera sem parar ("rotação simples").
// Aqui o setpoint é deslocado devagar na direção que zera a velocidade da roda: d(trim)/dt = lqi_trim * w.
float lqi_trim   = 0.002f; // ganho (adimensional). 0 = desligado                         comando $9
float lqi_sp_trim = 0;     // deslocamento aprendido do setpoint (rad), limitado a ±6 graus
float lqi_z = 0;          // estado do integrador: z = integral de (0 - w_roda), em rad
unsigned long t_prev_lqi = 0;
float dbg_W = 0;          // contribuição do termo da roda (-k3*w), em V (log)

// Tensão de alimentação usada pelo driver para converter volts em PWM (comando #).
float supply_V = SUPPLY_VOLTAGE;

// Tempos do kick (ajustáveis)
float kick_wait_ms = 120.0f;   // tempo parado antes de disparar o kick (ms)      comando G
float kick_gz_dps  = 15.0f;    // "parado" = |gz| abaixo disso (graus/s)          comando U
float kick_t1_ms   = 120.0f;   // duração do recuo da roda (ms)                   comando E
float kick_V       = 2.0f;     // tensão do kick (V). 0 = desligado               comando X

// Energia / segurança / auto-ajuste
float eco_s     = 120.0f;      // segundos sem comando em STANDBY até a ECONOMIA (0 = nunca)   comando $
float perda_s   = 20.0f;       // desliga sozinho se ficar longe do setpoint por mais disso     comando %
float tune_win_s = 12.0f;      // duração de cada teste do auto-ajuste (s)                      comando ~

// calibração do offset do giroscópio (comando B)
bool bias_cal = false; double bias_soma = 0; uint32_t bias_cnt = 0, bias_fim_ms = 0;

// Estados internos do controle (só a tarefa de controle mexe neles)
float    pm_dir = 0;
uint32_t pm_t0 = 0;
uint8_t  kst = 0;                 // kick: 0 = normal, 1 = recuo, 2 = pulso
uint32_t kt0 = 0, tstuck = 0;
float    ksign = 1;               // sentido da SUBIDA do kick
uint32_t perda_t0 = 0;
unsigned long t_ultima_troca = 0;
int sentido = 1;                  // M0

// ---- Métricas para o auto-ajuste (acumuladas pelo controle, lidas e zeradas pela comunicação) ----
struct Metrics { double err2; double u2; uint32_t n; uint32_t kicks; uint32_t fallen_n; };
Metrics met = {0, 0, 0, 0, 0};
portMUX_TYPE metMux = portMUX_INITIALIZER_UNLOCKED;

void metricsAdd(float err_deg, float u, bool fallen) {
  portENTER_CRITICAL(&metMux);
  met.err2 += (double)err_deg * err_deg;
  met.u2   += (double)u * u;
  met.n++;
  if (fallen) met.fallen_n++;
  portEXIT_CRITICAL(&metMux);
}
void metricsKick() { portENTER_CRITICAL(&metMux); met.kicks++; portEXIT_CRITICAL(&metMux); }
void metricsTake(Metrics* out) {
  portENTER_CRITICAL(&metMux);
  *out = met;
  met.err2 = 0; met.u2 = 0; met.n = 0; met.kicks = 0; met.fallen_n = 0;
  portEXIT_CRITICAL(&metMux);
}

// Custo do auto-ajuste: erro quadrático médio + esforço + cada kick (= bateu no limite) + tempo caído
#define TUNE_W_U     0.5f
#define TUNE_W_KICK  8.0f
#define TUNE_W_FALL  400.0f
float tuneCost(const Metrics& m) {
  if (m.n < 20) return 1e9f;
  double n = (double)m.n;
  return (float)(m.err2 / n + TUNE_W_U * (m.u2 / n) + TUNE_W_KICK * m.kicks + TUNE_W_FALL * (m.fallen_n / n));
}

// ---- Telemetria (cópia atômica do último ciclo de controle) ----
struct Telem {
  uint32_t t_ms; int modo; float ang, ang_acc, gz, err, P, I, D, pre, out; int sat, fallen;
  float ax, ay, az; int clip; float W; float trim;
};
Telem telem = {};
portMUX_TYPE telMux = portMUX_INITIALIZER_UNLOCKED;
void telemGet(Telem* out) { portENTER_CRITICAL(&telMux); *out = telem; portEXIT_CRITICAL(&telMux); }

// ============================================================
//  PID de balanceamento (só a tarefa de controle chama)
// ============================================================
void resetPID() {
  pid_integral = 0;
  t_prev_pid = 0;
  lqi_z = 0;          // o LQI também zera junto (troca de modo, R, S, Z, B, fim do kick...)
  t_prev_lqi = 0;
}

void ctrlResetStates() {
  resetPID();
  kst = 0; kt0 = 0; tstuck = 0; pm_dir = 0; pm_t0 = 0; perda_t0 = 0;
  lqi_sp_trim = 0;       // o trim aprendido é zerado junto (troca de modo, S, Z, R, ligar...)
}
void ctrlReset() { ctrl_reset_req = true; }     // pedido feito por qualquer tarefa

float computePID(float angle_rad, float gyro_rate_rads) {
  unsigned long now = micros();
  float dt = (t_prev_pid == 0) ? 0.005f : (now - t_prev_pid) * 1e-6f;
  t_prev_pid = now;
  dt = constrain(dt, 1e-4f, 0.05f);        // protege contra dt zero ou saltos grandes
  dbg_W = 0;

  if (fabsf(angle_rad) * RAD_TO_DEG > SAFETY_ANGLE_DEG) {
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

  float upk = Kp * error;
  dbg_P = upk;

  // I com anti-windup: limitado a ±I_MAX_V e desfeito se a saída saturar a favor do erro
  pid_integral += Ki * error * dt;
  pid_integral = constrain(pid_integral, -I_MAX_V, I_MAX_V);

  // D: usa a taxa do giroscópio (mais limpa que derivar o ângulo). d(erro)/dt = -taxa
  float udk = Kd * (-gyro_rate_rads);
  dbg_D = udk;

  float output = upk + pid_integral + udk;
  float sat = constrain(output, -VOLTAGE_LIMIT_MAX, VOLTAGE_LIMIT_MAX);
  if (sat != output && (output * error) > 0) {
    pid_integral -= Ki * error * dt;
  }

  dbg_I   = pid_integral;
  dbg_pre = output;
  dbg_sat = (sat != output);
  return dir_motor * sat;                  // dir_motor inverte o sentido do torque
}

// ============================================================
//  LQI de balanceamento (modo M3) — só a tarefa de controle chama
// ============================================================
// Convenção (a mesma do lqi_ganhos.py):
//   u > 0 acelera a roda no sentido positivo e empurra a haste para +theta.
//   O motor recebe dir_motor * u, e a velocidade da roda é lida como dir_motor * wheel_vel,
//   então o comando R inverte as duas coisas juntas e o modelo continua consistente.
float computeLQI(float angle_rad, float gyro_rate_rads, float wheel_vel_rads) {
  unsigned long now = micros();
  float dt = (t_prev_lqi == 0) ? 0.002f : (now - t_prev_lqi) * 1e-6f;
  t_prev_lqi = now;
  dt = constrain(dt, 1e-4f, 0.05f);

  float theta  = angle_rad - (setpoint_deg * DEG_TO_RAD + lqi_sp_trim);   // desvio em relação ao setpoint + trim (rad)
  float dtheta = gyro_rate_rads;                          // rad/s
  float w      = dir_motor * wheel_vel_rads;              // rad/s, no sentido de u

  dbg_err_deg = -theta * RAD_TO_DEG;   // mesmo significado do PID: setpoint - ângulo

  if (fabsf(angle_rad) * RAD_TO_DEG > SAFETY_ANGLE_DEG) {   // haste caída: desliga e zera o integrador
    lqi_z = 0;
    dbg_fallen = true;
    dbg_P = dbg_I = dbg_D = dbg_W = dbg_pre = 0;
    dbg_sat = false;
    return 0;
  }
  dbg_fallen = false;

  // Auto-trim: roda girando para + => o setpoint estava abaixo do equilíbrio real; sobe-o (e vice-versa)
  if (lqi_trim > 0) lqi_sp_trim = constrain(lqi_sp_trim + lqi_trim * w * dt, -0.105f, 0.105f);

  // Integrador: z = integral de (referência - saída), referência = roda parada
  float dz = (0.0f - w) * dt;
  lqi_z += dz;
  if (fabsf(lqi_k4) > 1e-6f) {                          // limita o termo integral a ±lqi_zmax_V volts
    float zlim = lqi_zmax_V / fabsf(lqi_k4);
    lqi_z = constrain(lqi_z, -zlim, zlim);
  }

  // Lei de controle u = -K x, termo a termo (para o log)
  float uP = -lqi_k1 * theta;     // ângulo
  float uD = -lqi_k2 * dtheta;    // velocidade da haste
  float uW = -lqi_k3 * w;         // velocidade da roda
  float uI = -lqi_k4 * lqi_z;     // integral
  float u  = uP + uD + uW + uI;

  float sat = constrain(u, -VOLTAGE_LIMIT_MAX, VOLTAGE_LIMIT_MAX);
  if (sat != u && (-lqi_k4 * dz) * u > 0) lqi_z -= dz;   // anti-windup: desfaz se saturou a favor do integrador

  dbg_P = uP; dbg_D = uD; dbg_W = uW; dbg_I = -lqi_k4 * lqi_z;
  dbg_pre = u;
  dbg_sat = (sat != u);
  return dir_motor * sat;
}

// M0: teste em malha aberta (onda quadrada)
float amplitude_V = 1.0f;
float periodo_ms  = 700;

// ============================================================
//  TAREFA DE CONTROLE (núcleo 0): IMU + filtro + PID/M2/M3/kick + proteções
// ============================================================
void ctrlStep() {
  if (mpu_sleep_req) { mpuSleep(); mpu_awake = false; mpu_sleep_req = false; }
  if (mpu_wake_req)  { mpuWake();  mpu_awake = true;  mpu_wake_req  = false; angle_resync_req = true; }
  if (!mpu_awake) { u_cmd = 0; return; }

  if (ctrl_reset_req) { ctrl_reset_req = false; ctrlResetStates(); }

  uint32_t now = millis();
  MpuSample s = mpuRead();
  if (s.ok) {
    mpu_fail_run = 0;
    if (angle_resync_req) {
      angle_est = ANGLE_SIGN * atan2f(s.ax, s.ay);
      t_prev_imu = 0;
      angle_resync_req = false;
    }
    updateAngle(s);
  } else {
    mpu_falhas = mpu_falhas + 1;
    if (mpu_fail_run < 60000) mpu_fail_run++;
  }

  // taxa real do controle (diagnóstico)
  static uint32_t cn = 0, ct0 = 0;
  cn++;
  if (now - ct0 >= 1000) { ctrl_hz = cn * 1000.0f / (now - ct0); cn = 0; ct0 = now; }

  // Calibração do offset do giroscópio (comando B): só com o motor desligado
  if (bias_cal) {
    if (s.ok) { bias_soma += s.gz + gyro_bias_z; bias_cnt++; }
    if ((int32_t)(now - bias_fim_ms) >= 0) {
      bias_cal = false;
      if (bias_cnt > 25) gyro_bias_z = (float)(bias_soma / bias_cnt);
      bias_msg = true;
      ctrl_reset_req = true;
    }
  }

  float out_V = 0;
  float md = modo;
  if (estado == EST_ATIVO && !bias_cal) {
    float gyro_rate_rads = GYRO_SIGN * s.gz * DEG_TO_RAD;

    if (md < 0.5f) {
      // ---- Modo M0: teste em malha aberta (onda quadrada) ----
      if (now - t_ultima_troca > (unsigned long)periodo_ms) { sentido *= -1; t_ultima_troca = now; }
      out_V = sentido * constrain(amplitude_V, 0.0f, VOLTAGE_LIMIT_MAX);
      dbg_err_deg = 0; dbg_P = dbg_I = dbg_D = dbg_W = dbg_pre = 0; dbg_sat = false; dbg_fallen = false;
    } else if (md > 2.5f) {
      // ---- Modo M3: LQI (ângulo, velocidade da haste, velocidade da roda, integral da roda) ----
      out_V = computeLQI(angle_est, gyro_rate_rads, wheel_vel);   // já inclui sentido R e limite V
      if (lqi_pulse > 0.5f) {
        float mag  = fabsf(out_V);
        float want = (mag <= lqi_dead_V) ? 0.0f : (out_V > 0 ? 1.0f : -1.0f);
        if (want != pm_dir && (now - pm_t0) >= (uint32_t)lqi_dwell_ms) { pm_dir = want; pm_t0 = now; }
        float vmin = min(bb_V, VOLTAGE_LIMIT_MAX);
        out_V = pm_dir * constrain(mag, vmin, VOLTAGE_LIMIT_MAX);
      }
    } else if (md > 1.5f) {
      // ---- Modo M2: PID + pulsos (amplitude vinda do PID, com mínimo = kick) ----
      //   sentido   = sinal de u (tempo mínimo O entre reversões -> cada reversão dá o pico de torque)
      //   amplitude = |u|, mas nunca menor que C e nunca maior que V
      //   zona morta: se |u| < Kp * N graus, a saída é zero
      float u = computePID(angle_est, gyro_rate_rads);      // já inclui sentido R e limite V
      float mag  = fabsf(u);
      float dead = Kp * bb_db_deg * DEG_TO_RAD;
      float want = (mag <= dead) ? 0.0f : (u > 0 ? 1.0f : -1.0f);
      if (want != pm_dir && (now - pm_t0) >= (uint32_t)bb_dwell_ms) { pm_dir = want; pm_t0 = now; }
      float vmin = min(bb_V, VOLTAGE_LIMIT_MAX);
      float amp  = constrain(mag, vmin, VOLTAGE_LIMIT_MAX);
      out_V = pm_dir * amp;
    } else {
      // ---- Modo M1: balanceamento por PID ----
      out_V = computePID(angle_est, gyro_rate_rads);
    }

    // ---- Kick de partida (comando X), vale para M1, M2 e M3 ----
    // No M3 o recuo também ajuda a descarregar a roda saturada (o termo -k3*w fica grande e prende a saída no limite).
    // Se a haste fica parada longe do setpoint (ex.: encostada no limite), a roda já está na velocidade
    // máxima da tensão e não gera mais torque. O kick recua a roda e depois inverte para o lado da subida.
    if (md >= 0.5f && kick_V > 0.01f) {
      if (kst == 0) {
        bool travado = !dbg_fallen && fabsf(dbg_err_deg) > KICK_ERR_DEG && fabsf(s.gz) < kick_gz_dps;
        if (!travado) tstuck = 0;
        else if (tstuck == 0) tstuck = now;
        else if (now - tstuck > (uint32_t)kick_wait_ms && now - kt0 > KICK_COOLDOWN_MS) {
          ksign = ((dbg_err_deg > 0) ? 1.0f : -1.0f) * dir_motor;
          kst = 1; kt0 = now; tstuck = 0;
          kick_evt = kick_evt + 1;
          metricsKick();
        }
      }
      if (kst == 1) {
        out_V = -ksign * kick_V;
        if (md > 2.5f) {
          // M3: o recuo dura até a roda atingir lqi_wk (rad/s) no sentido do recuo (ou 700 ms): o pico de torque da reversão
          // vem da força contra-eletromotriz, então só funciona se a roda realmente estiver girando para o lado oposto.
          if ((-ksign) * wheel_vel >= lqi_wk || now - kt0 > 700) { kst = 2; kt0 = now; }
        } else if (now - kt0 > (uint32_t)kick_t1_ms) { kst = 2; kt0 = now; }
      } else if (kst == 2) {
        out_V = ksign * kick_V;
        if (now - kt0 > KICK_T2_MS) { kst = 0; kt0 = now; resetPID(); }
      }
    }

    // ---- Proteções: desliga sozinho se ficar longe do setpoint por muito tempo ou se o MPU falhar ----
    if (md >= 0.5f && perda_s > 0.5f) {
      if (dbg_fallen || fabsf(dbg_err_deg) > PERDA_ERR_DEG) {
        if (perda_t0 == 0) perda_t0 = now;
        else if (now - perda_t0 > (uint32_t)(perda_s * 1000.0f)) auto_off_reason = 1;
      } else {
        perda_t0 = 0;
      }
    }
    if (mpu_fail_run > 50) auto_off_reason = 2;
    if (auto_off_reason) out_V = 0;

    if (md >= 0.5f) metricsAdd(dbg_err_deg, out_V, dbg_fallen);
  }
  u_cmd = out_V;

  // Publica a telemetria deste ciclo
  Telem t;
  t.t_ms = now; t.modo = modoIdx(md);
  t.ang = angle_est * RAD_TO_DEG; t.ang_acc = ANGLE_SIGN * atan2f(s.ax, s.ay) * RAD_TO_DEG; t.gz = s.gz;
  t.err = dbg_err_deg; t.P = dbg_P; t.I = dbg_I; t.D = dbg_D; t.pre = dbg_pre; t.out = out_V;
  t.sat = dbg_sat ? 1 : 0; t.fallen = dbg_fallen ? 1 : 0;
  t.ax = s.ax; t.ay = s.ay; t.az = s.az; t.W = dbg_W; t.trim = lqi_sp_trim * RAD_TO_DEG;
  t.clip = (fabsf(s.ax) > 1.99f || fabsf(s.ay) > 1.99f || fabsf(s.az) > 1.99f) ? 1 : 0;
  portENTER_CRITICAL(&telMux); telem = t; portEXIT_CRITICAL(&telMux);
}

void ctrlTask(void* arg) {
  TickType_t last = xTaskGetTickCount();
  uint8_t est_prev = estado;
  for (;;) {
    uint8_t est = estado;
    uint32_t per = (est == EST_ATIVO) ? CTRL_PERIOD_MS : (est == EST_STANDBY ? STANDBY_PERIOD_MS : ECO_PERIOD_MS);
    if (est != est_prev) { last = xTaskGetTickCount(); est_prev = est; }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(per));
    ctrlStep();
  }
}

// ============================================================
//  Bluetooth: interpretador de comandos próprio (roda na tarefa de comunicação)
// ============================================================
// O Commander do SimpleFOC só guarda 20 comandos; registrar mais que isso corrompe a memória.
// Aqui o limite é MAX_CMDS. Aceita "K30", "k30", "K 30"; Enter = \n ou \r\n.
#define MAX_CMDS 48
struct CmdEntry { char id; void (*fn)(char*); };
CmdEntry cmd_tab[MAX_CMDS];
int cmd_n = 0;
uint32_t t_last_cmd = 0;

void cmdAdd(char id, void (*fn)(char*), const char* label) {
  if (cmd_n < MAX_CMDS) { cmd_tab[cmd_n].id = id; cmd_tab[cmd_n].fn = fn; cmd_n++; }
}

// Se veio argumento, grava; sempre mostra o valor atual.
void cmdScalar(float* v, char* arg) {
  if (arg != NULL && arg[0] != '\0') *v = atof(arg);
  SerialBT.println(*v, 3);
}

void acordar();   // definida mais abaixo (precisa ser chamada ao receber qualquer comando em ECONOMIA)

void cmdRun() {
  static char buf[32];
  static uint8_t len = 0;
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        t_last_cmd = millis();
        if (estado == EST_ECO) acordar();          // qualquer comando acorda da economia
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
//  Transições de estado (executadas na tarefa de comunicação)
// ============================================================
const char* estadoNome(uint8_t e) { return e == EST_ATIVO ? "ATIVO" : (e == EST_STANDBY ? "STANDBY" : "ECONOMIA"); }

void tuneFinish(bool abortado, const char* motivo);   // definida mais abaixo
void onLQI(char* cmd);                                  // definida mais abaixo

void acordar() {
  if (estado != EST_ECO) return;
  setCpuFrequencyMhz(CPU_MHZ_NORMAL);
  mpu_wake_req = true;
  uint32_t t0 = millis();
  while (mpu_wake_req && millis() - t0 < 800) vTaskDelay(pdMS_TO_TICKS(10));
  uint32_t t1 = millis();
  while (!mpu_awake && millis() - t1 < 400) vTaskDelay(pdMS_TO_TICKS(10));
  estado = EST_STANDBY;
  t_last_cmd = millis();
}

void entrarEco() {
  if (estado != EST_STANDBY) return;
  mpu_sleep_req = true;
  uint32_t t0 = millis();
  while (mpu_awake && millis() - t0 < 800) vTaskDelay(pdMS_TO_TICKS(10));
  setCpuFrequencyMhz(CPU_MHZ_ECO);
  estado = EST_ECO;
  SerialBT.println("ECONOMIA: IMU em sleep, CPU a 80 MHz. Qualquer comando acorda.");
}

void desligar(bool eco) {
  if (estado == EST_ATIVO) {
    estado = EST_STANDBY;          // o loop() FOC vê isso, desabilita o driver e zera motor_on
    u_cmd = 0;
    uint32_t t0 = millis();
    while (motor_on && millis() - t0 < 500) vTaskDelay(pdMS_TO_TICKS(5));
    ctrlReset();
    SerialBT.println(">>> DESLIGADO: motor sem energia (STANDBY). '@' liga de novo.");
  } else {
    SerialBT.println("ja esta desligado (STANDBY/ECONOMIA).");
  }
  if (eco) entrarEco();
}

void ligar() {
  if (estado == EST_ATIVO) { SerialBT.println("ja esta ligado."); return; }
  if (estado == EST_ECO) acordar();
  if (bias_cal) { SerialBT.println("aguarde o fim da calibracao (B) para ligar."); return; }
  if (!mpu_awake) { SerialBT.println("IMU dormindo, nao posso ligar."); return; }
  ctrlReset();
  angle_resync_req = true;
  if (!foc_ok) {
    SerialBT.println("ligando: alinhando o encoder (a roda vai se mexer um pouco)...");
    motor.init();
    motor.initFOC();
    foc_ok = true;
  } else {
    motor.enable();
    sensor.update(); vTaskDelay(pdMS_TO_TICKS(2)); sensor.update();   // evita pico na 1a velocidade
  }
  motor_on = true;
  estado = EST_ATIVO;
  SerialBT.printf(">>> LIGADO em M%d. '!' desliga.\n", modoIdx(modo));
}

// ============================================================
//  Auto-ajuste do PID (twiddle) — roda na tarefa de comunicação
// ============================================================
Twiddle tw;
bool tune_on = false;
enum { T_WARM, T_MEAS } tune_st = T_WARM;
uint32_t tune_t0 = 0;
float* tune_ptr[Twiddle::MAXP];
const char* tune_name[Twiddle::MAXP];
int tune_n = 0;
float tune_cost0 = 0;
bool  tune_lqi = false;                 // true = ajusta escalas dos ganhos LQI (M3)
float tune_scale[4] = {1, 1, 1, 1};
float tune_base[4]  = {0, 0, 0, 0};
#define TUNE_WARM_MS 2000

void lqiApplyScale() {
  lqi_k1 = tune_base[0] * tune_scale[0];
  lqi_k2 = tune_base[1] * tune_scale[1];
  lqi_k3 = tune_base[2] * tune_scale[2];
  lqi_k4 = tune_base[3] * tune_scale[3];
}
void tuneApply(const float* v) {
  for (int i = 0; i < tune_n; i++) *tune_ptr[i] = v[i];
  if (tune_lqi) lqiApplyScale();
}

void tunePrintParams() {
  for (int i = 0; i < tune_n; i++) SerialBT.printf(" %s=%.3f", tune_name[i], *tune_ptr[i]);
}

void tuneFinish(bool abortado, const char* motivo) {
  if (!tune_on) return;
  tuneApply(tw.best);
  tune_on = false;
  SerialBT.printf("TUNE %s (%d testes, %s):", abortado ? "INTERROMPIDO" : "FIM", tw.trials, motivo);
  tunePrintParams();
  SerialBT.printf(" custo=%.2f (inicial %.2f). Mande W para salvar.\n", tw.best_cost, tune_cost0);
  if (tune_lqi) onLQI(NULL);
}

void tuneStart() {
  if (tune_on) { SerialBT.println("auto-ajuste ja esta rodando (~0 cancela)."); return; }
  if (estado != EST_ATIVO || modo < 0.5f) {
    SerialBT.println("auto-ajuste precisa do motor LIGADO (@) em M1, M2 ou M3, com o pendulo em equilibrio.");
    return;
  }
  float init[Twiddle::MAXP], lo[Twiddle::MAXP], hi[Twiddle::MAXP];
  int n = 0;
  // limites: de 1/4 a 4x o valor atual, dentro de faixas absolutas seguras
  #define TUNE_ADD(PTR, NAME, ABS_LO, ABS_HI) \
    { float v0 = *(PTR); tune_ptr[n] = (PTR); tune_name[n] = (NAME); init[n] = v0; \
      lo[n] = max((float)(ABS_LO), v0 * 0.25f); hi[n] = min((float)(ABS_HI), v0 * 4.0f); n++; }
  tune_lqi = (modo > 2.5f);
  if (tune_lqi) {
    // LQI: ajusta ESCALAS multiplicativas (ganho = base * escala), de 0,4x a 2,5x; assim o sinal do k3 é preservado
    tune_base[0] = lqi_k1; tune_base[1] = lqi_k2; tune_base[2] = lqi_k3; tune_base[3] = lqi_k4;
    const char* nm[4] = {"k1x", "k2x", "k3x", "k4x"};
    for (int i = 0; i < 4; i++) {
      if (fabsf(tune_base[i]) < 1e-6f) continue;     // ganho zerado: não ajusta
      tune_scale[i] = 1.0f;
      tune_ptr[n] = &tune_scale[i]; tune_name[n] = nm[i]; init[n] = 1.0f; lo[n] = 0.4f; hi[n] = 2.5f; n++;
    }
  } else {
    TUNE_ADD(&Kp, "Kp", 1.0f, 100.0f);
    TUNE_ADD(&Kd, "Kd", 0.02f, 10.0f);
    if (Ki > 0.05f) TUNE_ADD(&Ki, "Ki", 0.05f, 20.0f);
    if (modo > 1.5f) TUNE_ADD(&bb_V, "C", 0.3f, VOLTAGE_LIMIT_MAX);
  }
  #undef TUNE_ADD
  tune_n = n;
  tw.begin(n, init, lo, hi, 0.25f);
  tune_on = true;
  tune_st = T_WARM;
  tune_t0 = millis();
  Metrics d; metricsTake(&d);
  SerialBT.printf("TUNE inicio: %d parametros, janela %.0f s por teste. Deixe o pendulo equilibrando; '!' ou ~0 cancela.\n",
                  n, tune_win_s);
  SerialBT.println("TUNE medindo o ponto de partida...");
}

void tuneStep() {
  if (!tune_on) return;
  if (estado != EST_ATIVO || modo < 0.5f || (modo > 2.5f) != tune_lqi) { tuneFinish(true, "motor desligado ou modo trocado"); return; }
  uint32_t now = millis();
  if (tune_st == T_WARM) {
    if (now - tune_t0 >= TUNE_WARM_MS) { Metrics d; metricsTake(&d); tune_t0 = now; tune_st = T_MEAS; }
    return;
  }
  if (now - tune_t0 < (uint32_t)(tune_win_s * 1000.0f)) return;

  // fim da janela: mede o custo do conjunto que estava aplicado
  Metrics m; metricsTake(&m);
  float c = tuneCost(m);
  bool base = !tw.have_base;
  tw.report(c);
  if (base) tune_cost0 = c;
  float rms = (m.n > 0) ? sqrtf((float)(m.err2 / m.n)) : 0.0f;
  SerialBT.printf("TUNE teste %d:", tw.trials);
  tunePrintParams();
  SerialBT.printf(" -> custo=%.2f (erro_rms=%.2f deg, kicks=%lu) melhor=%.2f\n",
                  c, rms, (unsigned long)m.kicks, tw.best_cost);

  const float* nx = tw.trial();
  if (tw.done) { tuneFinish(false, "convergiu"); return; }
  tuneApply(nx);
  tune_st = T_WARM; tune_t0 = millis();
  Metrics d; metricsTake(&d);
}

// ============================================================
//  Comandos Bluetooth
// ============================================================
void onAmp(char* cmd)  { cmdScalar(&amplitude_V, cmd); }
void onPer(char* cmd)  { cmdScalar(&periodo_ms, cmd); }
void onModo(char* cmd) { cmdScalar(&modo, cmd); modo = constrain(modo, 0.0f, 3.0f); ctrlReset(); }

// R: inverte o sentido do torque do motor (use se ele reagir para o lado errado)
void onInv(char* cmd) {
  dir_motor = -dir_motor;
  ctrlReset();
  SerialBT.print("sentido motor: ");
  SerialBT.println(dir_motor > 0 ? "normal (+1)" : "invertido (-1)");
}

void printHeader() {
  SerialBT.println("t_ms,modo,ang_deg,ang_acc_deg,gz_dps,erro_deg,P_V,I_V,D_V,pid_V,saida_V,sat,caido,vel_roda_rads,ax_g,ay_g,az_g,clip,ctrl_hz,foc_hz,mpu_falhas,W_V,trim_deg");
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

// B: calibra o offset do giroscópio (haste PARADA). Só com o motor desligado.
void onBias(char* cmd) {
  if (estado == EST_ATIVO) { SerialBT.println("desligue o motor antes (!) para calibrar o giro."); return; }
  bias_soma = 0; bias_cnt = 0; bias_fim_ms = millis() + 2000; bias_cal = true;
  SerialBT.println("calibrando bias do giro por 2 s: segure a haste PARADA...");
}

bool paramsSalvos();   // definida mais abaixo (usa a memória NVS)

// Q: mostra todos os parâmetros atuais
void onStatus(char* cmd) {
  SerialBT.printf("estado=%s  modo=M%d  sentido_motor=%+d  setpoint=%.2f deg\n", estadoNome(estado),
                  modoIdx(modo), (int)dir_motor, setpoint_deg);
  SerialBT.printf("Kp=%.3f (= %.3f V por grau)  Ki=%.3f  Kd=%.3f\n", Kp, Kp * DEG_TO_RAD, Ki, Kd);
  SerialBT.printf("limite_saida=%.1f V  I_MAX=%.1f V  corte_queda=%.0f deg  fonte=%.2f V\n",
                  VOLTAGE_LIMIT_MAX, I_MAX_V, SAFETY_ANGLE_DEG, supply_V);
  SerialBT.printf("onda quadrada (M0): amp=%.2f V  periodo=%.0f ms\n", amplitude_V, periodo_ms);
  SerialBT.printf("log: %s a %.0f Hz, %s\n", fmt_csv ? "CSV" : "texto", telem_hz, log_ativo ? "ATIVO" : "parado");
  SerialBT.printf("controle=%.0f Hz (nucleo 0)  FOC=%.0f Hz (nucleo 1)  mpu_falhas=%lu  bias_gz=%.3f dps  DLPF=%d  CPU=%d MHz\n",
                  ctrl_hz, foc_hz, (unsigned long)mpu_falhas, gyro_bias_z, (int)MPU_DLPF_CFG, (int)getCpuFrequencyMhz());
  onLQI(NULL);
  SerialBT.printf("kick=%.1f V (0 = desligado, M1/M2/M3)  C=%.2f V  N=%.2f deg  O=%.0f ms\n", kick_V, bb_V, bb_db_deg, bb_dwell_ms);
  SerialBT.printf("kick tempos: G(espera)=%.0f ms  U(|gz| parado)=%.0f dps  E(recuo)=%.0f ms\n", kick_wait_ms, kick_gz_dps, kick_t1_ms);
  SerialBT.printf("economia em %.0f s sem comando (^)  auto-desligar longe do setpoint por %.0f s (%%)  janela do auto-ajuste %.0f s\n",
                  eco_s, perda_s, tune_win_s);
  SerialBT.printf("memoria: %s\n", paramsSalvos() ? "ha parametros salvos (carregados no boot)" : "nada salvo (valores padrao do codigo)");
  SerialBT.printf("ultimo reset: %s\n", reset_motivo);
}

// H: legenda do log e lista de comandos
void onHelp(char* cmd) {
  SerialBT.println("--- COLUNAS DO LOG (CSV) ---");
  SerialBT.println("t_ms        tempo desde o boot (ms)");
  SerialBT.println("modo        0 = onda quadrada, 1 = PID, 2 = PID + pulsos, 3 = LQI");
  SerialBT.println("ang_deg     angulo da haste filtrado (graus, 0 = vertical) <- o que o PID usa");
  SerialBT.println("ang_acc_deg angulo so do acelerometro (graus), compare com ang_deg");
  SerialBT.println("gz_dps      velocidade angular da haste (graus/s), bias ja removido");
  SerialBT.println("erro_deg    setpoint - ang_deg (graus)");
  SerialBT.println("P_V I_V D_V contribuicao de cada termo do PID (V)");
  SerialBT.println("            em M3 (LQI): P = -k1*theta, D = -k2*dtheta, I = -k4*z");
  SerialBT.println("pid_V       P+I+D antes de limitar (V)");
  SerialBT.println("saida_V     tensao enviada ao motor (V, ja com limite e sentido R)");
  SerialBT.println("sat         1 = pid_V foi cortado pelo limite de tensao");
  SerialBT.println("caido       1 = |angulo| > corte, PID desligado");
  SerialBT.println("vel_roda    velocidade da roda (rad/s)");
  SerialBT.println("ax,ay,az    acelerometro (g). ay~+1 e ax~0 com a haste em pe");
  SerialBT.println("clip        1 = acelerometro no fundo de escala (2 g)");
  SerialBT.println("ctrl_hz     voltas da tarefa de controle por segundo (ideal ~500)");
  SerialBT.println("foc_hz      voltas do loop FOC por segundo (ideal > 2000)");
  SerialBT.println("mpu_falhas  leituras do MPU que falharam");
  SerialBT.println("W_V         em M3: -k3*w_roda (V)");
  SerialBT.println("trim_deg    (ultima coluna) em M3: deslocamento aprendido do setpoint (graus). setpoint real = S + trim");
  SerialBT.println("--- COMANDOS ---");
  SerialBT.println("! desliga motor (!1 = ja em economia) | @ liga (@0..@3 escolhe modo) | ^<s> tempo p/ economia | %<s> auto-desligar");
  SerialBT.println("~ auto-ajuste do PID (~<s> janela, ~0 cancela) | W salva | Y apaga salvos");
  SerialBT.println("M0/M1/M2/M3 modo | K I D ganhos | S setpoint | Z setpoint=angulo atual | R inverte motor");
  SerialBT.println("L<seg> log (L0 para) | F formato | T<hz> taxa | B bias giro | Q status | H ajuda");
  SerialBT.println("M2 = PID + pulsos: C amplitude minima | N zona morta (graus) | O dwell (ms)");
  SerialBT.println("M3 = LQI: $ mostra | $1..$4 ganhos | $5 limite integral (V) | $6 zona morta (V) | $7 kick roda | $8 pulsos 1/0 | $9 auto-trim | $10 dwell M3 (ms)");
  SerialBT.println("kick (M1, M2 e M3): X<V> (X0 desliga) | G espera (ms) | U |gz| parado (dps) | E recuo (ms)");
  SerialBT.println("limites: V<V> tensao max (<=8) | J integrador (V) | #<V> tensao da fonte");
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
void onSet(char* cmd)  { cmdScalar(&setpoint_deg, cmd); ctrlReset(); }

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

// $: ganhos do LQI (M3). "$" mostra todos | "$1 170" muda k1 | $2 k2 | $3 k3 | $4 k4 | $5 limite integral (V)
void onLQI(char* cmd) {
  int idx = 0; float val = 0;
  int n = (cmd != NULL) ? sscanf(cmd, "%d %f", &idx, &val) : 0;
  if (n == 2) {
    switch (idx) {
      case 1: lqi_k1 = val; break;
      case 2: lqi_k2 = val; break;
      case 3: lqi_k3 = val; break;
      case 4: lqi_k4 = val; break;
      case 5: lqi_zmax_V = constrain(val, 0.0f, 8.0f); break;
      case 6: lqi_dead_V = constrain(val, 0.0f, 2.0f); break;
      case 7: lqi_wk = constrain(val, 5.0f, 150.0f); break;
      case 8: lqi_pulse = (val > 0.5f) ? 1.0f : 0.0f; break;
      case 9: lqi_trim = constrain(val, 0.0f, 0.05f); break;
      case 10: lqi_dwell_ms = constrain(val, 5.0f, 500.0f); break;
      default: SerialBT.println("use $1..$5"); return;
    }
    ctrlReset();   // zera o integrador ao mudar ganho
  }
  SerialBT.printf("LQI: k1=%.3f V/rad (%.3f V/grau)  k2=%.3f  k3=%.4f  k4=%.4f  lim_int=%.2f V\n",
                  lqi_k1, lqi_k1 * DEG_TO_RAD, lqi_k2, lqi_k3, lqi_k4, lqi_zmax_V);
  SerialBT.printf("LQI saida: %s  zona_morta($6)=%.2f V  kick_roda($7)=%.0f rad/s  (C=%.2f V valem tambem aqui)\n",
                  lqi_pulse > 0.5f ? "PULSOS" : "continua", lqi_dead_V, lqi_wk, bb_V);
  SerialBT.printf("LQI pulsos: dwell($10)=%.0f ms | auto-trim($9)=%.4f (0 = off): trim atual=%+.2f deg -> equilibrio estimado = %.2f deg (setpoint S=%.2f)\n",
                  lqi_dwell_ms, lqi_trim, lqi_sp_trim * RAD_TO_DEG, setpoint_deg + lqi_sp_trim * RAD_TO_DEG, setpoint_deg);
}

// #<volts>: tensão de alimentação (6 a 16 V). Ex.: "#12.4". Sem argumento só mostra. W salva.
void onSupply(char* cmd) {
  if (cmd != NULL && cmd[0] != '\0') {
    supply_V = constrain((float)atof(cmd), 6.0f, 16.0f);
    driver.voltage_power_supply = supply_V;
    driver.voltage_limit = supply_V;
  }
  SerialBT.printf("tensao de alimentacao = %.2f V\n", supply_V);
}

// Tempos do kick (valem em M1, M2 e M3). Sem argumento só mostram o valor atual.
void onKWait(char* cmd) { cmdScalar(&kick_wait_ms, cmd); kick_wait_ms = constrain(kick_wait_ms, 20.0f, 2000.0f); }
void onKGz(char* cmd)   { cmdScalar(&kick_gz_dps, cmd);  kick_gz_dps  = constrain(kick_gz_dps, 1.0f, 200.0f); }
void onKT1(char* cmd)   { cmdScalar(&kick_t1_ms, cmd);   kick_t1_ms   = constrain(kick_t1_ms, 20.0f, 500.0f); }

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
  ctrlReset();
  SerialBT.printf("setpoint = %.2f deg (angulo atual)\n", setpoint_deg);
}

// !  desliga o motor.  !1 = desliga e entra em economia já.
void onDesliga(char* cmd) {
  tuneFinish(true, "desligado pelo usuario");
  desligar(cmd != NULL && cmd[0] == '1');
}

// @  liga o motor.  @0 a @3 = liga já no modo escolhido.
void onLiga(char* cmd) {
  if (cmd != NULL && cmd[0] >= '0' && cmd[0] <= '3') { modo = (float)(cmd[0] - '0'); ctrlReset(); }
  ligar();
}

// $<s>: segundos sem comando até a economia (0 = nunca)
void onEco(char* cmd) { cmdScalar(&eco_s, cmd); eco_s = constrain(eco_s, 0.0f, 86400.0f); }

// %<s>: desliga sozinho se ficar longe do setpoint por mais de <s> s (0 = nunca)
void onPerda(char* cmd) { cmdScalar(&perda_s, cmd); perda_s = constrain(perda_s, 0.0f, 3600.0f); }

// ~  auto-ajuste.  ~<s> define a janela (s) e inicia.  ~0 cancela.
void onTune(char* cmd) {
  if (cmd != NULL && cmd[0] != '\0') {
    float v = atof(cmd);
    if (v <= 0.0f) { if (tune_on) tuneFinish(true, "cancelado"); else SerialBT.println("auto-ajuste nao esta rodando."); return; }
    tune_win_s = constrain(v, 4.0f, 120.0f);
  }
  tuneStart();
}

// ---- Parâmetros persistentes (NVS do ESP32) ----
// W salva os parâmetros abaixo; no próximo boot eles são carregados. Y apaga o que foi salvo.
// O modo e o estado NÃO são salvos: o pêndulo sempre liga DESLIGADO (STANDBY) e em M1.
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
    lqi_k1 = prefs.getFloat("l1", lqi_k1);
    lqi_k2 = prefs.getFloat("l2", lqi_k2);
    lqi_k3 = prefs.getFloat("l3", lqi_k3);
    lqi_k4 = prefs.getFloat("l4", lqi_k4);
    lqi_zmax_V = constrain(prefs.getFloat("l5", lqi_zmax_V), 0.0f, 8.0f);
    lqi_dead_V = constrain(prefs.getFloat("l6", lqi_dead_V), 0.0f, 2.0f);
    lqi_wk = constrain(prefs.getFloat("l7", lqi_wk), 5.0f, 150.0f);
    lqi_pulse = prefs.getFloat("l8", lqi_pulse) > 0.5f ? 1.0f : 0.0f;
    lqi_trim = constrain(prefs.getFloat("l9", lqi_trim), 0.0f, 0.05f);
    lqi_dwell_ms = constrain(prefs.getFloat("l10", lqi_dwell_ms), 5.0f, 500.0f);
    gyro_bias_z = prefs.getFloat("bias", gyro_bias_z);
    telem_hz = constrain(prefs.getFloat("thz", telem_hz), 1.0f, 50.0f);
    fmt_csv = prefs.getBool("csv", fmt_csv);
    eco_s = constrain(prefs.getFloat("eco", eco_s), 0.0f, 86400.0f);
    perda_s = constrain(prefs.getFloat("perda", perda_s), 0.0f, 3600.0f);
    tune_win_s = constrain(prefs.getFloat("twin", tune_win_s), 4.0f, 120.0f);
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
  prefs.putFloat("l1", lqi_k1);
  prefs.putFloat("l2", lqi_k2);
  prefs.putFloat("l3", lqi_k3);
  prefs.putFloat("l4", lqi_k4);
  prefs.putFloat("l5", lqi_zmax_V);
  prefs.putFloat("l6", lqi_dead_V);
  prefs.putFloat("l7", lqi_wk);
  prefs.putFloat("l8", lqi_pulse);
  prefs.putFloat("l9", lqi_trim);
  prefs.putFloat("l10", lqi_dwell_ms);
  prefs.putFloat("bias", gyro_bias_z);
  prefs.putFloat("thz", telem_hz);
  prefs.putBool("csv", fmt_csv);
  prefs.putFloat("eco", eco_s);
  prefs.putFloat("perda", perda_s);
  prefs.putFloat("twin", tune_win_s);
  prefs.end();
  SerialBT.println("parametros SALVOS na memoria (K I D S R V J X C N O G U E bias T F # ^ % ~ + LQI $1..$5). Serao carregados no boot.");
}

void onClearSaved(char* cmd) {
  prefs.begin("pend", false);
  prefs.clear();
  prefs.end();
  SerialBT.println("memoria apagada: no proximo boot volta aos valores padrao do codigo.");
}

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

// ============================================================
//  TAREFA DE COMUNICAÇÃO (núcleo 0, prioridade baixa): comandos, log, economia, auto-ajuste
// ============================================================
void comStep() {
  cmdRun();
  uint32_t now = millis();

  // eventos vindos do controle
  static uint32_t ke_prev = 0;
  if (kick_evt != ke_prev) { ke_prev = kick_evt; SerialBT.println("KICK"); }
  if (bias_msg) {
    bias_msg = false;
    SerialBT.printf("bias gz = %.3f dps (%lu amostras). W salva.\n", gyro_bias_z, (unsigned long)bias_cnt);
  }
  uint8_t r = auto_off_reason;
  if (r) {
    auto_off_reason = 0;
    SerialBT.println(r == 1 ? "AUTO-DESLIGADO: ficou longe do setpoint por tempo demais (%)."
                            : "AUTO-DESLIGADO: falha na leitura do MPU (verifique a fiacao).");
    tuneFinish(true, "desligado automaticamente");
    desligar(false);
  }

  // taxa do loop FOC
  static uint32_t fc_prev = 0, ft_prev = 0;
  if (now - ft_prev >= 1000) {
    uint32_t fc = foc_count;
    foc_hz = (fc - fc_prev) * 1000.0f / (now - ft_prev);
    fc_prev = fc; ft_prev = now;
  }

  // log pedido (comando L)
  static uint32_t t_telem = 0;
  if (log_ativo && (int32_t)(now - log_ate_ms) >= 0) {
    log_ativo = false;
    SerialBT.println("fim do log");
  }
  if (log_ativo && now - t_telem >= (uint32_t)(1000.0f / telem_hz)) {
    t_telem = now;
    if (SerialBT.hasClient()) {                    // (availableForWrite() retorna 0 no BluetoothSerial)
      Telem t; telemGet(&t);
      float vel = wheel_vel;
      char buf[360];
      int n;
      if (fmt_csv) {
        n = snprintf(buf, sizeof(buf),
                     "%lu,%d,%.2f,%.2f,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2f,%.3f,%.3f,%.3f,%d,%.0f,%.0f,%lu,%.3f,%.2f\n",
                     (unsigned long)t.t_ms, t.modo, t.ang, t.ang_acc, t.gz, t.err,
                     t.P, t.I, t.D, t.pre, t.out, t.sat, t.fallen,
                     vel, t.ax, t.ay, t.az, t.clip, (float)ctrl_hz, (float)foc_hz, (unsigned long)mpu_falhas, t.W, t.trim);
      } else {
        n = snprintf(buf, sizeof(buf),
                     "t=%.2fs M%d | ang=%+.2f acc=%+.2f gz=%+.1f erro=%+.2f | P=%+.3f I=%+.3f D=%+.3f pid=%+.3f OUT=%+.3fV SAT=%d CAIDO=%d | roda=%+.2f rad/s | a=(%+.2f,%+.2f,%+.2f)g CLIP=%d | ctrl=%.0fHz foc=%.0fHz falhas=%lu W=%+.3f trim=%+.2f\n",
                     t.t_ms / 1000.0f, t.modo, t.ang, t.ang_acc, t.gz, t.err,
                     t.P, t.I, t.D, t.pre, t.out, t.sat, t.fallen,
                     vel, t.ax, t.ay, t.az, t.clip, (float)ctrl_hz, (float)foc_hz, (unsigned long)mpu_falhas, t.W, t.trim);
      }
      if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
      SerialBT.write((const uint8_t*)buf, n);
    }
  }

  tuneStep();

  // economia automática: STANDBY sem comandos por eco_s segundos
  if (estado == EST_STANDBY && eco_s > 0.5f && !log_ativo && !bias_cal &&
      now - t_last_cmd > (uint32_t)(eco_s * 1000.0f)) {
    entrarEco();
  }
}

void comTask(void* arg) {
  for (;;) {
    comStep();
    vTaskDelay(pdMS_TO_TICKS(estado == EST_ECO ? ECO_PERIOD_MS : 10));
  }
}

// ============================================================
//  setup(): hardware pronto, motor DESLIGADO; cria as tarefas
// ============================================================
void setup() {
  Serial.begin(115200);   // só para debug via USB, se estiver plugado
  loadParams();           // carrega parâmetros salvos (comando W), se existirem
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

  // Driver: inicializa e já DESABILITA (saídas em alta impedância; DRV8313 dorme)
  driver.voltage_power_supply = supply_V;
  driver.voltage_limit = supply_V;
  driver.init();
  driver.disable();
  motor.linkDriver(&driver);

  // Controle por torque (tensão), sem sensor de corrente
  motor.voltage_limit = VOLTAGE_LIMIT_MAX;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.useMonitoring(SerialBT);
  // motor.init() e motor.initFOC() NÃO são chamados aqui: o motor começa desligado.
  // Eles rodam no primeiro '@' (liga), quando o encoder é alinhado.

  cmdAdd('!', onDesliga, "desliga o motor (!1 = economia)");
  cmdAdd('@', onLiga,  "liga o motor (@0/@1/@2 = modo)");
  cmdAdd('^', onEco,   "segundos ate a economia");
  cmdAdd('$', onLQI,   "ganhos do LQI (M3)");
  cmdAdd('%', onPerda, "auto-desligar longe do setpoint (s)");
  cmdAdd('~', onTune,  "auto-ajuste do PID");
  cmdAdd('A', onAmp,  "amplitude onda quadrada (V) [modo M0]");
  cmdAdd('P', onPer,  "periodo onda quadrada (ms) [modo M0]");
  cmdAdd('M', onModo, "modo: 0=onda quadrada, 1=PID, 2=PID+pulsos, 3=LQI");
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
  cmdAdd('C', onBBV,    "M2: amplitude minima do pulso (V)");
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
  t_last_cmd = millis();
  modo = 1.0f;            // sempre liga em M1 (e DESLIGADO)

  // Controle e comunicação no núcleo 0; o loop() (FOC) já roda no núcleo 1.
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", CTRL_STACK, NULL, CTRL_PRIO, NULL, 0);
  xTaskCreatePinnedToCore(comTask,  "com",  COM_STACK,  NULL, COM_PRIO,  NULL, 0);
}

// ============================================================
//  loop(): roda no núcleo 1 — FOC. Só executa com o motor ligado.
// ============================================================
void loop() {
  if (estado == EST_ATIVO && motor_on) {
    motor.loopFOC();
    wheel_vel = motor.shaft_velocity;
    motor.move(u_cmd);
    foc_count = foc_count + 1;
  } else {
    if (motor_on) {                 // saiu do estado ATIVO: tira a energia do motor aqui, no dono do motor
      motor.move(0);
      motor.disable();
      motor_on = false;
    }
    wheel_vel = 0;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}