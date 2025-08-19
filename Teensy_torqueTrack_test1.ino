/*
K_fric を0.7–1.2で調整 → 全体の追従感を合わせる。
静止近傍の押し引きが残るなら bias_cutoff_hz と TAU_BIAS_MAX を微調整。(ゼロ近傍での押し引きが残るなら bias_cutoff_hz を0.3–0.8 Hzで調整し、TAU_BIAS_MAXを安全側に。)
微小速度でチラつくなら OMEGA_DB を 0.01–0.05 rad/s で調整。（静止付近のチラツキ対策）。
StribeckのTs_fric：今0なのでブレークアウェイが欲しければ Ts_fric ≥ Tc_fric に（実機同定値に合わせて）。
ドライバが荒れる場合は DI_MAX を小さく（レート制限強化）。
*/

#include <SPI.h>
#include <TimerOne.h>
#include <math.h>

// ========================== Pin definitions ==========================
#define LED1h 23
#define LED1k 22
#define LED2h 19
#define LED2k 18

#define SS1h 7
#define SS1k 38
#define SS2h 6
#define SS2k 5

// ========================== Motor / model params =====================
// Motor torque constant
const float Kt = 0.043f;        // [Nm/A]

// Encoder (19-bit SSI)
const int   ENC_BITS = 19;
const long  ENC_MOD  = 1L << ENC_BITS;   // 524288
const long  ENC_HALF = ENC_MOD >> 1;     // 262144
const float ENC2RAD  = 2.0f * PI / 524288.0f; // [rad/count]

// Identified friction parameters (tune to your plant)
const float Tc_fric  = 0.011f;     // Coulomb friction [Nm]
const float Ts_fric  = 0.0f;       // Static/Stribeck peak [Nm] (>= Tc_fric for breakaway)
const float omega_s  = 0.3469f;    // Stribeck velocity [rad/s]
const float alpha_s  = 0.444f;     // Stribeck shape factor [-]
const float B        = 0.000111f;  // Viscous friction [Nm/(rad/s)]

// ========================== Control frequency ========================
const int   control_freq_hz = 300;          // [Hz]
const float dt              = 1.0f / control_freq_hz; // [s]

// ========================== Feedforward-only control =================
// Outer PI is removed. We rely on driver’s inner current loop.
// We add: FF gain, bias learning, current rate limiting, and filtering.
const float K_fric           = 0.9f;    // Friction FF gain (tune 0.7–1.2)
const float I_MAX          = 1.0f;     // Current limit [A] (respect driver spec)
const float DI_MAX         = 0.02f;    // Max current step per ISR [A/tick] (~6 A/s @300Hz)
const float OMEGA_DB       = 0.01f;    // Deadband for sign stabilization [rad/s] 0.02 initially
const float TAU_BIAS_MAX   = 0.06f;    // Clamp for bias integrator [Nm]

// Bias learning (slow offset rejection near neutral)
float tau_bias             = 0.0f;     // Learned torque bias [Nm]
float bias_alpha           = 0.0f;     // IIR coefficient for slow bias (set in setup)
const float bias_cutoff_hz = 0.5f;     // ~0.3–0.8 Hz reasonable

// Bias learning gates (learn only when "near neutral")
const float tau_ref_th     = 0.005f;    // |Tp_ref| threshold [Nm]
const float omega_th       = 0.1f;     // |omega| threshold [rad/s] (~5.7 deg/s)

// [Nm/(rad/s)] virtual damping (start 0.00005–0.0001)
const float Kd_visc = 0.00015f;
const float Tgate    = 0.005f;    // [Nm] gate center for |Tp_ref|. g becomes 0.5 at Tgate [Nm].
const float gate_p   = 2.0f;     // shape (>=1), larger => sharper gate

// ========================== Velocity filtering =======================
float omega       = 0.0f;              // filtered velocity [rad/s]
float omega_lpf   = 0.0f;              // LPF state
const float omega_fc_hz = 25.0f;       // 20–30 Hz typical @300Hz
float omega_alpha = 0.0f;              // IIR coefficient for velocity LPF

// ========================== SPI comm ================================
SPISettings settings(100000, MSBFIRST, SPI_MODE3); // 100 kHz (comment says 1 MHz max, keep 100k for safety)

bool  conn1 = false;
volatile long  enc1 = 0;               // last raw encoder sample (from SPI)
volatile float cur1 = 0.0f;            // last measured current [A] (from SPI)
byte last_rx_buf[12];
byte last_tx_buf[12];

// Current command (to driver)
volatile float cur_cmd = 0.0f;

// ========================== Reference torque profile ================
float refT[] = {
  -0.024044409379305, -0.057379170811646, -0.082347726221913, -0.101766214933962,
  -0.113655220028023, -0.117663416354635, -0.115056381548052, -0.108405281732599,
  -0.099988540207362, -0.090879528747806, -0.081642672434475, -0.072527546212679,
  -0.063458960718276, -0.054249479016750, -0.044811952196179, -0.035216063994700,
  -0.025540612547182, -0.015911614705552, -0.006501919148448,  0.002432548302564,
   0.010745191351587,  0.018194139928874,  0.024496791284796,  0.029439389639198,
   0.032744632722659,  0.034241708096117,  0.034058915272989,  0.032568936986831,
   0.030153271810964,  0.027117638786424,  0.023741043145104,  0.020194030904439,
   0.016649395025736,  0.013355774514323,  0.010522464005195,  0.008247017600510,
   0.006424851063036,  0.004835031486204,  0.003273337451531,  0.001722633564767,
   0.000388970497158, -0.000470791700955, -0.000641985659551, -0.000067311244177,
   0.001179679523487,  0.002736159666071,  0.004406850039273,  0.007230227976409,
   0.009152320921940, -0.006956574288570
}; // example: hip profile

// float refT[] = {
//   -0.001627031189353, -0.010078134806660, -0.019665055287961, -0.028449318847842,
//   -0.035490434652401, -0.040735705640484, -0.044278299173914, -0.046483940264661,
//   -0.047862586378117, -0.048865240917389, -0.049824949487050, -0.050939523812327,
//   -0.052302587460691, -0.053956909666418, -0.055916233613297, -0.058138984270470,
//   -0.060535773640330, -0.062966925474941, -0.065243978655166, -0.067132944173804,
//   -0.068349359525542, -0.068588261887857, -0.067571582397193, -0.064978413580525,
//   -0.060575065155178, -0.054480797499138, -0.047138104002056, -0.039079301695244,
//   -0.030977734228921, -0.023438668348530, -0.016788037942653, -0.011143652372293,
//   -0.006623949318000, -0.003236190508745, -0.000910088917431,  0.000458748984804,
//    0.001073982824656,  0.001214655788092,  0.001175950182825,  0.001181999947384,
//    0.001345780427386,  0.001695567089574,  0.002206015874143,  0.002808956358927,
//    0.003390201082491,  0.003804974224416,  0.003985610244484,  0.004088539502501,
//    0.004036081121161,  0.001432784028159
// };

int   indexRefT = 0;
int   refTsize  = sizeof(refT)/sizeof(refT[0]);

volatile float Tp_ref = 0.0f;          // global torque reference [Nm]

// ========================== Encoder unwrap & velocity ===============
long enc_prev_raw  = 0;   // last raw sample [0..ENC_MOD-1]
long enc_accum     = 0;   // unwrapped cumulative counts (could exceed 32-bit range in long runs)
long acc_prev1     = 0;   // accum[n-1]
long acc_prev2     = 0;   // accum[n-2]

// ========================== Helpers ================================
inline float iirAlpha(float cutoff_hz, float Ts) {
  // First-order IIR in exponential form: y = a*y + (1-a)*x, with a = exp(-2*pi*fc*Ts)
  return expf(-2.0f * PI * cutoff_hz * Ts);
}

inline long unwrapEncDelta(long now, long prev) {
  // Unwrap delta between two modular encoder samples
  long d = now - prev;
  if (d >  ENC_HALF) d -= ENC_MOD;
  if (d < -ENC_HALF) d += ENC_MOD;
  return d;
}

// Return only friction magnitude of Stribeck + Coulomb (no viscous)
inline float frictionMag(float w_abs) {
  // (Ts - Tc)*exp(-( |w|/ws )^alpha) + Tc
  float strb = (Ts_fric - Tc_fric) * expf(-powf(w_abs / omega_s, alpha_s));
  return (Tc_fric + strb);
}

// Smooth gate: g(|Tp_ref|) in [0..1], 1 near zero torque, ->0 for large |Tp_ref|
inline float gate_by_ref(float Tp_ref_abs) {
  float r = Tp_ref_abs / Tgate;
  return 1.0f / (1.0f + powf(r, gate_p));   // 1/(1+r^p)
}

// ========================== Prototypes =============================
void send_current_command(int SS, bool servo_on, float current_cmd, bool* connection, long* val_enc, float* val_cur);
void validate(byte* rx);

// ========================== Setup ==================================
void setup() {
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT); // MOSI
  pinMode(12, INPUT);  // MISO
  pinMode(13, OUTPUT); // SCK

  pinMode(38, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);

  // Precompute IIR alphas
  omega_alpha = iirAlpha(omega_fc_hz, dt);
  bias_alpha  = iirAlpha(bias_cutoff_hz, dt);

  Serial.begin(57600);
  SPI.begin();
  delay(100);

  // Establish SPI/driver connection and stabilize readings
  bool init_conn = false;
  while (!init_conn) {
    send_current_command(38, 0, 0.0f, &init_conn, (long*)&enc1, (float*)&cur1);
    Serial.print(" connection "); Serial.println(init_conn);
    delay(10);
  }
  for (int i = 0; i < 5; i++) {
    send_current_command(38, 0, 0.0f, &conn1, (long*)&enc1, (float*)&cur1);
    delay(5);
  }

  // Initialize unwrap state with current reading
  enc_prev_raw = enc1;
  enc_accum    = 0;
  acc_prev1    = 0;
  acc_prev2    = 0;
  omega        = 0.0f;
  omega_lpf    = 0.0f;

  // Start control ISR
  Timer1.initialize(1000000 / control_freq_hz);
  Timer1.attachInterrupt(timerISR);
}

// ========================== Main loop ===============================
void loop() {
  // Update torque reference from the profile (example: scaling by 0.5)
  // Tp_ref = refT[indexRefT] * 0.75f;
  // indexRefT++;
  // if (indexRefT >= refTsize) indexRefT = 0;

  // Throttle loop a bit
  delay(10);
}

// ========================== Control ISR =============================
void timerISR() {
  // 1) SPI: send current command, read encoder & measured current
  send_current_command(38, 1, cur_cmd, &conn1, (long*)&enc1, (float*)&cur1);
  if (!conn1) {
    // Fail-safe: if comm error, command 0A and return
    cur_cmd = 0.0f;
    return;
  }

  // 2) Encoder unwrap to cumulative counts
  long d_counts = unwrapEncDelta(enc1, enc_prev_raw);
  enc_prev_raw  = enc1;
  enc_accum    += d_counts;

  // 3) 3-point backward difference on unwrapped counts
  long num_counts = (3L * enc_accum - 4L * acc_prev1 + acc_prev2);
  float omega_raw = (num_counts * ENC2RAD) / (2.0f * dt);

  // 4) LPF on velocity
  omega     = omega_alpha * omega_lpf + (1.0f - omega_alpha) * omega_raw;
  omega_lpf = omega;

  // Shift accum history
  acc_prev2 = acc_prev1;
  acc_prev1 = enc_accum;

  // 5) Measured current (light LPF for robustness in bias learning)
  static float i_meas_lpf = 0.0f;
  const  float i_alpha    = iirAlpha(30.0f, dt);  // ~30 Hz
  i_meas_lpf = i_alpha * i_meas_lpf + (1.0f - i_alpha) * cur1;
  float Tm_meas = Kt * i_meas_lpf;

  // 6) Stabilized friction sign (avoid dithering near zero speed)
  static int last_sign = 0;
  int sgn_w = 0;
  if (fabsf(omega) > OMEGA_DB) {
    sgn_w = (omega > 0) - (omega < 0);
  } else {
    // Prefer reference torque sign; if zero, keep last sign
    int sgn_ref = (Tp_ref > 0) - (Tp_ref < 0);
    sgn_w = (sgn_ref != 0) ? sgn_ref : last_sign;
  }
  if (sgn_w != 0) last_sign = sgn_w;

  // 7) Friction estimate (Stribeck+Coulomb magnitude with stabilized sign, plus viscous)
  float Tfric = sgn_w * frictionMag(fabsf(omega)) + B * omega;

  // 8) Output-side torque estimate (for bias learning / monitoring)
  float Tp_meas = Tm_meas - Tfric;

  // 9) Bias learning near neutral
  if (fabsf(Tp_ref) < tau_ref_th && fabsf(omega) < omega_th) {
    float err_slow = (Tp_meas - Tp_ref); // residual push/pull
    tau_bias = bias_alpha * tau_bias + (1.0f - bias_alpha) * err_slow;
    // Clamp to avoid runaway
    if (tau_bias >  TAU_BIAS_MAX) tau_bias =  TAU_BIAS_MAX;
    if (tau_bias < -TAU_BIAS_MAX) tau_bias = -TAU_BIAS_MAX;
  }

  // Gate factor: strong when |Tp_ref| small, vanishes when large
  float g = gate_by_ref(fabsf(Tp_ref));     // tor_cmd == Tp_ref

  // Additional) Virtual damping (always oppose motion)
  float T_damp = -Kd_visc * omega * g;

  // 10) Command: pure FF with bias compensation (no outer PI)
  float Tm_cmd_target  = Tp_ref + K_fric * Tfric - tau_bias + T_damp;
  float cur_cmd_target = Tm_cmd_target / Kt;

  // 11) Current rate limiting (protect inner current loop)
  static float cur_cmd_prev = 0.0f;
  float di = cur_cmd_target - cur_cmd_prev;
  if (di >  DI_MAX) di =  DI_MAX;
  if (di < -DI_MAX) di = -DI_MAX;
  cur_cmd = cur_cmd_prev + di;
  cur_cmd_prev = cur_cmd;

  // 12) Saturation
  if (cur_cmd >  I_MAX) cur_cmd =  I_MAX;
  if (cur_cmd < -I_MAX) cur_cmd = -I_MAX;

  // 13) Optional: throttle debug to ~50 Hz
  static uint16_t dbg_div = 0;
  if (++dbg_div >= 6) {
    // Serial.print("Tp_ref "); Serial.print(Tp_ref,4);
    // Serial.print(" Tp_meas "); Serial.print(Tp_meas,4);
    // Serial.print(" tau_bias "); Serial.print(tau_bias,4);
    // Serial.print(" cur_cmd "); Serial.println(cur_cmd,4);
    Serial.print(" connection "); Serial.println(conn1);
    dbg_div = 0;
  }
}

// ========================== SPI packet I/O ===========================
void send_current_command(int SS, bool servo_on, float current_cmd, bool* connection, long* val_enc, float* val_cur) {
  byte tx_buf[12] = {0};
  byte rx_buf[12] = {0};

  // Build TX
  tx_buf[0] = servo_on ? 0x08 : 0x00;        // header
  memcpy(&tx_buf[3], &current_cmd, sizeof(float)); // 4 bytes current at [3..6]

  // Checksum (sum of 16-bit words over first 10 bytes)
  uint16_t checksum = 0;
  for (int i = 0; i < 10; i += 2) {
    uint16_t w = tx_buf[i] | (tx_buf[i + 1] << 8);
    checksum += w;
  }
  checksum &= 0x7FFF;
  checksum |= 0x8000;
  tx_buf[10] = checksum & 0xFF;
  tx_buf[11] = (checksum >> 8) & 0xFF;

  // SPI transfer
  SPI.beginTransaction(settings);
  delayMicroseconds(5);
  digitalWrite(SS, LOW);
  delayMicroseconds(10);
  for (int i = 0; i < 12; i++) {
    rx_buf[i] = SPI.transfer(tx_buf[i]);
  }
  delayMicroseconds(10);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();

  // Save last TX/RX
  memcpy(last_tx_buf, tx_buf, 12);
  memcpy(last_rx_buf, rx_buf, 12);

  // Parse checksum
  uint16_t rx_checksum = rx_buf[10] | (rx_buf[11] << 8);
  uint16_t computed = 0;
  for (int i = 0; i < 10; i += 2) {
    uint16_t w = rx_buf[i] | (rx_buf[i + 1] << 8);
    computed += w;
  }
  computed &= 0x7FFF;
  computed |= 0x8000;

  *connection = (rx_checksum == computed);

  // Parse encoder (bytes [1..4])
  int32_t enc_raw = *((int32_t*)&rx_buf[1]);
  *val_enc = enc_raw;

  // Parse measured current as float (bytes [5..8])
  memcpy(val_cur, &rx_buf[5], sizeof(float));
}

// ========================== Debug validator ==========================
void validate(byte* rx) {
  Serial.print("TX: ");
  for (int i = 0; i < 12; i++) { Serial.print(last_tx_buf[i], HEX); Serial.print(" "); }
  Serial.println();

  Serial.print("RX: ");
  for (int i = 0; i < 12; i++) { Serial.print(rx[i], HEX); Serial.print(" "); }
  Serial.println();

  uint16_t rx_checksum = rx[10] | (rx[11] << 8);
  uint16_t computed = 0;
  for (int i = 0; i < 10; i += 2) {
    uint16_t w = rx[i] | (rx[i + 1] << 8);
    computed += w;
  }
  computed &= 0x7FFF;
  computed |= 0x8000;

  Serial.print("Received checksum: 0x"); Serial.println(rx_checksum, HEX);
  Serial.print("Computed checksum: 0x"); Serial.println(computed, HEX);
}