#include <Wire.h>
#include <math.h>

// ===================== CONFIGURAÇÕES =====================
const uint8_t MPU_ADDR = 0x68;
const int SDA_PIN = 21;
const int SCL_PIN = 22;

const float SENSIBILIDADE_4G = 8192.0f; // LSB/g para faixa de +-4g
const int N_AMOSTRAS_CAL = 200;         // Mínimo exigido: 100 leituras
const int N_AMOSTRAS_VAL = 100;         // Leituras por ponto do transferidor

const int MODO_ANGULO = 1;
const float OFFSET_TRANSFERIDOR = 90.0f; 
const float SINAL_ANGULO = 1.0f;

// ===================== PARÂMETROS DE CALIBRAÇÃO =====================
float ganhoX = 0.985912f, offsetX = 1.322125f;
float ganhoY = 0.989389f, offsetY = 0.268019f;
float ganhoZ = 1.014135f, offsetZ = -0.163328f;

// ===================== FUNÇÕES I2C / MPU-6050 =====================
void escreverRegistrador(uint8_t reg, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  Wire.endTransmission(true);
}

bool inicializarMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (size_t)1, true) != 1) return false;
  uint8_t whoAmI = Wire.read();
  Serial.printf("WHO_AM_I: 0x%02X\n", whoAmI);

  // Acorda o sensor
  escreverRegistrador(0x6B, 0x00);
  delay(100);

  // CONFIG = 0x1A -> 0x06
  escreverRegistrador(0x1A, 0x06);

  // Taxa de amostragem (SMPLRT_DIV = 0x19 -> 0x09)
  escreverRegistrador(0x19, 0x09);

  // Fundo de escala (ACCEL_CONFIG = 0x1C -> 0x08)
  escreverRegistrador(0x1C, 0x08);

  return true;
}

bool lerAceleracaoBrutaG(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (size_t)6, true) != 6) return false;

  uint8_t xh = Wire.read(); uint8_t xl = Wire.read();
  uint8_t yh = Wire.read(); uint8_t yl = Wire.read();
  uint8_t zh = Wire.read(); uint8_t zl = Wire.read();

  int16_t rawX = (int16_t)((xh << 8) | xl);
  int16_t rawY = (int16_t)((yh << 8) | yl);
  int16_t rawZ = (int16_t)((zh << 8) | zl);

  ax = rawX / SENSIBILIDADE_4G;
  ay = rawY / SENSIBILIDADE_4G;
  az = rawZ / SENSIBILIDADE_4G;
  return true;
}

bool lerAceleracaoCalibradaG(float &ax, float &ay, float &az) {
  float bx, by, bz;
  if (!lerAceleracaoBrutaG(bx, by, bz)) return false;

  ax = (bx - offsetX) / ganhoX;
  ay = (by - offsetY) / ganhoY;
  az = (bz - offsetZ) / ganhoZ;
  return true;
}

float calcularAngulo(float ax, float ay, float az, int modo) {
  float ang = 0.0f;
  switch (modo) {
    case 1: ang = atan2f(ax, az) * 180.0f / PI; break;
    case 2: ang = atan2f(ay, az) * 180.0f / PI; break;
    case 3: ang = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / PI; break;
    case 4: ang = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / PI; break;
    default: ang = atan2f(ax, az) * 180.0f / PI; break;
  }
  return OFFSET_TRANSFERIDOR + (SINAL_ANGULO * ang);
}

// ===================== UTILITÁRIOS DE SERIAL =====================
void limparSerial() {
  while (Serial.available()) Serial.read();
}

String lerLinhaSerial() {
  while (!Serial.available()) delay(10);
  String linha = Serial.readStringUntil('\n');
  linha.trim();
  limparSerial();
  return linha;
}

void aguardarEnterComPreview(const char* instrucao) {
  Serial.println();
  Serial.println(instrucao);
  Serial.println("Gire o bloco ate o eixo atingir o extremo pedido, nivele e aperte ENTER:");
  limparSerial();

  while (!Serial.available()) {
    float ax, ay, az;
    if (lerAceleracaoBrutaG(ax, ay, az)) {
      Serial.printf("  [Ao vivo] X: %+.3f g | Y: %+.3f g | Z: %+.3f g\n", ax, ay, az);
    }
    delay(400);
  }
  lerLinhaSerial(); 
}

void coletarMediaBruta(int nAmostras, float &mx, float &my, float &mz) {
  double sx = 0, sy = 0, sz = 0;
  int validas = 0;

  while (validas < nAmostras) {
    float ax, ay, az;
    if (lerAceleracaoBrutaG(ax, ay, az)) {
      sx += ax;
      sy += ay;
      sz += az;
      validas++;
    }
    delay(10);
  }

  mx = (float)(sx / nAmostras);
  my = (float)(sy / nAmostras);
  mz = (float)(sz / nAmostras);
}

// ===================== MODO 1: CALIBRAÇÃO ESTÁTICA =====================
void executarCalibracao() {
  const char* nomesEixo[3] = {"X", "Y", "Z"};
  float aPos[3] = {0, 0, 0};
  float aNeg[3] = {0, 0, 0};
  float ganhos[3] = {1, 1, 1};
  float offsets[3] = {0, 0, 0};

  Serial.println("\n=== CALIBRACAO ESTATICA (+-4g, 6 POSICOES) ===");

  for (int eixo = 0; eixo < 3; eixo++) {
    char msgPos[80], msgNeg[80];
    snprintf(msgPos, sizeof(msgPos), ">>> [%d/6] Eixo +%s (procure o MAIOR valor possivel em %s)", eixo * 2 + 1, nomesEixo[eixo], nomesEixo[eixo]);
    snprintf(msgNeg, sizeof(msgNeg), ">>> [%d/6] Eixo -%s (procure o MENOR valor possivel em %s)", eixo * 2 + 2, nomesEixo[eixo], nomesEixo[eixo]);

    // +1g
    aguardarEnterComPreview(msgPos);
    Serial.println("Coletando 200 amostras...");
    float mx, my, mz;
    coletarMediaBruta(N_AMOSTRAS_CAL, mx, my, mz);
    aPos[eixo] = (eixo == 0) ? mx : (eixo == 1 ? my : mz);
    Serial.printf("   Media +%s -> X:%+.5f | Y:%+.5f | Z:%+.5f\n", nomesEixo[eixo], mx, my, mz);

    // -1g
    aguardarEnterComPreview(msgNeg);
    Serial.println("Coletando 200 amostras...");
    coletarMediaBruta(N_AMOSTRAS_CAL, mx, my, mz);
    aNeg[eixo] = (eixo == 0) ? mx : (eixo == 1 ? my : mz);
    Serial.printf("   Media -%s -> X:%+.5f | Y:%+.5f | Z:%+.5f\n", nomesEixo[eixo], mx, my, mz);

    // Calcula Ganho e Offset do eixo
    ganhos[eixo]  = (aPos[eixo] - aNeg[eixo]) / 2.0f;
    offsets[eixo] = (aPos[eixo] + aNeg[eixo]) / 2.0f;

    Serial.printf("   => Eixo %s: Ganho = %.5f | Offset = %+.5f g\n",
                  nomesEixo[eixo], ganhos[eixo], offsets[eixo]);

    if (ganhos[eixo] < 0.85f || ganhos[eixo] > 1.15f) {
      Serial.println("   [!] ALERTA: Ganho longe de 1.0! (Eixo errado ou sinais trocados?)");
      Serial.println("   Digite 'r' + ENTER para repetir este eixo, ou apenas ENTER para aceitar:");
      String resp = lerLinhaSerial();
      if (resp.equalsIgnoreCase("r")) {
        eixo--;
        continue;
      }
    }
  }

  ganhoX = ganhos[0]; offsetX = offsets[0];
  ganhoY = ganhos[1]; offsetY = offsets[1];
  ganhoZ = ganhos[2]; offsetZ = offsets[2];

  Serial.println("\n=== RESULTADOS DA CALIBRACAO (TABELA PARA O RELATORIO) ===");
  Serial.println("Eixo | Media +1g [g] | Media -1g [g] | Ganho     | Offset [g]");
  Serial.printf("  X  | %+13.5f | %+13.5f | %9.5f | %+10.5f\n", aPos[0], aNeg[0], ganhoX, offsetX);
  Serial.printf("  Y  | %+13.5f | %+13.5f | %9.5f | %+10.5f\n", aPos[1], aNeg[1], ganhoY, offsetY);
  Serial.printf("  Z  | %+13.5f | %+13.5f | %9.5f | %+10.5f\n", aPos[2], aNeg[2], ganhoZ, offsetZ);

  Serial.println("\n--- Copie e cole no topo do codigo ---");
  Serial.printf("float ganhoX = %.6ff, offsetX = %.6ff;\n", ganhoX, offsetX);
  Serial.printf("float ganhoY = %.6ff, offsetY = %.6ff;\n", ganhoY, offsetY);
  Serial.printf("float ganhoZ = %.6ff, offsetZ = %.6ff;\n", ganhoZ, offsetZ);
  Serial.println("--------------------------------------\n");
}

// ===================== MODO 2: COLETA COM TRANSFERIDOR =====================
void executarValidacao() {
  Serial.println("\n=== VALIDACAO COM TRANSFERIDOR (SAIDA CSV) ===");
  Serial.printf("Usando MODO_ANGULO = %d\n", MODO_ANGULO);
  Serial.println("Digite o angulo de referencia e aperte ENTER ('s' para sair):\n");
  Serial.println("ang_ref_deg,ang_med_deg,erro_deg,desvio_padrao_deg");

  while (true) {
    String entrada = lerLinhaSerial();
    if (entrada.equalsIgnoreCase("s")) break;
    if (entrada.length() == 0) continue;

    float angRef = entrada.toFloat();
    double soma = 0, somaQuad = 0;
    int validas = 0;

    while (validas < N_AMOSTRAS_VAL) {
      float ax, ay, az;
      if (lerAceleracaoCalibradaG(ax, ay, az)) {
        float ang = calcularAngulo(ax, ay, az, MODO_ANGULO);
        soma += ang;
        somaQuad += (double)ang * ang;
        validas++;
      }
      delay(10);
    }

    float media = (float)(soma / N_AMOSTRAS_VAL);
    float variancia = (float)((somaQuad / N_AMOSTRAS_VAL) - (media * media));
    float desvio = (variancia > 0) ? sqrtf(variancia) : 0.0f;
    float erro = media - angRef;

    Serial.printf("%.2f,%.3f,%.3f,%.3f\n", angRef, media, erro, desvio);
  }
}

// ===================== MODO 3: TEMPO REAL =====================
void executarTempoReal() {
  Serial.println("\n=== MONITORAMENTO EM TEMPO REAL ===");
  Serial.println("Envie qualquer tecla + ENTER para sair.\n");
  limparSerial();

  while (!Serial.available()) {
    float ax, ay, az;
    if (lerAceleracaoCalibradaG(ax, ay, az)) {
      float mag = sqrtf(ax * ax + ay * ay + az * az);
      float angXZ = calcularAngulo(ax, ay, az, 1);
      float angYZ = calcularAngulo(ax, ay, az, 2);
      float ang3X = calcularAngulo(ax, ay, az, 3);
      float ang3Y = calcularAngulo(ax, ay, az, 4);

      Serial.printf("Cal[g] X:%+.3f Y:%+.3f Z:%+.3f | |A|:%.3f | XZ:%+6.2f | YZ:%+6.2f | 3D_X:%+6.2f | 3D_Y:%+6.2f\n",
                    ax, ay, az, mag, angXZ, angYZ, ang3X, ang3Y);
    }
    delay(200);
  }
  limparSerial();
}

// ===================== MENU PRINCIPAL =====================
void mostrarMenu() {
  Serial.println("\n==========================================");
  Serial.println("  MPU-6050 (+-4g) | INSTRUMENTACAO");
  Serial.println("==========================================");
  Serial.println("1 - Executar calibracao estatica (6 posicoes)");
  Serial.println("2 - Coleta de validacao com transferidor (CSV)");
  Serial.println("3 - Leitura em tempo real (teste / demonstracao)");
  Serial.print("Escolha uma opcao: ");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!inicializarMPU6050()) {
    Serial.println("ERRO: MPU-6050 nao encontrado no I2C (pinos 21/22).");
    while (true) delay(1000);
  }

  Serial.println("MPU-6050 inicializado com sucesso (+-4g, DLPF 5Hz).");
  mostrarMenu();
}

void loop() {
  if (Serial.available()) {
    String op = lerLinhaSerial();
    if (op == "1") executarCalibracao();
    else if (op == "2") executarValidacao();
    else if (op == "3") executarTempoReal();
    mostrarMenu();
  }
}