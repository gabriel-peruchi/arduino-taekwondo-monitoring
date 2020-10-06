/* 
 * MPU6050 offset-finder, baseado no MPU6050_RAW de Jeff Rowberg
 * 2016-10-19 por Robert R. Fenichel (bob@fenichel.net) 
*/
 
/* 
 * Esboço do Arduino de demonstração da classe de dispositivo I2C (I2Cdev) para a classe MPU6050
 * 07/10/2011 por Jeff Rowberg <jeff@rowberg.net>
 * As atualizações devem (espero) estar sempre disponíveis em https://github.com/jrowberg/i2cdevlib
 *
 * Changelog:
 *          11/07/2019 - adicionada geração de deslocamento de PID no início. Gera os primeiros deslocamentos
 *                     - em @ 6 segundos e completa com mais 4 séries em 10 segundos
 *                     - então continua com o código de calibração original de 2016.
 *          2016-11-25 - atrasos adicionados para reduzir a taxa de amostragem para ~ 200 Hz
 *                     - adição de impressão temporizadora durante cálculos longos
 *          2016-10-25 - requer desigualdade (baixa <meta, alta> meta) durante a expansão
 *                     - mudança de velocidade dinâmica ao fechar
 *          22/10/2016 - mudanças cosméticas
 *          2016-10-19 - versão inicial do IMU_Zero
 *          08/05/2013 - adicionados vários formatos de saída
 *                     - adicionado suporte Fastwire contínuo
 *          07-10-2011 - versão inicial de MPU6050_RAW
*/ 
   
/* 
 * O código da biblioteca do dispositivo I2Cdev é colocado sob a licença MIT Copyright (c) 2011 Jeff Rowberg
 *
 * A permissão é concedida, gratuitamente, a qualquer pessoa que obtenha uma cópia
 * deste software e arquivos de documentação associados (o "Software"), para lidar
 * no Software sem restrição, incluindo, sem limitação, os direitos
 * para usar, copiar, modificar, mesclar, publicar, distribuir, sublicenciar e / ou vender
 * cópias do Software, e para permitir as pessoas a quem o Software é
 * fornecido para fazê-lo, sujeito às seguintes condições:
 *
 * O aviso de direitos autorais acima e este aviso de permissão devem ser incluídos em
 * todas as cópias ou partes substanciais do Software.
 *
 * O SOFTWARE É FORNECIDO "COMO ESTÁ", SEM GARANTIA DE QUALQUER TIPO, EXPRESSA OU
 * IMPLÍCITA, INCLUINDO, MAS NÃO SE LIMITANDO ÀS GARANTIAS DE COMERCIALIZAÇÃO,
 * ADEQUAÇÃO A UMA FINALIDADE ESPECÍFICA E NÃO VIOLAÇÃO. EM NENHUMA HIPÓTESE O
 * AUTORES OU TITULARES DE DIREITOS AUTORAIS SÃO RESPONSÁVEIS POR QUALQUER RECLAMAÇÃO, DANOS OU OUTROS
 * RESPONSABILIDADE, SEJA EM AÇÃO DE CONTRATO, DELITO OU DE OUTRA FORMA, DECORRENTE DE,
 * FORA DE OU EM CONEXÃO COM O SOFTWARE OU O USO OU OUTRAS NEGOCIAÇÕES EM
 * O SOFTWARE.
 *
 * Se um MPU6050
 *     - é um membro ideal de sua tribo,
 *     - está devidamente aquecido,
 *     - está em repouso em uma posição neutra,
 *     - está em um local onde a atração da gravidade é exatamente 1g, e
 *     - foi carregado com os melhores deslocamentos possíveis,
 * então ele reportará 0 para todas as acelerações e deslocamentos, exceto para
 * Aceleração Z, para a qual relatará 16384 (ou seja, 2 ^ 14). Seu dispositivo
 * provavelmente não funcionará muito bem, mas bons offsets obterão a linha de base
 * saídas próximas a esses valores alvo.
 *
 * Coloque o MPU6050 em uma superfície plana e horizontal e deixe-o funcionando por
 * 5-10 minutos para que sua temperatura se estabilize.
 *
 * Execute este programa. Uma linha "----- feito -----" indicará que fez o seu melhor.
 * Com as constantes atuais relacionadas à precisão (NFast = 1000, NSlow = 10000), levará
 * alguns minutos para chegar lá.
 *
 * Ao longo do caminho, ele irá gerar uma dúzia ou mais de linhas de saída, mostrando que para cada
 * dos 6 deslocamentos desejados, é
 *     - primeiro, tentando encontrar duas estimativas, uma muito baixa e outra muito alta, e
 *     - então, fechando até que o colchete não possa ser menor.
 *
 * A linha logo acima da linha "concluída" será semelhante a [567,567] -> [-1,2] [-2223, -2223] -> [0,1] [1131,1132] -> [16374,16404] [155,156] -> [-1, 1] [-25, -24] -> [0,3] [5,6] -> [0,4]
 * Como terá sido mostrado nas linhas de cabeçalho intercaladas, os seis grupos que compõem este
 * linha descreve os deslocamentos ideais para a aceleração X, aceleração Y, aceleração Z,
 * Giroscópio X, giroscópio Y e giroscópio Z, respectivamente. Na amostra mostrada logo acima, o ensaio mostrou
 * que +567 foi o melhor deslocamento para a aceleração X, -2223 foi o melhor para a aceleração Y, e assim por diante.
 * 
 * A necessidade do atraso entre as leituras (usDelay) foi trazida à minha atenção por Nikolaus Doppelhammer.
*/

/* I2Cdev e MPU6050 devem ser instalados como bibliotecas, ou então os arquivos .cpp / .h
   para ambas as classes deve estar no caminho de inclusão do seu projeto */
#include "I2Cdev.h"
#include "MPU6050.h"

/* A biblioteca Arduino Wire é necessária se a implementação I2Cdev I2CDEV_ARDUINO_WIRE é usado em I2Cdev.h */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* O endereço I2C padrão da classe é 0x68
 * endereços I2C específicos podem ser passados como um parâmetro aqui
 * AD0 baixo = 0x68 (padrão para placa de avaliação InvenSense)
 * AD0 alto = 0x69 */
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- usar para AD0 alto

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empírico, para manter a amostragem a 200 Hz
const int NFast =  1000;    // quanto maior, melhor (mas mais lento)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
      int LowValue[6];
      int HighValue[6];
      int Smoothed[6];
      int LowOffset[6];
      int HighOffset[6];
      int Target[6];
      int LinesOut;
      int N;
      
void ForceHeader() { 
  LinesOut = 99; 
}
    
void GetSmoothed() { 
    int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
      { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
      { // get sums
        accelgyro.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
          Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
      } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
      { Smoothed[i] = (Sums[i] + N/2) / N ; }
} // GetSmoothed

void Initialize() {
    /* junte-se ao barramento I2C (a biblioteca I2Cdev não faz isso automaticamente) */
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);

    // inicializar dispositivo
    Serial.println("Inicializando dispositivos I2C ...");
    accelgyro.initialize();

    // verificar a conexão
    Serial.println("Testando conexões do dispositivo ...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection bem-sucedida": "MPU6050 connection failed");
    Serial.println("Ajuste PID a cada ponto = 100 leituras");
  
    /*
      Uma informação sobre como funciona a sintonização PID (PI).
      Quando alteramos o deslocamento no MPU6050, podemos obter resultados instantâneos. Isso nos permite usar Proporcional e
      integral do PID para descobrir os deslocamentos ideais. Integral é a chave para descobrir esses deslocamentos, Integral
      usa o erro do ponto de ajuste (ponto de ajuste é zero), pega uma fração deste erro (erro * ki) e adiciona
      para o valor integral. Cada leitura reduz o erro ao deslocamento desejado. Quanto maior o erro de
      ponto de ajuste, mais ajustamos o valor integral. O proporcional faz a sua parte, escondendo o ruído do
      matemática integral. O Derivativo não é usado por causa do ruído e porque o sensor está parado. Com o
      ruído removido, o valor integral cai em um deslocamento sólido após apenas 600 leituras. No final de cada conjunto de 100
      leituras, o valor integral é usado para os deslocamentos reais e a última leitura proporcional é ignorada devido a
      o fato de que reage a qualquer ruído.
    */
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        Serial.println("\nem 600 leituras");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("700 leituras totais");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("800 leituras totais");
        accelgyro.PrintActiveOffsets();
        Serial.println();
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("900 leituras totais");
        accelgyro.PrintActiveOffsets();
        Serial.println();    
        accelgyro.CalibrateAccel(1);
        accelgyro.CalibrateGyro(1);
        Serial.println("1000 leituras totais");
        accelgyro.PrintActiveOffsets();
     Serial.println("\n\n Qualquer um dos deslocamentos acima funcionará bem \n \n Vamos testar o ajuste do PID usando outro método:"); 
} // Inicializar

void SetOffsets(int TheOffsets[6]) { 
    accelgyro.setXAccelOffset(TheOffsets [iAx]);
    accelgyro.setYAccelOffset(TheOffsets [iAy]);
    accelgyro.setZAccelOffset(TheOffsets [iAz]);
    accelgyro.setXGyroOffset (TheOffsets [iGx]);
    accelgyro.setYGyroOffset (TheOffsets [iGy]);
    accelgyro.setZGyroOffset (TheOffsets [iGz]);
} // Set Offsets

void ShowProgress() { 
  if (LinesOut >= LinesBetweenHeaders)
      { // Mostrar Cabeçalho
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
      } // Mostrar Cabeçalho
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
      { Serial.print(LBRACKET);
        Serial.print(LowOffset[i]),
        Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz)
          { Serial.println(RBRACKET); }
        else
          { Serial.print("]\t"); }
      }
    LinesOut++;
} // Mostrar progresso

void PullBracketsIn() { 
    boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];
  
    Serial.println("\nfechando em:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking) 
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
          { SetAveraging(NSlow); }
        else
          { AllBracketsNarrow = true; }// Tentativa
        for (int i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // busca binária
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // busca binária
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // fechando em
            if (Smoothed[i] > Target[i])
              { // use a metade inferior
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use a metade inferior
            else
              { // use a metade superior
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use a metade superior
          } // fechando em
        ShowProgress();
      } // ainda trabalhando
} // Puxar os colchetes

void PullBracketsOut() { 
    boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();
 
    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // obteve valores baixos
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // obteve valores baixos
      
        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // tem valores altos
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // tem valores altos
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // teve que esperar até ShowProgress terminar
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // continue
} // Puxe os suportes para fora

void SetAveraging(int NewN) { 
    N = NewN;
    Serial.print("média ");
    Serial.print(N);
    Serial.println(" leituras de cada vez");
} // Definir a média

void setup() { 
    Initialize();
    for (int i = iAx; i <= iGz; i++)
      { // definir metas e suposições iniciais
        Target[i] = 0; // deve corrigir para ZAccel 
        HighOffset[i] = 0;
        LowOffset[i] = 0;
      } // definir metas e suposições iniciais
    Target[iAz] = 16384;
    SetAveraging(NFast);
    
    PullBracketsOut();
    PullBracketsIn();
    
    Serial.println("-------------- feito --------------");
} // setup
 
void loop(){} // loop
