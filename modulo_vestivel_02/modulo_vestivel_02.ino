/* 
 * I2C device class (I2Cdev) esboço Arduino de demonstração para a classe MPU6050
 * 07/10/2011 por Jeff Rowberg <jeff@rowberg.net>
 * As atualizações devem (espero) estar sempre disponíveis em https://github.com/jrowberg/i2cdevlib
 *   
 * Changelog:
 * 2013-05-08 - adicionados vários formatos de saída
 * - adicionado suporte contínuo Fastwire
 * 07-10-2011 - lançamento inicial
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
 * FORA DE OU EM CONEXÃO COM O SOFTWARE OU O USO OU OUTRAS NEGOCIAÇÕES EM O SOFTWARE.
*/

/* Comunicação Serial (Bluetooth) */
#include "SoftwareSerial.h"

/* I2Cdev e MPU6050 devem ser instalados como bibliotecas, ou então os arquivos .cpp / .h
 * para ambas as classes deve estar no caminho de inclusão do seu projeto */
#include "I2Cdev.h"
#include "MPU6050.h"

/* A biblioteca Arduino Wire é necessária se a implementação I2Cdev I2CDEV_ARDUINO_WIRE é usado em I2Cdev.h */
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* Valor de escala para 16g */
const double ACCEL_SCALE = 2048.0;

/* O endereço I2C padrão da classe é 0x68
 * Endereços I2C específicos podem ser passados ​​como um parâmetro aqui
 * AD0 baixo = 0x68 (padrão para placa de avaliação InvenSense)
 * AD0 alto = 0x69 */
MPU6050 accelgyro;

/* RX - 10, TX - 11 (As ligações devem ser RX com TX e TX com RX) 
 * Bluetooth */
SoftwareSerial serialBluetooth(10, 11);

/* Enviar dados ao App 
 * 0 -> Não enviar
 * 1 -> Enviar
 */
int sendData = 0;

/* Variáveis de aceleração e gyro */
int16_t ax, ay, az;
int16_t gx, gy, gz;

/* Variáveis de aceleração em unidade gravitacional (g) */
double accelX, accelY, accelZ;

/* Tempo de amostra */
long sampleTime = 0;

/* Tempo de leitura */
double readTime = 0;

void setup() {
    /* Junta-se ao barramento I2C (a biblioteca I2Cdev não faz isso automaticamente) */
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    /* Inicializa a comunicação serial */
    Serial.begin(9600);

    /* Inicializa a comunicação serial Bluetooth */
    serialBluetooth.begin(9600);
    
    /* Inicializa o dispositivo */
    Serial.println("Inicializando dispositivos I2C...");
    accelgyro.initialize();

    /* Intervalo de medição inicial */
    Serial.print ("Intervalo inicial =");
    Serial.println (accelgyro.getFullScaleAccelRange());
    Serial.println (accelgyro.getFullScaleGyroRange());
   
    /* Essas linhas definem a escala do acelerômetro para 16g e do giroscópio para 2.000 rads por segundo
     * Essas linhas devem ser colocadas após o método accelgyro.initialize.*/
    accelgyro.setFullScaleAccelRange (MPU6050_ACCEL_FS_16);
    accelgyro.setFullScaleGyroRange (MPU6050_GYRO_FS_2000);

    /* Novo intervalo de medição inicial */
    Serial.print ("Novo intervalo =");
    Serial.println (accelgyro.getFullScaleAccelRange());
    Serial.println (accelgyro.getFullScaleGyroRange());
    
    /* Verifica a conexão */
    Serial.println("Testando conexões de dispositivos...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 conexão com sucesso" : "MPU6050 conexão falhou");

    /* Offsets iniciais */
    Serial.println("Valores iniciais de offset");
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); 
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); 
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); 
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); 
    Serial.print("\n");

    /* Seta os valores de offset pré calculados */
    accelgyro.setXGyroOffset(51);
    accelgyro.setYGyroOffset(-57);
    accelgyro.setZGyroOffset(3);
    accelgyro.setXAccelOffset(-3179);
    accelgyro.setYAccelOffset(1875);
    accelgyro.setZAccelOffset(1118);

    /* Novos offsets */
    Serial.println("Novos valores de offset");
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t");
    Serial.print("\n");

    /* Tempo de Amostra */
    sampleTime = micros();
}

void loop() {

    /* Comando recebido pelo App */
    String comando = "";

    /* Bluetooth recebendo informação */
    if(serialBluetooth.available()){
      /* Lê a informação enquanto estiver recebendo (Pode receber em partes) */
      while(serialBluetooth.available()){
        char caracter = serialBluetooth.read();
        comando += caracter;
        delay(10);
      }
      sendData = comando.toInt();
    }
    
    /* Leitura das medições brutas de aceleração / giroscópio do MPU6050 */
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    /* Tempo atual */
    if(sendData == 0){
      sampleTime = micros();
    }

    /* Tempo da leitura */
    if(sendData == 1){
      /* Segundos */
      readTime = (double) (micros() - sampleTime)/1000000; 
    }

    /* Divide pelo valor de escala para transformar em aceleração gravitacional (g) */
    accelX =  ax / ACCEL_SCALE;
    accelY =  ay / ACCEL_SCALE;
    accelZ =  az / ACCEL_SCALE;

    /* Valores de aceleração (g) no tempo (s) */
    Serial.print(readTime);
    Serial.print("AccelX : "); 
    Serial.print(accelX);
    Serial.print("\tAccelY : "); 
    Serial.print(accelY);
    Serial.print("\tAccelZ : "); 
    Serial.println(accelZ);

    /* Enviar dados ao App */
    if(sendData == 1){
      /* Protocolo */
      serialBluetooth.print("a");
      serialBluetooth.print("{");
      serialBluetooth.print(readTime);
      serialBluetooth.print("|");
      serialBluetooth.print(accelX);
      serialBluetooth.print("|");
      serialBluetooth.print(accelY);
      serialBluetooth.print("|");
      serialBluetooth.print(accelZ);
      serialBluetooth.print("}");
    }
}
