// LIBRERIAS A USAR:

#include <Wire.h>

// REGISTROS DEL SENSOR MPU6050

const int MPU_ADDR          = 0x68;   // DIRECCION I2C DEL MPU6050.
const int MPU_SMPLRT_DIV    = 0x19;   // CONFIGURACIÓN DE LA TASA DE MUESTREO DEL ACELERÓMETRO Y EL GIROSCOPIO.
const int MPU_CONFIG        = 0x1a;   // CONFIGURACIÓN DEL FILTRO DIGITAL Y DEL RANGO DEL GIROSCOPIO.
const int MPU_GYRO_CONFIG   = 0x1b;   // CONFIGURACIÓN DEL RANGO DE MEDICIÓN DEL GIROSCOPIO.
const int MPU_ACCEL_CONFIG  = 0x1c;   // CONFIGURACIÓN DEL RANGO DE MEDICIÓN DEL ACELERÓMETRO.
const int MPU_WHO_AM_I      = 0x75;   // DIRECCIÓN DEL REGISTRO DE IDENTIFICACIÓN QUE CONTIENE EL VALOR 0X68 (VALOR PREDETERMINADO) PARA VERIFICAR LA PRESENCIA DEL DISPOSITIVO EN EL BUS I2C.
const int MPU_PWR_MGMT_1    = 0x6b;   // CONFIGURACIÓN DEL MODO DE ALIMENTACIÓN Y DEL RELOJ DEL DISPOSITIVO.
const int MPU_ACCEL         = 0x3b;   // REGISTRO DE LECTURAS GIROSCOPIO (ACCEL_XOUT_H HASTA ACCEL_ZOUT_L)
const int MPU_TEMP_H        = 0x41;   // PARTE ALTA DE LA LECTURA DE LA TEMPERATURA DEL DISPOSITIVO.
const int MPU_TEMP_L        = 0x42;   // PARTE BAJA DE LA LECTURA DE LA TEMPERATURA DEL DISPOSITIVO.
const int MPU_GYRO          = 0x43;   // REGISTRO DE LECTURAS GIROSCOPIO (GYRO_XOUT_H HASTA GYRO_ZOUT_L)

// INICIALIZACION DE VARIABLES GLOBALES

float Gy_Y_Offs = 0, Gy_Z_Offs = 0;                         // VARIABLES GLOBALES DE OFFSETS DE GIROSCOPIO
float Ang_Y = 0, Ang_Z = 0, Offs_Ang_Y = 0, Offs_Ang_Z = 0, Corr_Ang_Y = 0, Corr_Ang_Z = 0; // VARIABLES GLOBALES PARA ANGULOS OBTENIDOS POR EL SENSOR
float Temp;                                                 // VARIABLE GLOBAL DE TEMPERATURA
float dt, Tiempo_Prev;                                      // VARIABLES GLOBALES PARA CALCULO DE TEIMPO TRNASCURRIDO EN EL CICLO

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  Begin_MPU();
  Calc_Gyro_Offsets(3000);  // CALCULA LOS OFFSET DEL GIROSCOPIO A PARTIR DE 3000 ITERACIONES
  Get_Offsets_MPU(500);     // CALCULA LOS OFFTES DE LOS ANGULOS A PARTIR DE 500 ITERACIONES
}

void loop() 
{
  Get_Corr_Values_MPU();
  Serial.print("\tANGULO EN Y: ");
  Serial.print(Ang_Y);
  Serial.print("\tANGULO EN Z: ");
  Serial.print(Ang_Z); 
  Serial.print("\tTEMPERATURA: ");
  Serial.println(Temp);
  
}

//FUNCIONES:

void Begin_MPU()                       // FUNCION PARA LA INICIALIZACION DEL SENSOR
{
  Write_MPU(MPU_SMPLRT_DIV, 0x00);     // NO APLICA DIVISOR A LA FREC DE MUESTREO, ESTA QUEDA EN 8kHz.
  Write_MPU(MPU_CONFIG, 0x01);         // INHABILITA LA SINCRONIZACION EXTERNA Y HABLITA EL 1ER EL FILTRO DIGITAL
  Write_MPU(MPU_GYRO_CONFIG, 0x08);    // RANGO DE MEDICION DEL GIROSCOPIO EN ±500 °/s
  Write_MPU(MPU_ACCEL_CONFIG, 0x00);   // RANGO DE MEDICION DEL ACELEROMETRO EN ±2g
  Write_MPU(MPU_PWR_MGMT_1, 0x01);     // ACTIVA EL SENSOR
  Serial.println("****** SENSOR CONFIGURADO *****");
}

void Write_MPU(byte reg, byte dato)     // FUNCION PARA LA ESCRITURA DE DATOS EN EL SENSOR
{
  Wire.beginTransmission(MPU_ADDR);     // INICIA COMUNICACION CON LA DIRECCION DEFINIDA PARA EL SENSOR
  Wire.write(reg);                      // DIRECCION DE REGISTRO QUE SE MODIFICARA 
  Wire.write(dato);                     // DATO QUE SE ESCRIBIRA EN LA DIRECCION MENCIONADA
  Wire.endTransmission();               // TERMINA COMUNICACION
}

byte* Read_MPU(byte reg, int cant_bytes)     // FUNCION PARA LA LECTURA DE VARIOS BYTES CONSECUTIVOS
{
  Wire.beginTransmission(MPU_ADDR);             // INICIA COMUNICACION CON LA DIRECCION DEFINIDA PARA EL SENSOR
  Wire.write(reg);                              // DIRECCION DE REGISTRO QUE SE LEERA
  Wire.endTransmission(true);                   // FINALIZA LA COMUNICACION LUEGO DE QUE SE LEA EL DATO
  byte* datos = new byte [cant_bytes];          // CREA UN ARREGLO DINAMICO DONDE SE ALMACENAN LOS BYTES LEIDOS
  Wire.requestFrom(MPU_ADDR, cant_bytes);       //LEE LOS BYTES DEL REGISTRO ESPECIFICADO
  for(int i=0; i<cant_bytes; i++)               //CICLO FOR PARA ALMACENAR CADA BYTE EN EL ARREGLO DATOS
  {
    datos[i] = Wire.read();                     // ALMACENA CADA DATO EN EL ARREGLO
  }
  return datos;                                 // DEVUELVE EL ARREGLO DATOS CON LA CANTIDAD DE BYTES SELECCIONADA
  // LUEGO DE USAR ESTA FUNCION EL ARREGLO DEBE SER LIBERADO DE LA MEMORIA
  // USANDO LA LINEA DE CODIGO "delete[] nombre del arreglo;" PARA EVITAR FUGAS DE MEMORIA
}

void Calc_Gyro_Offsets(int Cant_Iteraciones)                                // FUNCION PARA CALCULAR EL OFFSET DEL GIROSCOPIO
{
  Serial.println("****** INICIO CALIBRACION DE GIROSCOPIO *****");
  float  X = 0, Y = 0, Z = 0;                                               // INICIA VARIABLES LOCALES EN 0
  //float Gy_Offs [3];
  delay(5000);                                                              //ESPERA 5 SEGUNDOS PARA ESTABILIZAR EL SENSOR
  for(int i = 0; i < Cant_Iteraciones; i++)
  {
    byte* datos = Read_MPU(MPU_GYRO, 6);                                    //LEE LOS VALORES DEL GIROSCOPIO EN LOS 3 EJES
    int16_t Gy_X = datos[0] << 8 | datos[1];                                // GUARDA EN Gy_X LA VELOCIDAD ANGULAR EN X
    int16_t Gy_Y = datos[2] << 8 | datos[3];                                // GUARDA EN Gy_Y LA VELOCIDAD ANGULAR EN Y
    int16_t Gy_Z = datos[4] << 8 | datos[5];                                // GUARDA EN Gy_Z LA VELOCIDAD ANGULAR EN Z
    Y += ((float)Gy_Y) / 65.5;                                              // GUARDA EN X LA VELOCIDAD ANGULAR EN EL EJE Y
    Z += ((float)Gy_Z) / 65.5;                                              // GUARDA EN X LA VELOCIDAD ANGULAR EN EL EJE Z 
    delete[] datos;
  }    
  Gy_Y_Offs = Y / Cant_Iteraciones;                                         // PROMEDIO DE LOS VALORES EN Y
  Gy_Z_Offs = Z / Cant_Iteraciones;                                         // PROMEDIO DE LOS VALORES EN Z
  Serial.println("****** CALIBRACION DE GIROSCOPIO CORRECTA *****");
  delay(3000);
  Serial.println("\tOFFSETS GIROSCOPIO: ");
  Serial.print("\t   OFFSET Y:   ");Serial.print(Gy_Y_Offs);
  Serial.print("\t   OFFSET Z:   ");Serial.println(Gy_Z_Offs);       
}

void Get_Values_MPU(float Gy_Coeff, float Acc_Coeff)                              // FUNCION PARA CALCULAR ANGULOS
{
  // INICIAR VARIABLES LOCALES EN 0
  float Acc_X = 0, Acc_Y = 0, Acc_Z = 0, Gy_Y = 0, Gy_Z = 0;
  float Theta_Acc_Y = 0, Theta_Acc_Z = 0;
  
  byte* datos= Read_MPU(MPU_ACCEL, 14);                                           // LEE VALORES DE ACELEROMETRO, TEMPERATURA Y GIROSCOPIO
  int16_t Acc_X_R = datos[0] << 8 | datos[1];                                     // GUARDA EN Acc_X_R LA VELOCIDAD ANGULAR EN X
  int16_t Acc_Y_R = datos[2] << 8 | datos[3];                                     // GUARDA EN Acc_Y_R LA VELOCIDAD ANGULAR EN Y
  int16_t Acc_Z_R = datos[4] << 8 | datos[5];                                     // GUARDA EN Acc_Z_R LA VELOCIDAD ANGULAR EN Z
  int16_t temp = datos[6] << 8 | datos[7];                                        // GUARDA EN temp EL VALOR DE LA TEMPERATURA
  int16_t Gy_X_R = datos[8] << 8 | datos[9];                                      // GUARDA EN Gy_X_R LA VELOCIDAD ANGULAR EN X
  int16_t Gy_Y_R = datos[10] << 8 | datos[11];                                    // GUARDA EN Gy_Y_R LA VELOCIDAD ANGULAR EN Y
  int16_t Gy_Z_R = datos[12] << 8 | datos[13];                                    // GUARDA EN Gy_Z_R LA VELOCIDAD ANGULAR EN Z 
  delete [] datos;                                                                // ELIMINA VECTOR PARA EVITAR FUGA DE DATOS
  Temp = (temp/340) + 36.53;                                                      // CALCULO DE LA TEMPERATURA EN GRADOS CELSIUS
  Acc_X = ((float)Acc_X_R) / 16384;                                               // GUARDA EN X LA ACELERACION EN EL EJE X
  Acc_Y = ((float)Acc_Y_R) / 16384;                                               // GUARDA EN X LA ACELERACION EN EL EJE Y
  Acc_Z = ((float)Acc_Z_R) / 16384;                                               // GUARDA EN X LA ACELERACION EN EL EJE Z
  Theta_Acc_Y = atan2(Acc_Z, sqrt(Acc_X * Acc_X + Acc_Y * Acc_Y)) * 180 / PI;     // CALCULO DE INCLINACION EN EJE Y
  Theta_Acc_Z = atan2(Acc_Y, sqrt(Acc_X * Acc_X + Acc_Z * Acc_Z)) * (-180 / PI);  // CALCULO DE INCLINACION EN EJE Z
  Gy_Y = ((float)Gy_Y_R) / 65.5;                                                  // GUARDA EN X LA VELOCIDAD ANGULAR EN EL EJE Y
  Gy_Z = ((float)Gy_Z_R) / 65.5;                                                  // GUARDA EN X LA VELOCIDAD ANGULAR EN EL EJE Z
  Gy_Y -= Gy_Y_Offs;                                                              // AJUSTE DE OFFSET EN VALOR DE GIROSCOPIO EJE Y
  Gy_Z -= Gy_Z_Offs;                                                              // AJUSTE DE OFFSET EN VALOR DE GIROSCOPIO EJE Z
  dt = (millis() - Tiempo_Prev) * 0.001;                                          // CALCULO DE dt PARA CALCULO DE ANGULO EN LOS EJES 
  Ang_Y = (Gy_Coeff * (Ang_Y + Gy_Y * dt)) + (Acc_Coeff * Theta_Acc_Y);           // FILTRO COMPLEMENTARIO EN Y
  Ang_Z = (Gy_Coeff * (Ang_Z + Gy_Z * dt)) + (Acc_Coeff * Theta_Acc_Z);           // FILTRO COMPLEMENTARIO EN Z
  Tiempo_Prev = millis();                                                         // CALCULO DE TIEMPO PREVIO PARA CORRECTA MEDICION DEL ANGULO
}
void Get_Offsets_MPU(int Cant_Iteraciones)                        // FUNCION PARA CALCULAR EL OFFSET DE LOS ANGULOS
{
  Serial.println("****** INICIO CALIBRACION DE ANGULOS *****");
  float  Y = 0, Z = 0;                                            // INICIA VARIABLES LOCALES EN 0
  for(int i = 0; i < Cant_Iteraciones; i++)                       //CICLO DE ITERACIONES
  {
    Get_Values_MPU(0.98, 0.02);                                   //CALCULA ANGULOS
    Y += Ang_Y;                                                   // SUMATORIA DE VALORES EN Y
    Z += Ang_Z;                                                   // SUMATORIA DE VALORES EN Z
  }
  Offs_Ang_Y = Y / Cant_Iteraciones;                              // PROMEDIO DE LOS VALORES EN Y
  Offs_Ang_Z = Z / Cant_Iteraciones;                              // PROMEDIO DE LOS VALORES EN Z
  Serial.println("****** CALIBRACION DE ANGULOS CORRECTA *****");
  delay(3000);
  Serial.println("\tOFFSETS ANGULOS: ");
  Serial.print("\t   OFFSET Y:   ");Serial.print(Offs_Ang_Y);
  Serial.print("\t   OFFSET Z:   ");Serial.println(Offs_Ang_Z); 
}
void Get_Corr_Values_MPU()      // FUNCION PARA CALCULAR OFFSET A LOS VALORES MEDIDOS
{
  Get_Values_MPU(0.98, 0.02);
  Corr_Ang_Y = Ang_Y; 
  Corr_Ang_Z = Ang_Z;
  Corr_Ang_Y -= Offs_Ang_Y;     // AJUSTE DE OFFSET EN VALOR DE GIROSCOPIO EJE Y
  Corr_Ang_Y -= Offs_Ang_Z;     // AJUSTE DE OFFSET EN VALOR DE GIROSCOPIO EJE Z 
}

