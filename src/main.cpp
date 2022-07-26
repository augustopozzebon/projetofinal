#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <Wire.h> 
#include <LiquidCrystal.h>
#include <Adafruit_BMP280.h>
 
/* define - LCD */
#define LCD_16X2_CLEAN_LINE                "                "
#define LCD_RS                      12
#define LCD_EN                      11 
#define LCD_D4                      5 
#define LCD_D5                      4 
#define LCD_D6                      3  
#define LCD_D7                      2 
 
/* define - LED */
#define LED_PIN                      LED_BUILTIN
#define LED_THRESHOLD                3.58 /* V
 
/* define - ADC */
#define ADC_MAX                      1023.0
#define MAX_VOLTAGE_ADC              5.0
 
/* tasks */
void task_breathing_light( void *pvParameters );
void task_serial( void *pvParameters );
void task_lcd( void *pvParameters );
void task_sensor( void *pvParameters );
void task_led( void *pvParameters );
 
/* Variaveis relacionadas aos periféricos */
LiquidCrystal lcd(LCD_RS,LCD_EN, LCD_D4, LCD_D5, LCD_D6,LCD_D7);
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
 
/* filas (queues) */
QueueHandle_t xQueue_LCD, xQueue_LED;
 
/* semaforos utilizados */
SemaphoreHandle_t xSerial_semaphore;
 
void setup() {
  
  /* Inicializa serial (baudrate 19200) */
  Serial.begin(19200);
 
  /* Inicializa o LCD, */
  lcd.begin(16,2);
  


  
 
  /* Inicializa e configura GPIO do LED */ 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  while (!Serial) {
    ; /* Somente vai em frente quando a serial estiver pronta para funcionar */
  }
 
  /* Criação das filas (queues) */ 
  xQueue_LCD = xQueueCreate( 1, sizeof( float ) );
  xQueue_LED = xQueueCreate( 1, sizeof( float ) );
 
  /* Criação dos semaforos */
  xSerial_semaphore = xSemaphoreCreateMutex();
 
  if (xSerial_semaphore == NULL)
  {
     Serial.println("Erro: nao e possivel criar o semaforo");
     while(1); /* Sem semaforo o funcionamento esta comprometido. Nada mais deve ser feito. */
  }
  
  /* Criação das tarefas */
  xTaskCreate(
    task_sensor                     /* Funcao a qual esta implementado o que a tarefa deve fazer */
    ,  "sensor"                     /* Nome (para fins de debug, se necessário) */
    ,  128                          /* Tamanho da stack (em words) reservada para essa tarefa */
    ,  NULL                         /* Parametros passados (nesse caso, não há) */
    ,  3                            /* Prioridade */
    ,  NULL );                      /* Handle da tarefa, opcional (nesse caso, não há) */
 
  xTaskCreate(
    task_lcd
    ,  "LCD"
    ,  156  
    ,  NULL
    ,  2 
    ,  NULL );
 
 
 
  /* A partir deste momento, o scheduler de tarefas entra em ação e as tarefas executam */
}
 




void loop()
{
  /* Tudo é executado nas tarefas. Há nada a ser feito aqui. */
}
 




/* --------------------------------------------------*/
/* ---------------------- Tarefas -------------------*/
/* --------------------------------------------------*/
 
void task_sensor( void *pvParameters )
{
    (void) pvParameters;
        UBaseType_t uxHighWaterMark;
    float pressure = 0.0;
    float temperatura_lida= 0.0;
    
    while(1)
    {   
        

        sensors_event_t temp_event, pressure_event;
        bmp_temp->getEvent(&temp_event);
        bmp_pressure->getEvent(&pressure_event);  

         



        /* Envia tensão lida em A0 para as tarefas a partir de filas */
        xQueueOverwrite(xQueue_LCD, (void *)&temperatura_lida);
        // xQueueOverwrite(xQueue_LED, (void *)&voltage);
        
        /* Espera um segundo */
        vTaskDelay( 1000 / portTICK_PERIOD_MS ); 
 
      


    if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
                }


              /* Para fins de teste de ocupação de stack, printa na serial o high water mark */
        xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        Serial.print("task_sensor high water mark (words): ");
        Serial.println(uxHighWaterMark);
        Serial.println("---");
        xSemaphoreGive(xSerial_semaphore);
    }
}
 






void task_lcd( void *pvParameters )
{
    (void) pvParameters;
    float temp_rcv = 0.0;
    UBaseType_t uxHighWaterMark;
 
    while(1)
    {        
        /* Espera até algo ser recebido na queue */
        xQueueReceive(xQueue_LCD, (void *)&temp_rcv, portMAX_DELAY);
        
        /* Uma vez recebida a informação na queue, a escreve no display LCD */
        lcd.setCursor(0,0);
        lcd.print(temp_rcv);
       
        lcd.setCursor(15,1);
        lcd.print("V");
 
        /* Para fins de teste de ocupação de stack, printa na serial o high water mark */
        xSemaphoreTake(xSerial_semaphore, portMAX_DELAY );
        
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        Serial.print("task_lcd high water mark (words): ");
        Serial.println(uxHighWaterMark);
        Serial.println("---");
        xSemaphoreGive(xSerial_semaphore);
    }  
}
