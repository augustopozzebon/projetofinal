#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <Wire.h> 
#include <LiquidCrystal.h>
#include <Adafruit_BMP280.h>
 

/* define - LCD */
#define LCD_16X2_CLEAN_LINE         "                "
#define LCD_RS                      12
#define LCD_EN                      11 
#define LCD_D4                      5 
#define LCD_D5                      4 
#define LCD_D6                      3  
#define LCD_D7                      2 
 
 
/* define - LED */
#define LED_PIN                      LED_BUILTIN
#define LED_THRESHOLD                3.58 /* V
 
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

unsigned status;

 
/* filas (queues) */
QueueHandle_t xQueue_LCD, xQueue_LED, xQueue_SENSOR;
 
/* semaforos utilizados */
SemaphoreHandle_t xSerial_semaphore;
 
void setup() {
  
  /* Inicializa serial (baudrate 19200) */
  Serial.begin(19200);
 
  /* Inicializa o LCD e bmp280, ADICIONAR O ACC */
  lcd.begin(16,2);
  lcd.clear();
  status = bmp.begin();

  /* Inicializa e configura GPIO do LED */ 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  while (!Serial) {
    ; /* Somente vai em frente quando a serial estiver pronta para funcionar */
  }
 
  /* Criação das filas (queues) */ 
  xQueue_LCD = xQueueCreate( 1, sizeof( float ) );
  xQueue_LED = xQueueCreate( 1, sizeof( float ) );
  xQueue_SENSOR = xQueueCreate( 1, sizeof( float ) );
  
 
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
    ,  2                            /* Prioridade */
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
    float pressure = 0.0;
    float temperatura_lida = 0.0;
    
    while(1)
    {   
        
          bmp.setSampling (Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
          Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
          Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
          Adafruit_BMP280::FILTER_X16,      /* Filtering. */
          Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        
          sensors_event_t temp_event, pressure_event;
          bmp_temp->getEvent(&temp_event);
          bmp_pressure->getEvent(&pressure_event);  

          temperatura_lida = temp_event.temperature;
          temperatura_lida = temp_event.temperature;
             
        /* Envia TEMPERATURA para as tarefas a partir de filas */
        xQueueOverwrite(xQueue_LCD, (void *)&temperatura_lida);
        // xQueueOverwrite(xQueue_LED, (void *)&voltage);
        
        /* Espera um segundo */
        vTaskDelay( 100 / portTICK_PERIOD_MS ); 
 
        
    }
}
 


void task_lcd( void *pvParameters )
{
    (void) pvParameters;
    float temp_rcv = 0.0;
     
    while(1)
    {        
        /* Espera até algo ser recebido na queue */
        xQueueReceive(xQueue_LCD, (void *)&temp_rcv, portMAX_DELAY);
        
        /* Uma vez recebida a informação na queue, a escreve no display LCD */
        lcd.setCursor(0,0);
        lcd.print(temp_rcv);
        
        lcd.setCursor(12,1);
        lcd.print("T[C]");
 
        
    }  
}
