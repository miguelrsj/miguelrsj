/*			Sistemas Eletricos e Eletronicos de Veiculos
				LICENCIATURA EM ENGENHARIA AUTOMÓVEL
							MINI PROJETO
				Medição e Controlo de temperatura

AUTORES:
Miguel antunes Nº2191576
Pedro Durão Nº2191593
*/

// inclusao de bibiliotecas
#include "Arduino.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
#include "DHT.h"
#include "BluetoothSerial.h"
//bibliotecas display
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

//definiçao dos pinos Display
#define TFT_DC 21
#define TFT_CS 22
#define TFT_MOSI 23
#define TFT_CLK 18
#define TFT_RST 17
#define TFT_MISO 19
#define DISTC 12
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

//Atribuição de pinos de sensor e atuadores
#define PINO_FAN 2
#define LED_ALARME 4
#define BUZZER 15

//definiçao sensor
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//definiçao bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
BluetoothSerial SerialBT;

//definiçao dos parametros das tarefas
void vTaskReadTemp(void *pvParameters);
void vTaskPWM(void *pvParameters);
void vTaskDisplay(void *pvParameters);
void vTarefaAlarme(void *pvParameters);
void vTaskEscolha(void *pvParameters);
void vTaskControlo(void *pvParameters);

//definiçao de ponteiros de tarefas
const char *pcTextForTaskTemp = "vTaskReadTemp \r\n";
const char *pcTextForTaskFan = "vTaskControlFan\t\n";
const char *pcTextForTaskDisplay = "vTaskdisplay \r\n";
const char *pcTextForTaskAlarme = "vTarefaAlarme\t\n";
const char *pcTextForTaskEscolha = "vTaskEscolha\t\n";

/*----------------------------*/
//definiçao de interupçoes
static void IRAM_ATTR vInterrupcaoALARME(void);

//definiçao do pino para gerar a interrupçao
const uint8_t interruptPin = 0;

//definiçao de semaforos
SemaphoreHandle_t xBinarySemaphore;

//criaçao de queues
QueueHandle_t xQueueSendTemperature;
QueueHandle_t xQueueVerificada;

//pre-definiçao da temperatura de alarme a 20ºC
int temp_alarme = 20;

//criaçao de estrutura para a temperatura
typedef struct {
	float ucValue;
	unsigned char ucSource;
} xData;

/*-------------------------------------------------------------------------------------------------*/
void setup(void) {
	//definiçao de ordem de prioridades
	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

	//definiçao de baudrate
	Serial.begin(115200);

	//inicializaçao do sensor
	dht.begin();

	//criaçao de queues
	xQueueSendTemperature = xQueueCreate(3, sizeof(xData));
	xQueueVerificada = xQueueCreate(3, sizeof(xData));

	//criaçao do semaforo
	vSemaphoreCreateBinary(xBinarySemaphore);

	//configuraçao de interupçao
	pinMode(interruptPin, OUTPUT);
	attachInterrupt(digitalPinToInterrupt(interruptPin), &vInterrupcaoALARME, RISING);

	//configuração do Buzzer
	pinMode(BUZZER, OUTPUT);

	//configuraçao do bluetooth
	SerialBT.begin("ESP32test"); //Bluetooth device name
	Serial.println("The device started, now you can pair it with bluetooth!");

	//Caso o semaforo nao tenha sido iniciado, nao há criação de tarefas
	if (xBinarySemaphore != NULL) {
		//criaçao das tarefas
		xTaskCreatePinnedToCore(vTaskReadTemp, "Temperatura", 1024, NULL, 5, NULL, 1);
		xTaskCreatePinnedToCore(vTaskPWM, "FAN control", 1024, NULL, 4, NULL, 1);
		xTaskCreatePinnedToCore(vTaskDisplay, "Display", 1024, NULL, 2, NULL, 1);
		xTaskCreatePinnedToCore(vTarefaAlarme, "alarme", 1024, NULL, 3, NULL, 1);
		xTaskCreatePinnedToCore(vTaskEscolha, "Escolha", 1024, NULL, 1, NULL, 1);
		xTaskCreatePinnedToCore(vTaskControlo, "Controlo", 1024, NULL, 5, NULL, 1);
	}

	//inicializaçao e configuraçao do display base
	tft.begin();

	// selecionar cor de fundo preta
	tft.fillScreen(ILI9341_BLACK);
	tft.fillScreen(ILI9341_BLACK);

	//definir orientação do display
	tft.setRotation(2);

	//definir posiçao inicial do cursor
	tft.setCursor(0, 10);

	//Definição da cor do texto
	tft.setTextColor(ILI9341_WHITE);

	//Definiçao do tamanho de texto
	tft.setTextSize(2);

	//Imprimir texto no LCD
	tft.println(" Trabalho Pratico ");
	tft.println(" SEEV - 2021/2022 ");

	//Desenhar retangulo em torno do texto escrito
	tft.drawRect(5, 5, 230, 60, ILI9341_RED);
	tft.drawRect(6, 6, 228, 58, ILI9341_RED);
	tft.fillRoundRect(30, 80, 180, 28, 10, ILI9341_BLUE);

	//Escrever título
	tft.setCursor(55, 85);
	tft.println("TEMPERATURA");

	//Desenhar círculo (com espessura)
	tft.fillCircle(60, 200, 48, ILI9341_YELLOW);
	tft.fillCircle(60, 200, 45, ILI9341_BLACK);

	//Tabela de opções de temperatura de alarme
	tft.setTextColor(ILI9341_WHITE);
	tft.setTextSize(2);

	//1 = 17ºC
	tft.setCursor(120, 130);
	tft.println(" 1: 17");
	tft.setCursor(197, 130);
	tft.print((char) 167);
	tft.print("C");

	//2 = 18ºC
	tft.setCursor(120, 150);
	tft.println(" 2: 18 ");
	tft.setCursor(197, 150);
	tft.print((char) 167);
	tft.print("C");

	//3 = 19ºC
	tft.setCursor(120, 170);
	tft.println(" 3: 19 ");
	tft.setCursor(197, 170);
	tft.print((char) 167);
	tft.print("C");

	//4 = 20ºC
	tft.setCursor(120, 190);
	tft.println(" 4: 20 ");
	tft.setCursor(197, 190);
	tft.print((char) 167);
	tft.print("C");

	//5 = 21ºC
	tft.setCursor(120, 210);
	tft.println(" 5: 21 ");
	tft.setCursor(197, 210);
	tft.print((char) 167);
	tft.print("C");

	//6 = 22ºC
	tft.setCursor(120, 230);
	tft.println(" 6: 22 ");
	tft.setCursor(197, 230);
	tft.print((char) 167);
	tft.print("C");

	//7 = 23ºC
	tft.setCursor(120, 250);
	tft.println(" 7: 23 ");
	tft.setCursor(197, 250);
	tft.print((char) 167);
	tft.print("C");
}
/*-------------------------------------------------------------------------------------------------*/

//Funçao de medição de temperatura
void vTaskReadTemp(void *pvParameters) {
	float my_temperature = 0;
	xData xSendTemperature;
	portBASE_TYPE xStatus;

	//Parametros para o vTaskDelayUntil
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {

		//Informar que entrou na funçao
		Serial.println("Entrou Sensor");

		// ler temperatura
		my_temperature = dht.readTemperature();

		//mostra temperatura na porta serie
		Serial.print("temperatura = ");
		Serial.println(my_temperature);

		xSendTemperature.ucValue = my_temperature;

		// envia para para a tarefa brain
		xStatus = xQueueSendToBack(xQueueSendTemperature, &xSendTemperature, 0);

		//verificar se a temperatura foi enviada
		if (xStatus != pdPASS) {
			Serial.println("nao enviado valor para a queue.");
		}

		//Informar que saiu da funçao e aguarda 300ms até entrar novamente
		Serial.println("Saiu Sensor\r\n");
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}
/*-------------------------------------------------------------------------------------------------*/

// tarefa de controlo FAN
void vTaskPWM(void *pvParameters) {
	int freq = 5000;
	int ledChannel = 2;
	int resolution = 8;
	float temperatura = 0;
	xData xReceivedStructure;
	portBASE_TYPE xRecebeFAN;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	//configuração do pino da FAN
	pinMode(PINO_FAN, OUTPUT);
	ledcSetup(ledChannel, freq, resolution);
	ledcAttachPin(PINO_FAN, ledChannel);

	for (;;) {
		Serial.println("Entrou PWM");

		//Recebe e verifica valor temperatura da queue
		xRecebeFAN = xQueuePeek(xQueueVerificada, &xReceivedStructure, 0);
		if (xRecebeFAN != pdPASS) {
			Serial.println("nao recebido valor da queue");
		}
		temperatura = xReceivedStructure.ucValue;

		// Atribuiçao da intensidade da FAN
		//desligada
		if (temperatura <= temp_alarme - 4) {
			ledcWrite(ledChannel, 0);
			Serial.println("FAN desligado");
		}

		//50% de intensidade da ventoinha
		if (temperatura > temp_alarme - 4 && temperatura <= temp_alarme) {
			ledcWrite(ledChannel, 190);
			Serial.println("FAN a 50%");
		}

		//100% de intensidade da ventoinha
		if (temperatura > temp_alarme) {
			ledcWrite(ledChannel, 255);
			Serial.println("FAN a 100%");
		}
		Serial.println("Saiu PWM\r\n");
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}

/*-------------------------------------------------------------------------------------------------*/

// tarefa de controlo do display
void vTaskDisplay(void *pvParameters) {
	float temperatura = 0;
	xData xReceivedStructure;
	portBASE_TYPE xRecebeDisplay;
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	int barra, color;

	for (;;) {
		Serial.println("Entrou Display");

		//recebe e verifica se recebeu o valor da temperatura da tarefa brain
		xRecebeDisplay = xQueueReceive(xQueueVerificada, &xReceivedStructure, 0);
		if (xRecebeDisplay != pdPASS) {
			Serial.print("nao recebido valor da queue ");
		}

		//transformacao da variavel da temperatura em string
		temperatura = xReceivedStructure.ucValue;
		String tempStr = String(temperatura);

		//apresentaçao da temperatura medida
		tft.setCursor(21, 195);
		tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
		tft.setTextSize(2);
		tft.print(tempStr);
		tft.print((char) 167);
		tft.print("C");

		//atribuir cor da barra consoante a temperatura
		if (temperatura < 16) {
			color = ILI9341_BLUE;
		} else {
			if (temperatura > temp_alarme) {
				color = ILI9341_RED;
			} else {
				color = ILI9341_GREEN;
			}
		}

		//Calculo do tamanho da barra
		barra = (temperatura) * (200 / (temp_alarme + 10));

		//Preenchimento da barra
		if (color == ILI9341_RED) {
			tft.fillRect(19, 271, barra, 38, color);

			//limpa a parte que nao é usada da barra
			tft.fillRect(20 + barra, 271, 199 - barra, 38, ILI9341_LIGHTGREY);
		}
		if (color == ILI9341_GREEN) {
			tft.fillRect(19, 271, barra, 38, color);
			tft.fillRect(20 + barra, 271, 199 - barra, 38, ILI9341_LIGHTGREY);
		}
		if (color == ILI9341_BLUE) {
			tft.fillRect(19, 271, barra, 38, color);
			tft.fillRect(20 + barra, 271, 199 - barra, 38, ILI9341_LIGHTGREY);
		}

		Serial.println("Saiu Display\r\n");
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}
/*-------------------------------------------------------------------------------------------------*/

// tarefa de alarme
static void vTarefaAlarme(void *pvParameters) {
	int freq = 5000;
	int ledChannel = 4;
	int resolution = 8;
	float t = 0, temperatura;
	xData xReceivedStructure;
	portBASE_TYPE xRecebe_semaforo, uxPriority;
	TickType_t xLastWakeTime, xStatus;
	xLastWakeTime = xTaskGetTickCount();

	//configuração do pino do LED
	pinMode(LED_ALARME, OUTPUT);
	ledcSetup(ledChannel, freq, resolution);
	ledcAttachPin(LED_ALARME, ledChannel);

	//receção inicial do semaforo
	xSemaphoreTake(xBinarySemaphore, 0);
	for (;;) {
		Serial.println("Entrou Alarme");

		//recebe semaforo
		xStatus = xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);

		if (xStatus == pdPASS) {

			// aumento da prioridade para assegurar que nao é interrompida
			vTaskPrioritySet( NULL, 5 );

			// faz intermitencia do LED e Buzzer
			if (t == 0) {
				ledcWrite(ledChannel, pow(2, resolution));
				digitalWrite(BUZZER, HIGH);
				t = 1;
			} else {
				ledcWrite(ledChannel, 0);
				digitalWrite(BUZZER, LOW);
				t = 0;
			}
		}

		//diminuiçao da prioridade
		vTaskPrioritySet( NULL, 3 );

		Serial.println("Saiu Alarme");
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}
/*-------------------------------------------------------------------------------------------------*/

//tarefa para escolha de temperatura de alarme
static void vTaskEscolha(void *pvParameters) {
	int temp, dadoint;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		Serial.println("entrou escolha");

		//correr quando a porta serie do Bluetooth está ativada
		if (SerialBT.available()) {

			//receção da opção
			temp = SerialBT.read();

			//conversao para inteiro
			if(temp>48 && temp<48+8){
				dadoint = temp - 48;
			}

			Serial.println(dadoint);
			if (dadoint != -35 && dadoint != -38 ) {
				Serial.println(dadoint);

				//atribuiçao de temperatura em funcao da escolha perante a tabela apresentada no Dispaly
				if (dadoint == 1) {
					temp_alarme = 17;
				}
				if (dadoint == 2) {
					temp_alarme = 18;
				}
				if (dadoint == 3) {
					temp_alarme = 19;
				}
				if (dadoint == 4) {
					temp_alarme = 20;
				}
				if (dadoint == 5) {
					temp_alarme = 21;
				}
				if (dadoint == 6) {
					temp_alarme = 22;
				}
				if (dadoint == 7) {
					temp_alarme = 23;
				}
			}
		}
		Serial.println("Saiu escolha\r\n");
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}

/*-------------------------------------------------------------------------------------------------*/

// tarefa de controlo
static void vTaskControlo(void *pvParameters) {
	xData xReceivedStructure;
	xData xSendVerif;
	portBASE_TYPE xStatus, xStatusV;
	float temperatura;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
		Serial.println("Entrou Controlo");
		xStatus = xQueueReceive(xQueueSendTemperature, &xReceivedStructure, 0);

		if (xStatus == pdPASS) {
			Serial.print("temperatura alarme atual: ");
			Serial.println(temp_alarme);
			temperatura = xReceivedStructure.ucValue;
			xSendVerif.ucValue = xReceivedStructure.ucValue;

			// envia para uma queue para as outras tarefas
			xStatusV = xQueueSendToBack(xQueueVerificada, &xSendVerif, 0);

			// verificacao do valor de temperatura
			if (xStatusV == pdPASS) {
				Serial.println("verificado controlo ");
			}

			// ativaçao de interrupçao de alarme
			if (temperatura > temp_alarme) {
				Serial.print("Vai gerar interrupcao\r\n");
				digitalWrite(interruptPin, LOW);
				digitalWrite(interruptPin, HIGH);
				Serial.print("Gerou interrupcao\r\n");
			} else {

				// apagar LED e Buzzer quando alarme desativo
				ledcWrite(4, 0);
				digitalWrite(BUZZER, LOW);
			}
		}
		Serial.println("Saiu Controlo\r\n");
		vTaskDelayUntil(&xLastWakeTime, (300 / portTICK_PERIOD_MS));
	}
}

/*-------------------------------------------------------------------------------------------------*/

// Interrupção para a tarefa de Alarme
static void IRAM_ATTR vInterrupcaoALARME(void) {

	static signed portBASE_TYPE xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

	// dá semaforo pra desbloquear a vTaskAlarme
	xSemaphoreGiveFromISR(xBinarySemaphore, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken);
	Serial.println("Semaforo dado\r\n");

	//caso receba semaforo, sai da interrupção
	if (xHigherPriorityTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR();
	}
}
/*-------------------------------------------------------------------------------------------------*/

void loop() {
	vTaskDelete( NULL);
}
