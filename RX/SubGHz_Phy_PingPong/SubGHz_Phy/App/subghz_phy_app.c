/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"
#include "aes.h"
#include "stdio.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_SEND_REQUEST,
  APP_WAIT_RESPONSE,
  APP_RESPONSE_OK,
  APP_RETRY_REQUEST,
} AppState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000
/*Size of the payload to be sent*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
#define REQUEST_PERIOD_MS             3000
#define REQUEST_RETRY_DELAY_MS        1000
#define REQUEST_NODE_COUNT            4U
#define PROTO_MAGIC                   0xA5U
#define PROTO_TYPE_REQUEST            0x01U
#define PROTO_TYPE_DATA               0x02U
#define PROTO_REQUEST_SIZE            5U
#define PROTO_DATA_HEADER_SIZE        9U
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333
/* LED blink Period*/
#define LED_PERIOD_MS                 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
/* RX controller FSM state */
static AppState_t AppState = APP_SEND_REQUEST;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/* Led Timers objects*/
static UTIL_TIMER_Object_t timerLed;
/* device state. Master: true, Slave: false*/
bool isMaster = false;
static int32_t random_delay;
static const uint8_t requestNodeIds[REQUEST_NODE_COUNT] = {236U, 237U, 238U, 239U};
static uint8_t requestNodeIndex = 0;
static uint8_t requestedNodeId = 0;
static uint16_t requestSeq = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  Function executed on when led timer elapses
  * @param  context ptr of LED context
  */
static void OnledEvent(void *context);

/**
  * @brief PingPong state machine implementation
  */
static void PingPong_Process(void);

/**
  * @brief Update the currently requested node from the round-robin list
  */
static void UpdateRequestedNode(void);

/**
  * @brief Build a request packet for the selected node
  */
static uint8_t PrepareRequestPayload(void);

/**
  * @brief Controller TX configure and process
  */
static void RadioSend(uint8_t size);

/**
  * @brief PingPong RX configure and process
  */
static void RadioRx(void);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  APP_LOG(TS_OFF, VLEVEL_M, "\n\rRX CONTROLLER\n\r");
  /* Get SubGHY_Phy APP version*/
  APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
          (uint8_t)(APP_VERSION_MAIN),
          (uint8_t)(APP_VERSION_SUB1),
          (uint8_t)(APP_VERSION_SUB2));

  /* Get MW SubGhz_Phy info */
  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
          (uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
          (uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

  /* Led Timers*/
  UTIL_TIMER_Create(&timerLed, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledEvent, NULL);
  UTIL_TIMER_Start(&timerLed);
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);
  MX_AES_Init();

  /* USER CODE BEGIN SubghzApp_Init_2 */
  random_delay = (Radio.Random()) >> 22;
  UpdateRequestedNode();

  /* Radio Set frequency */
  Radio.SetChannel(RF_FREQUENCY);

  /* Radio configuration */
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_BW=%d kHz\n\r", (1 << LORA_BANDWIDTH) * 125);
  APP_LOG(TS_OFF, VLEVEL_M, "LORA_SF=%d\n\r", LORA_SPREADING_FACTOR);
#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
  APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);
#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  /*fills tx buffer*/
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

  APP_LOG(TS_ON, VLEVEL_L, "rand=%d\n\r", random_delay);

  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);

  HAL_Delay(RX_TIMEOUT_VALUE + random_delay);
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_L, "Request sent\n\r");
  AppState = APP_WAIT_RESPONSE;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}

static void AES_SetCounter(uint32_t counter)
{
  static uint32_t iv[4];

  iv[0] = 0x00000000;
  iv[1] = 0x00000000;
  iv[2] = 0x00000000;
  iv[3] = counter;

  hcryp.Init.pInitVect = iv;
  hcryp.Init.KeyIVConfigSkip = CRYP_KEYIVCONFIG_ALWAYS;

  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  uint32_t rxCounter = 0;
  uint16_t responseSeq = 0;
  char line[200];
  int pos = 0;
  uint16_t cipher_len = 0;
  uint8_t plaintext[MAX_APP_BUFFER_SIZE] = {0};

  APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\r\n");

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\r\n", rssi, LoraSnr_FskCfo);
  SnrValue = LoraSnr_FskCfo;
#endif

#if ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, Cfo=%dkHz\r\n", rssi, LoraSnr_FskCfo);
  SnrValue = 0;
#endif

  RssiValue = rssi;
  RxBufferSize = size;

  memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);

  if (RxBufferSize > MAX_APP_BUFFER_SIZE)
  {
    RxBufferSize = MAX_APP_BUFFER_SIZE;
  }

  memcpy(BufferRx, payload, RxBufferSize);

  APP_LOG(TS_ON, VLEVEL_M, "Size received = %u\r\n", RxBufferSize);

//  pos = 0;
//  pos += snprintf(&line[pos], sizeof(line) - pos, "Packet: ");
//  for (uint16_t i = 0; i < RxBufferSize && pos < (int)sizeof(line) - 4; i++)
//  {
//    pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", BufferRx[i]);
//  }
//  snprintf(&line[pos], sizeof(line) - pos, "\r\n");
//  APP_LOG(TS_ON, VLEVEL_M, "%s", line);

  if (RxBufferSize < PROTO_DATA_HEADER_SIZE)
  {
    APP_LOG(TS_ON, VLEVEL_M, "Packet too small\r\n");
    AppState = APP_WAIT_RESPONSE;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    return;
  }

  if ((BufferRx[0] != PROTO_MAGIC) || (BufferRx[1] != PROTO_TYPE_DATA))
  {
    APP_LOG(TS_ON, VLEVEL_M, "Ignoring packet with invalid protocol header\r\n");
    AppState = APP_WAIT_RESPONSE;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    return;
  }

  if (BufferRx[2] != requestedNodeId)
  {
    APP_LOG(TS_ON, VLEVEL_M, "Ignoring response from node %u, waiting for %u\r\n",
            (unsigned int)BufferRx[2],
            (unsigned int)requestedNodeId);
    AppState = APP_WAIT_RESPONSE;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    return;
  }

  responseSeq = (uint16_t)BufferRx[3] | ((uint16_t)BufferRx[4] << 8);
  if (responseSeq != requestSeq)
  {
    APP_LOG(TS_ON, VLEVEL_M, "Ignoring response seq %u, waiting for seq %u\r\n",
            (unsigned int)responseSeq,
            (unsigned int)requestSeq);
    AppState = APP_WAIT_RESPONSE;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    return;
  }

  cipher_len = RxBufferSize - PROTO_DATA_HEADER_SIZE;

  if ((cipher_len % 16) != 0)
  {
	  APP_LOG(TS_ON, VLEVEL_M, "Invalid ciphertext size: %u\r\n", cipher_len);
      AppState = APP_WAIT_RESPONSE;
	  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	  return;
  }

  memcpy(&rxCounter, &BufferRx[5], 4);
  APP_LOG(TS_ON, VLEVEL_M, "counter: %u\r\n", (unsigned int)rxCounter);
  APP_LOG(TS_ON, VLEVEL_M, "Response from node %u for seq %u\r\n",
          (unsigned int)requestedNodeId,
          (unsigned int)responseSeq);

  MX_AES_Init();
  AES_SetCounter(rxCounter);

  if (HAL_CRYP_Encrypt(&hcryp,
                       (uint32_t *)&BufferRx[PROTO_DATA_HEADER_SIZE],
                       cipher_len / 4,
                       (uint32_t *)plaintext,
                       HAL_MAX_DELAY) != HAL_OK)
  {
    APP_LOG(TS_ON, VLEVEL_M, "AES CTR process error\r\n");
    AppState = APP_WAIT_RESPONSE;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    return;
  }

  if (cipher_len < MAX_APP_BUFFER_SIZE)
  {
	  plaintext[cipher_len] = '\0';
  }
  else
  {
	  plaintext[MAX_APP_BUFFER_SIZE -1] = '\0';
  }

  APP_LOG(TS_ON, VLEVEL_M, "Plaintext ASCII: %s\r\n", plaintext);

  pos = 0;
  pos += snprintf(&line[pos], sizeof(line) - pos, "Plaintext HEX: ");
  for (uint8_t i = 0; i < cipher_len && pos < (int)sizeof(line) - 4; i++)
  {
    pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", plaintext[i]);
  }
  snprintf(&line[pos], sizeof(line) - pos, "\r\n");
  APP_LOG(TS_ON, VLEVEL_M, "%s", line);

  AppState = APP_RESPONSE_OK;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "Request Tx timeout\n\r");
  AppState = APP_RETRY_REQUEST;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "Response timeout\n\r");
  AppState = APP_RETRY_REQUEST;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  APP_LOG(TS_ON, VLEVEL_L, "Response Rx error\n\r");
  AppState = APP_RETRY_REQUEST;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
static void UpdateRequestedNode(void)
{
  requestedNodeId = requestNodeIds[requestNodeIndex];
}

static uint8_t PrepareRequestPayload(void)
{
  memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);
  BufferTx[0] = PROTO_MAGIC;
  BufferTx[1] = PROTO_TYPE_REQUEST;
  BufferTx[2] = requestedNodeId;
  BufferTx[3] = (uint8_t)(requestSeq & 0xFFU);
  BufferTx[4] = (uint8_t)((requestSeq >> 8) & 0xFFU);

  APP_LOG(TS_ON, VLEVEL_M, "Requesting node %u with seq %u\r\n",
          (unsigned int)requestedNodeId,
          (unsigned int)requestSeq);

  return PROTO_REQUEST_SIZE;
}

static void RadioSend(uint8_t size)
{
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  Radio.Sleep();
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, TX_TIMEOUT_VALUE);
  Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);
#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  Radio.Send(BufferTx, size);
}

static void RadioRx(void)
{
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  Radio.Sleep();
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  if (LORA_FIX_LENGTH_PAYLOAD_ON == true)
  {
	Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  }
  else
  {
    Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  }
#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                    0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                    0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                    0, 0, false, true);
  if (LORA_FIX_LENGTH_PAYLOAD_ON == true)
  {
    Radio.SetMaxPayloadLength(MODEM_FSK, PAYLOAD_LEN);
  }
  else
  {
    Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);
  }
#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  Radio.Rx(RX_TIMEOUT_VALUE);
}

static void PingPong_Process(void)
{
  Radio.Sleep();

  switch (AppState)
  {
    case APP_SEND_REQUEST:
      RadioSend(PrepareRequestPayload());
      break;

    case APP_WAIT_RESPONSE:
      APP_LOG(TS_ON, VLEVEL_M, "Waiting for response from node %u seq %u\r\n",
              (unsigned int)requestedNodeId,
              (unsigned int)requestSeq);
      RadioRx();
      break;

    case APP_RESPONSE_OK:
      APP_LOG(TS_ON, VLEVEL_M, "Response handled, scheduling next request\r\n");
      HAL_Delay(REQUEST_PERIOD_MS);
      requestSeq++;
      requestNodeIndex = (uint8_t)((requestNodeIndex + 1U) % REQUEST_NODE_COUNT);
      UpdateRequestedNode();
      AppState = APP_SEND_REQUEST;
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
      break;

    case APP_RETRY_REQUEST:
      APP_LOG(TS_ON, VLEVEL_M, "Retrying request for node %u seq %u\r\n",
              (unsigned int)requestedNodeId,
              (unsigned int)requestSeq);
      HAL_Delay(REQUEST_RETRY_DELAY_MS);
      AppState = APP_SEND_REQUEST;
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
      break;

    default:
      APP_LOG(TS_ON, VLEVEL_M, "Unknown state, restarting request loop\r\n");
      AppState = APP_SEND_REQUEST;
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
      break;
  }
}

static void OnledEvent(void *context)
{
  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* LED_GREEN */
  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* LED_RED */
  UTIL_TIMER_Start(&timerLed);
}

/* USER CODE END PrFD */


