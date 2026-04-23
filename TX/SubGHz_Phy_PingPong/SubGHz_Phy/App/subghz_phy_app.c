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
#include "stm32_seq.h"
#include "aes.h"
#include <string.h>
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

extern char tx_message[64];
extern uint8_t id_node;
extern void tx_prepare_message(void);

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  APP_LISTEN,
  APP_SEND_REPLY,
  APP_REPLY_RETRY
} AppState_t;

static AppState_t AppState = APP_LISTEN;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
/*Timeout*/
#define TX_TIMEOUT_VALUE              3000
#define RX_TIMEOUT_VALUE              3000
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#define REPLY_RETRY_DELAY_MS         1000

#define TX_COUNTER_SIZE 4U
#define TX_MAX_MESSAGE_SIZE 64U
#define PROTO_MAGIC                  0xA5U
#define PROTO_TYPE_REQUEST           0x01U
#define PROTO_TYPE_DATA              0x02U
#define PROTO_REQUEST_SIZE           5U
#define PROTO_DATA_HEADER_SIZE       9U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
static uint32_t txCounter = 0;
static uint16_t pendingRequestSeq = 0;
static uint8_t pendingReplySize = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);
/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

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
  * @brief  AES set counter
  */
static void AES_LoadCounter(uint32_t counter);

/**
  * @brief Tx prepare payload
  */
static uint8_t PrepareTxPayload(void);

/**
  * @brief  TX configure and process send
  */
static void RadioSend(uint8_t size);

/**
  * @brief  TX configure and process RX
  */
static void RadioRx(void);

/**
  * @brief TX configure and process
  */
static void AppProcess(void);

/**
  * @brief Call the send of the packet
  */
static void SendNow(void);

static void DWT_Init(void);


/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */
  APP_LOG(TS_OFF, VLEVEL_M, "\n\rTX APP\n\r");

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);
  MX_AES_Init();
  DWT_Init();

  /* USER CODE BEGIN SubghzApp_Init_2 */

  /* Radio Set frequency */
  Radio.SetChannel(RF_FREQUENCY);

  /*register task to to be run in while(1) after Radio IT*/
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
                     UTIL_SEQ_RFU,
                     AppProcess);

  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  APP_LOG(TS_ON, VLEVEL_L, "Tx Done\n\r");
  /* Update the State of the FSM*/
  AppState = APP_LISTEN;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  APP_LOG(TS_ON, VLEVEL_L, "Tx Timeout\n\r");
  /* Update the State of the FSM*/
  AppState = APP_REPLY_RETRY;
  /* Run PingPong process in background*/
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  uint16_t requestSeq = 0;
  uint8_t requestedNodeId = 0;

  APP_LOG(TS_ON, VLEVEL_L, "Rx Done\n\r");

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\r\n", rssi, LoraSnr_FskCfo);
#endif

  if ((size != PROTO_REQUEST_SIZE) || (payload[0] != PROTO_MAGIC) || (payload[1] != PROTO_TYPE_REQUEST))
  {
    APP_LOG(TS_ON, VLEVEL_M, "Ignoring non-request packet\r\n");
    AppState = APP_LISTEN;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    return;
  }

  requestedNodeId = payload[2];
  requestSeq = (uint16_t)payload[3] | ((uint16_t)payload[4] << 8);

  APP_LOG(TS_ON, VLEVEL_M, "Request for node %u seq %u\r\n",
          (unsigned int)requestedNodeId,
          (unsigned int)requestSeq);

  if (requestedNodeId != id_node)
  {
    APP_LOG(TS_ON, VLEVEL_M, "Request not for this node (%u)\r\n", (unsigned int)id_node);
    AppState = APP_LISTEN;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    return;
  }

  pendingRequestSeq = requestSeq;
  AppState = APP_SEND_REPLY;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnRxTimeout(void)
{
  APP_LOG(TS_ON, VLEVEL_L, "Rx Timeout\n\r");
  AppState = APP_LISTEN;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}

static void OnRxError(void)
{
  APP_LOG(TS_ON, VLEVEL_L, "Rx Error\n\r");
  AppState = APP_LISTEN;
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
}


/* USER CODE BEGIN PrFD */

static void AES_LoadCounter(uint32_t counter)
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

static uint8_t PrepareTxPayload(void)
{
  /* time measure */

  uint32_t prepare_start, prepare_end, prepare_cycles;
  uint32_t enc_start, enc_end, enc_cycles;

  tx_prepare_message();

  APP_LOG(TS_ON, VLEVEL_M, "message: %s\r\n", tx_message);
  uint32_t counter = txCounter++;
  APP_LOG(TS_ON, VLEVEL_M, "counter: %u\r\n", (unsigned int)counter);
  APP_LOG(TS_ON, VLEVEL_M, "replying as node %u for seq %u\r\n",
          (unsigned int)id_node,
          (unsigned int)pendingRequestSeq);

  prepare_start = DWT->CYCCNT;

  uint8_t msg_len = (uint8_t)strnlen(tx_message, TX_MAX_MESSAGE_SIZE);
  uint8_t enc_len = (uint8_t)(((msg_len + 15U) / 16U) * 16U);

  uint8_t plain[TX_MAX_MESSAGE_SIZE] = {0};
  uint8_t cipher[TX_MAX_MESSAGE_SIZE] = {0};

  if (msg_len == 0U)
  {
    APP_LOG(TS_ON, VLEVEL_M, "Empty message\r\n");
    return 0;
  }

  memcpy(plain, tx_message, msg_len);

  AES_LoadCounter(counter);

  enc_start = DWT->CYCCNT;

  if (HAL_CRYP_Encrypt(&hcryp,
                       (uint32_t *)plain,
                       enc_len / 4U,
                       (uint32_t *)cipher,
                       HAL_MAX_DELAY) != HAL_OK)
  {
    APP_LOG(TS_ON, VLEVEL_M, "AES encrypt error\r\n");
    return 0;
  }

  enc_end = DWT->CYCCNT;
  enc_cycles = enc_end - enc_start;

  BufferTx[0] = PROTO_MAGIC;
  BufferTx[1] = PROTO_TYPE_DATA;
  BufferTx[2] = id_node;
  BufferTx[3] = (uint8_t)(pendingRequestSeq & 0xFFU);
  BufferTx[4] = (uint8_t)((pendingRequestSeq >> 8) & 0xFFU);
  memcpy(BufferTx + 5, &counter, TX_COUNTER_SIZE);
  memcpy(BufferTx + PROTO_DATA_HEADER_SIZE, cipher, enc_len);

  prepare_end = DWT->CYCCNT;

  prepare_cycles = prepare_end - prepare_start;

  APP_LOG(TS_ON, VLEVEL_M, "enc_cycles: %u\r\n", (unsigned int)enc_cycles);
  APP_LOG(TS_ON, VLEVEL_M, "prepare_cycles: %u\r\n", (unsigned int)prepare_cycles);

  uint64_t enc_01us = ((uint64_t)enc_cycles * 100000000ULL) / SystemCoreClock;
  uint64_t prepare_01us = ((uint64_t)prepare_cycles * 100000000ULL) / SystemCoreClock;

  APP_LOG(TS_ON, VLEVEL_M, "enc_time_us: %u.%02u\r\n",
          (unsigned int)(enc_01us / 100ULL),
          (unsigned int)(enc_01us % 100ULL));

  APP_LOG(TS_ON, VLEVEL_M, "prepare_time_us: %u.%02u\r\n",
          (unsigned int)(prepare_01us / 100ULL),
          (unsigned int)(prepare_01us % 100ULL));

  APP_LOG(TS_ON, VLEVEL_M, "encrypted message: ");
  for (uint8_t i = 0; i < enc_len; i++)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "%02X", BufferTx[PROTO_DATA_HEADER_SIZE + i]);
  }
  APP_LOG(TS_OFF, VLEVEL_M, "\r\n");

  return (uint8_t)(PROTO_DATA_HEADER_SIZE + enc_len);
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
#else
#error "Pnly LoRa mode handled in this simplified version"
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
  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
#else
#error "Only LoRa mode handled in this simplified version"
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

  Radio.Rx(RX_TIMEOUT_VALUE);
}

static void SendNow(void)
{
  pendingReplySize = PrepareTxPayload();

  if (pendingReplySize > 0U)
  {
    RadioSend(pendingReplySize);
  }
  else
  {
    AppState = APP_LISTEN;
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  }
}

static void AppProcess(void)
{
  switch (AppState)
  {
    case APP_LISTEN:
      APP_LOG(TS_ON, VLEVEL_M, "Listening for requests\r\n");
      RadioRx();
      break;

    case APP_SEND_REPLY:
      SendNow();
      break;

    case APP_REPLY_RETRY:
      HAL_Delay(REPLY_RETRY_DELAY_MS);
      if (pendingReplySize > 0U)
      {
        RadioSend(pendingReplySize);
      }
      else
      {
        AppState = APP_LISTEN;
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
      }
      break;
  }
}

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* USER CODE END PrFD */
