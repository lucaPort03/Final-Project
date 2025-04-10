/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/udp.h"
#include <string.h>
#include <tcp.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
struct client_struct *send_context = 0;
struct tcp_pcb *send_pcb = 0;
int counter = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint32_t adc_val;
uint32_t dac_val = 0;
volatile uint8_t tcp_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
const int udp = 0; //Decide whether UDP or TCP transmission
void tcp_client_init(void);
static err_t connected_callback(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *packet, err_t err);
static err_t client_poll(void *arg, struct tcp_pcb *pcb);
static err_t client_send_callback(void *arg, struct tcp_pcb *pcb, u16_t len); //Acknowledges data sent by client
static void client_send(struct tcp_pcb *pcb, struct client_struct *client_context); //Send data to server
static void client_close(struct tcp_pcb *pcb, struct client_struct *client_context);
static void client_handle(struct tcp_pcb *pcb, struct client_struct *client_context);

static err_t client_processing(struct client_struct *client_context, client_event_t event, struct tcp_pcb *pcb, struct pbuf *p, err_t err);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Callback functions */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  //Check SysTick configuration
  if (SysTick_Config(SystemCoreClock / 1000) != 0)
  {
      // SysTick configuration failed
      Error_Handler();
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  /* USER CODE BEGIN 2 */
  MX_LWIP_Init();


  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val,1);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void tcp_client_init(void) {
	struct tcp_pcb *tcpcb = tcp_new(); //Create a new TCP Process Control Block (PCB)
	ip_addr_t PC_IPADDR;
	IP4_ADDR(&PC_IPADDR, 192, 168, 0, 1);
	tcp_connect (tcpcb, &PC_IPADDR, 24, connected_callback); //Connect to IP address 192.168.0.1 and port 31
}

static err_t connected_callback(void *arg, struct tcp_pcb *pcb, err_t err) {

    LWIP_UNUSED_ARG(arg); // Avoid warnings if arg is not used
    LWIP_UNUSED_ARG(err);

    err_t err_callback;
    struct client_struct *client_context = (struct client_struct *)mem_malloc(sizeof(struct client_struct));

    if(client_context != NULL){

        client_context->state = CLIENT_STATE_CONNECT;
        client_context->pcb = pcb;
        client_context->retries = 0;
        client_context->p = NULL;

        // Initialize callbacks
        tcp_arg(pcb, client_context);
        tcp_recv(pcb, client_recv);
        tcp_poll(pcb, client_poll, 0);
        tcp_sent(pcb, client_send_callback);

        client_handle(pcb, client_context); // Ensure processing starts

        err_callback = ERR_OK;

        const char *message = "Hello, Server! Packet 1\n";
        err_t write_err = tcp_write(pcb, message, strlen(message), TCP_WRITE_FLAG_COPY);
    } else {
        client_close(pcb, client_context); // Ensure proper cleanup
        err_callback = ERR_MEM;
    }

    return err_callback;
}

static err_t client_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *packet, err_t err) {

	struct client_struct *client_context;
	err_t err_callback;

	client_context = (struct client_struct *)arg;

	if (packet == NULL ) { //Check if empty TCP Frame

		client_context->state = CLIENT_STATE_CLOSE;
		if(client_context->p == NULL) {
			client_close(pcb, client_context);
		}
		err_callback = ERR_OK;
	}
	else if (err != ERR_OK) {
		printf("TCP Error: %d \n", err);
		return client_processing(client_context, EVENT_STATE_ERR, pcb, packet, err);
	}
	else {
		tcp_recved(pcb, packet->len);
		return client_processing(client_context, EVENT_STATE_RECV, pcb, packet, err);
	}
	return err_callback;
}

static err_t client_processing(struct client_struct *client_context, client_event_t event, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
	switch(client_context->state) {
	case CLIENT_STATE_CONNECT:
		switch(event) {
			case EVENT_STATE_RECV:
				tcp_recved(pcb, p->tot_len);
				client_handle(pcb, client_context);
				pbuf_free(p);
				client_context->p=NULL;
				return ERR_OK;

			case EVENT_STATE_CLOSE:
				client_context->state = CLIENT_STATE_CLOSE;
				if(client_context->p == NULL) {
					client_close(pcb, client_context);
				}
				return ERR_OK;

			case EVENT_STATE_ERR:
				if (p != NULL) {
					pbuf_free(p);
					client_context->p = NULL;
				}
				return err;

			default:
				return ERR_VAL;
		} //EndCase
	break;

	case CLIENT_STATE_CLOSE:
	{
		switch (event) {
			case EVENT_STATE_RECV:
				tcp_recved(pcb, p->tot_len);
				pbuf_free(p);
				client_context->p = NULL;
				return ERR_OK;

			case EVENT_STATE_CLOSE:
				client_close(pcb, client_context);
				client_context->state = CLIENT_STATE_CLOSE;
				return ERR_OK;

			case EVENT_STATE_ERR:
				if(p != NULL) {
					pbuf_free(p);
					client_context->p = NULL;
				}
				return err;

			default:
				return ERR_VAL;
		}
		break;
	}
	default:
		if (p != NULL) {
			tcp_recved(pcb, p->tot_len);
			pbuf_free(p);
		}
		return ERR_OK;
	}
}

static err_t client_poll(void *arg, struct tcp_pcb *pcb) {

	err_t err_callback;
	struct client_struct *client_context;
	client_context = (struct client_struct *)arg;
	if(client_context != NULL) {
		if(client_context->p != NULL) {
			//Add code here for pending data
		}

		else {
			if(client_context->state == CLIENT_STATE_CLOSE) {
				client_close(pcb, client_context);
			} //end if
		}//end else
		err_callback = ERR_OK;
	} //end if

	else {
		tcp_abort(pcb);
		err_callback = ERR_ABRT;
	}

	return err_callback;
}

static err_t client_send_callback(void *arg, struct tcp_pcb *pcb, u16_t len) {
	struct client_struct *client_context;
	client_context = (struct client_struct *)arg;
	client_context->retries = 0;

	if(client_context->p != NULL) {
		return ERR_OK;
	}
	else {
		if(client_context->state == CLIENT_STATE_CLOSE) {
			client_close(pcb, client_context);
		}
	}
	return ERR_OK;
}

static void client_send(struct tcp_pcb *pcb, struct client_struct *client_context){

	struct pbuf *send_buffer;
	err_t send_err = ERR_OK;

	while((send_err == ERR_OK ) && (client_context->p != NULL) && (client_context->p->len <= tcp_sndbuf(pcb)) ) {
		send_buffer = client_context->p;
		send_err = tcp_write(pcb, send_buffer->payload, send_buffer->len, 1);

		if(send_err == ERR_OK){

			u16_t buff_len;
			u8_t freeState;

			buff_len = send_buffer->len;

			if(client_context->p != NULL){
				pbuf_ref(client_context->p);
			}
			do{
				freeState = pbuf_free(send_buffer);
			} while(freeState == 0);
		}
		else if(send_err == ERR_MEM) {
			client_context->p = send_buffer;
		}
	}
}

static void client_close(struct tcp_pcb *pcb, struct client_struct *client_context) {

    tcp_arg(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 0);

    if (client_context != NULL) {
        mem_free(client_context);
    }

    tcp_close(pcb);
}
static void client_handle(struct tcp_pcb *pcb, struct client_struct *client_context) {
	ip4_addr_t IP = pcb->remote_ip;
	uint16_t port = pcb->remote_port;
	char *remoteIP = ipaddr_ntoa(&IP); //Convert numeric IP to ASCII

	send_context = client_context;

	send_pcb = pcb;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
    tcp_client_init();
    const char* message = "Hello UDP message!\n\r";

    osDelay(1);

    ip_addr_t PC_IPADDR;
    IP4_ADDR(&PC_IPADDR, 192, 168, 0, 1);


    if(udp == 1){
		struct udp_pcb* my_udp = udp_new();
		if (!my_udp) {
			return; // Error handling if needed
		}

		udp_bind(my_udp, IP_ADDR_ANY, 0); // Bind to any available port
		udp_connect(my_udp, &PC_IPADDR, 55151);

		struct pbuf* udp_buffer = NULL;
		struct pbuf *p = pbuf_alloc(PBUF_RAW, 1472, PBUF_REF);
		static uint8_t tx_buff[1500] = "hello UDP!\n\r";

		p->payload = (void*)tx_buff;

		/* Infinite loop */
		for (;;)
		{
			osDelay(1);

			 //Allocate memory for UDP message (including null terminator)

			if (udp_buffer != NULL) {
				udp_buffer = pbuf_alloc(PBUF_TRANSPORT, 1472, PBUF_RAM); //if just message size then: strlen(message) + 1

				pbuf_take(udp_buffer, message, strlen(message));
				memcpy(udp_buffer->payload, message, strlen(message) + 1);
				udp_sendto(my_udp, udp_buffer, &PC_IPADDR, 55151);
				pbuf_free(udp_buffer);
			}

			memcpy(p->payload, message, 1400);
			for(int i = 0; i< 120; i++) { //Set to 120 for best performance
			udp_sendto(my_udp, p, &PC_IPADDR, 55151);
			}
		}
		// Free the UDP PCB when done
				udp_remove(my_udp);
		}
    else if (udp == 0) {
        char buf[100];

        for (;;) {
            sys_check_timeouts(); // LwIP timeouts
            if (tcp_ready && send_pcb != NULL && send_pcb->state == ESTABLISHED) {
                char buf[100];
                int len = sprintf(buf, "ID: %d\n", counter++);
                if (tcp_sndbuf(send_pcb) >= len) {
                    tcp_write(send_pcb, buf, len, TCP_WRITE_FLAG_COPY);
                    tcp_output(send_pcb);
                }
                osDelay(500); // Adjustable sending rate
            } else {
                tcp_client_init(); // Retry if not connected
                osDelay(1000);
            }
        }
    }
    /* USER CODE END 5 */
}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
