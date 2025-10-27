#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "csp/csp.h"
#include "csp/interfaces/csp_if_kiss.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"

static void system_clock_init(void);
static void csp_router(void * args);
static void csp_server(void * args);
static void csp_client(void * args);
static void uart_init(void);
static int uart_tx(void * driver_data, const uint8_t * data, size_t len);

static UART_HandleTypeDef uart;
static SemaphoreHandle_t uart_mutex;
static volatile uint8_t uart_rx_byte;
static csp_iface_t kiss_iface;
static csp_kiss_interface_data_t kiss_ifdata;

int main(void) {
    HAL_Init();
    system_clock_init();
    
    csp_init();

    kiss_ifdata.tx_func = uart_tx;
    kiss_iface.interface_data = &kiss_ifdata;
    kiss_iface.name = CSP_IF_KISS_DEFAULT_NAME;
    kiss_iface.driver_data = &uart;
    kiss_iface.addr = 10;

    if (CSP_ERR_NONE != csp_kiss_add_interface(&kiss_iface)) {
        configASSERT(false);
    }

    kiss_iface.is_default = true;

    uart_mutex = xSemaphoreCreateRecursiveMutex();
    configASSERT(uart_mutex);

    uart_init();

    extern void SVC_Setup(void);
    SVC_Setup();

    xTaskCreate(csp_router, "csp_router", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(csp_server, "csp_server", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(csp_client, "csp_client", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    vTaskStartScheduler();
    
    while (1);
}

static void csp_router(void * args) {
    while (1) {
        csp_route_work();
    }
}

static void csp_client(void * args) {
    uint8_t count = 0;
    
    while (1) {
        /* 1. Connect to host on 'server_address', port SERVER_PORT with regular UDP-like protocol and 1000 ms timeout */
		csp_conn_t * conn = csp_connect(CSP_PRIO_NORM, 2, 10, 1000, CSP_O_NONE);
		if (conn == NULL) {
			continue;
		}

		/* 2. Get packet buffer for message/data */
		csp_packet_t * packet = csp_buffer_get(0);
		if (packet == NULL) {
            csp_close(conn);
			continue;
		}

		/* 3. Copy data to packet */
		memcpy(packet->data, "Hello world ", 12);
		memcpy(packet->data + 12, &count, 1);
		memset(packet->data + 13, 0, 1);
		count++;

		/* 4. Set packet length */
		packet->length = (strlen((char *) packet->data) + 1); /* include the 0 termination */

		/* 5. Send packet */
		csp_send(conn, packet);

		/* 6. Close connection */
		csp_close(conn);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void csp_server(void * args) {
    /* Create socket with no specific socket options, e.g. accepts CRC32, HMAC, etc. if enabled during compilation */
    csp_socket_t sock = {0};

    /* Bind socket to all ports, e.g. all incoming connections will be handled here */
    csp_bind(&sock, CSP_ANY);

    /* Create a backlog */
    csp_listen(&sock, 10);

    /* Wait for connections and then process packets on the connection */
    while (1) {
        /* Wait for a new connection, 10000 mS timeout */
        csp_conn_t * conn;
        if ((conn = csp_accept(&sock, 10000)) == NULL) {
            /* timeout */
            continue;
        }

        /* Read packets on connection, timout is 100 mS */
        csp_packet_t * packet;
        while ((packet = csp_read(conn, 100)) != NULL) {
            switch (csp_conn_dport(conn)) {
                case 10:
                    /* Process packet here */
                    csp_buffer_free(packet);
                    break;

                default:
                    /* Call the default CSP service handler, handle pings, buffer use, etc. */
                    csp_service_handler(packet);
                    break;
            }
        }

        /* Close current connection */
        csp_close(conn);
    }
}

static void uart_init(void) {
    uart.Instance = USART3;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
    uart.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    uart.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
    if (HAL_UART_Init(&uart) != HAL_OK) {
        configASSERT(false);
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&uart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        configASSERT(false);
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&uart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        configASSERT(false);
    }
    if (HAL_UARTEx_DisableFifoMode(&uart) != HAL_OK) {
        configASSERT(false);
    }

    if (HAL_UART_Receive_IT(&uart, (uint8_t *)&uart_rx_byte, 1) != HAL_OK) {
        configASSERT(false);
    }
}

static int uart_tx(void * driver_data, const uint8_t * data, size_t len) {
    if ((NULL != driver_data) && (NULL != data) && (len > 0)) {
        UART_HandleTypeDef * huart = (UART_HandleTypeDef *)driver_data;
        (void)HAL_UART_Transmit(huart, data, len, 100);
        return CSP_ERR_NONE;
    }

    return CSP_ERR_INVAL;
}

/** @brief Required by CSP for thread safety. */
void csp_usart_lock(void * driver_data) {
    UART_HandleTypeDef * huart = (UART_HandleTypeDef *)driver_data;
    if ((NULL != huart) && (huart == &uart)) {
        if (xSemaphoreTakeRecursive(uart_mutex, portMAX_DELAY) != pdTRUE) {
            configASSERT(false);
        }
    }
}

/** @brief Required by CSP for thread safety. */
void csp_usart_unlock(void * driver_data) {
    UART_HandleTypeDef * huart = (UART_HandleTypeDef *)driver_data;
    if ((NULL != huart) && (huart == &uart)) {
        if (xSemaphoreGiveRecursive(uart_mutex) != pdTRUE) {
            configASSERT(false);
        }
    }
}

/** @brief This function handles USART3 RX complete interrupt. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
    if (huart == &uart) {
        BaseType_t task_woken = pdFALSE;
        csp_kiss_rx(&kiss_iface, (uint8_t *)&uart_rx_byte, 1, &task_woken);
        if (HAL_UART_Receive_IT(huart, (uint8_t *)&uart_rx_byte, 1) != HAL_OK) {
            configASSERT(false);
        }

        portYIELD_FROM_ISR(task_woken);
    }
}

/** @brief This function handles USART3 global interrupt. */
void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&uart);
}

/** @brief System Clock Configuration. */
static void system_clock_init(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    /**
     * Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure. 
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        configASSERT(false);
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        configASSERT(false);
    }
}
