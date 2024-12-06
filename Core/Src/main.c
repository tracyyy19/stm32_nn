/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fatfs.h"
#include "libjpeg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

// for BSP
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"

// for AI
#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "network.h"	// AI model header
#include "network_data.h"	// AI model data header

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
FATFS fs;             // File system object
FIL file;             // File object
uint8_t workBuffer[512]; // Work buffer for FATFS
extern SDRAM_HandleTypeDef sdramHandle;     // SDRAM handle
extern LTDC_HandleTypeDef hLtdcHandler; // LTDC handle

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CRC_HandleTypeDef hcrc;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void LCD_Init(void);
void DisplayJPG(const char *filename);
int read_binary_file(const char *file_name, uint8_t *buffer, uint32_t buffer_size);
int has_bin_extension(const char *filename);
int has_corresponding_jpg(const char *bin_filename, char *jpg_filename);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_LIBJPEG_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  // Debug SD initialization
  printf("Initializing SD...\n");
  if (BSP_SD_Init() != MSD_OK) {
  		printf("SD card initialization failed\n");
  		while (1);  // Stop execution if SD card initialization fails
  	}
  printf("SD initialized successfully\n");

  // Debug SDRAM Initialization
  printf("Initializing SDRAM...\n");
  if (BSP_SDRAM_Init() != SDRAM_OK) {
      printf("SDRAM initialization failed\n");
      Error_Handler();
  }
	printf("SDRAM initialized successfully\n");

	// Debug LCD Initialization
	printf("Initializing LCD...\n");
	LCD_Init();
	printf("LCD initialized successfully\n");

	if (f_mount(&fs, (TCHAR const*) SDPath, 0) == FR_OK) {
		printf("SD card mounted successfully\n");
	} else {
		printf("Failed to mount SD card\n");
		while (1);  // Stop execution if mounting fails
	}
	// Test directory access
	DIR dir;
	FRESULT res = f_opendir(&dir, SDPath);  // Test root directory access
	if (res == FR_OK) {
		printf("Root directory accessed successfully.\n");
		f_closedir(&dir);  // Close the directory
	} else {
		printf("Failed to access SD card root directory. Error code: %d\n",res);
		if (res == FR_INVALID_NAME) {
			printf("Hint: Invalid path or SD card not formatted properly.\n");
		}
		while (1);  // Stop execution if directory access fails
	}

	/* AI initialization */
	ai_error ai_err;
	ai_i8 nbatch;

	// Chunk of memory used to hold intermediate values for neural network
	AI_ALIGNED(4) ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

	// Buffers used to store input and output tensor
	AI_ALIGNED(4) ai_u8 in_data[AI_NETWORK_IN_1_SIZE_BYTES];
	AI_ALIGNED(4) ai_float out_data[AI_NETWORK_OUT_1_SIZE_BYTES];

	// Pointer to our model
	ai_handle robotDetection = AI_HANDLE_NULL;

	// Initialize wrapper structures that hold pointers to data and info
	// about data (tensor height, width, channels)
	ai_buffer ai_input_network[AI_NETWORK_IN_NUM] = AI_NETWORK_IN;
	ai_buffer ai_output_network[AI_NETWORK_OUT_NUM] = AI_NETWORK_OUT;

	// Set working memory and get weights/biases from model
	ai_network_params ai_params = AI_NETWORK_PARAMS_INIT(AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()), AI_NETWORK_DATA_ACTIVATIONS(activations));

	// Set pointers wrapper structures to our data buffers
	ai_input_network[0].n_batches = 1;
	ai_input_network[0].data = AI_HANDLE_PTR(in_data);
	ai_output_network[0].n_batches = 1;
	ai_output_network[0].data = AI_HANDLE_PTR(out_data);

	printf("Initializing neural network...\n");
	ai_err = ai_network_create(&robotDetection, AI_NETWORK_DATA_CONFIG);
	if (ai_err.type != AI_ERROR_NONE) {
		printf("Error Creating NN");
		while (1);
	}
	if (!ai_network_init(robotDetection, &ai_params)) {
		printf("Initializing NN Error");
		while (1);
	}

	printf("Neural network initialized\n");

	uint32_t counter = 1; // Track file count
	char buffer[20];
//	HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	DIR dir;
	FILINFO fno;
	FRESULT res;

	char test_path[100] = "test2"; // Path to the test folder

	// Open the test directory
	res = f_opendir(&dir, test_path);
	if (res != FR_OK) {
		printf("Failed to open test directory. Error code: %d\n", res);
		while (1);  // Stop execution if directory can't be opened
	}

	// Read files from the directory
	while (1) {
		res = f_readdir(&dir, &fno); // Read a directory item
		if (res != FR_OK || fno.fname[0] == 0)
			break; // Break on error or end of directory

		if (fno.fattrib & AM_DIR) {
			// It's a directory; skip it
			continue;
		} else {
			// Check if the file has a .bin extension
			if (has_bin_extension(fno.fname)) {
				// Display the filename being processed on the LCD
				BSP_LCD_DisplayStringAt(0, 35,(uint8_t*) "Processing File:", CENTER_MODE);
				BSP_LCD_DisplayStringAt(0, 55, (uint8_t*) fno.fname,CENTER_MODE);
				printf("\nProcessing %s\n", fno.fname);

				// Check for a corresponding .jpg file
				char jpg_filename[100];
                if (has_corresponding_jpg(fno.fname, jpg_filename)) {
                    // Display the .jpg file
                    printf("Displaying image: %s\n", jpg_filename);
                    DisplayJPG(jpg_filename); // Function to display the .jpg file on the LCD
                    HAL_Delay(3000); // Add delay for user to see the image
                } else {
                    printf("No corresponding image found for %s\n", fno.fname);
                }

                // Read the binary file into in_data
				if (read_binary_file(fno.fname, in_data, sizeof(in_data))!= 0) {
					printf("read error\n");
					HAL_Delay(2000); // Wait before continuing
					continue; // Skip to the next file
				}

				// Display "Performing Inference" on the LCD
				BSP_LCD_DisplayStringAt(0, 110,(uint8_t*) "Performing Inference", CENTER_MODE);

				// Perform inference
				uint32_t start_time = HAL_GetTick();
				nbatch = ai_network_run(robotDetection,&ai_input_network[0], &ai_output_network[0]);
				uint32_t end_time = HAL_GetTick();
				uint32_t inference_time = end_time - start_time;

				if (nbatch <= 0) {
					printf("Inference failed for %s\n", fno.fname);
					HAL_Delay(2000); // Wait before continuing
					continue; // Skip to the next file
				}

				// Extract confidence scores
				float confidence_robot = ((float*) out_data)[0];
				float confidence_non_robot = 1.0f - confidence_robot;

				char result_buffer[50];
				char time_buffer[30];

				// Determine the output based on confidence
				if (confidence_robot >= 0.5) {
					snprintf(result_buffer, sizeof(result_buffer),"Robot: %.2f%%", confidence_robot * 100.0);
					BSP_LCD_SetBackColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(0, 130,(uint8_t*) "Robot Detected", CENTER_MODE);
					printf("Performing inference %s\n", fno.fname);
					printf("Robot confidence: %.2f%%\n", confidence_robot * 100.0);
					printf("Output: Robot detected\n");
				} else {
					snprintf(result_buffer, sizeof(result_buffer),"Non-Robot: %.2f%%", confidence_non_robot * 100.0);
					BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
					BSP_LCD_DisplayStringAt(0, 130, (uint8_t*) "Non-Robot Detected", CENTER_MODE);
					printf("Performing inference %s\n", fno.fname);
					printf("Non-robot confidence: %.2f%%\n", confidence_non_robot * 100.0);
					printf("Output: Non-robot detected\n");
				}

				snprintf(time_buffer, sizeof(time_buffer), "Time: %lu ms", inference_time);

				// Display results on the LCD
				BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_DisplayStringAt(0, 170,(uint8_t*) result_buffer, CENTER_MODE);
				BSP_LCD_DisplayStringAt(0, 200, (uint8_t*) time_buffer,CENTER_MODE);

				snprintf(buffer, sizeof(buffer), "File Count: %lu", counter++);
				BSP_LCD_DisplayStringAt(0, 220, (uint8_t *)buffer, CENTER_MODE);

				// Delay
				HAL_Delay(1000);
			}
		}
		BSP_LCD_Clear(LCD_COLOR_WHITE);
	}

	f_closedir(&dir);
	counter = 1;
	BSP_LCD_Clear(LCD_COLOR_WHITE);

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
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int read_binary_file(const char *file_name, uint8_t *buffer,
		uint32_t buffer_size) {
	DIR dir;
	printf("SDPath: %s\n", SDPath);  // Debug SDPath
	FRESULT res = f_opendir(&dir, SDPath);
	if (res != FR_OK) {
		printf("Failed to access SD card root directory. Error code: %d\n",res);
		return -1;
	}
	printf("Root directory opened successfully.\n");

	printf("Attempting to open file: %s\n", file_name);

	// Check file existence
	FILINFO file_info;
	res = f_stat(file_name, &file_info);
	if (res != FR_OK) {
		printf("Error: File not found - %s. FatFS Error: %d\n", file_name, res);
		return -1;
	}

	// Check file size
	if (file_info.fsize != buffer_size) {
		printf("File size mismatch. Expected: %lu, Found: %lu\n", buffer_size,file_info.fsize);
		return -1;
	}

	// Open file
	res = f_open(&file, file_name, FA_READ);
	if (res != FR_OK) {
		printf("Error opening file: %s. FatFS Error: %d\n", file_name, res);
		return -1;
	}

	// Read file
	UINT bytes_read;
	res = f_read(&file, buffer, buffer_size, &bytes_read);
	if (res != FR_OK || bytes_read != buffer_size) {
		printf("Error reading file: %s. FatFS Error: %d, Bytes read: %u\n",file_name, res, bytes_read);
		f_close(&file);
		return -1;
	}

	// Close file
	f_close(&file);
	printf("File read successfully: %s\n", file_name);
	return 0;
}

void LCD_Init(void) {

	BSP_SDRAM_Init();
	BSP_SD_Init();
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);

    // Set pixel format explicitly to RGB565
    LTDC_LayerCfgTypeDef LayerConfig = {0};
    LayerConfig.WindowX0 = 0;
    LayerConfig.WindowX1 = BSP_LCD_GetXSize();
    LayerConfig.WindowY0 = 0;
    LayerConfig.WindowY1 = BSP_LCD_GetYSize();
    LayerConfig.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    LayerConfig.FBStartAdress = LCD_FB_START_ADDRESS;
    LayerConfig.Alpha = 255;
    LayerConfig.Alpha0 = 0;
    LayerConfig.Backcolor.Blue = 0;
    LayerConfig.Backcolor.Green = 0;
    LayerConfig.Backcolor.Red = 0;
    LayerConfig.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    LayerConfig.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    LayerConfig.ImageWidth = BSP_LCD_GetXSize();
    LayerConfig.ImageHeight = BSP_LCD_GetYSize();

    if (HAL_LTDC_ConfigLayer(&hLtdcHandler, &LayerConfig, 0) != HAL_OK) {
        printf("LTDC layer configuration failed\n");
        Error_Handler();
    }

    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_DisplayOn();
    BSP_LCD_SetFont(&Font20);

    printf("LCD configured\n");
}

void DisplayJPG(const char *filename) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    uint8_t *jpeg_output_buffer;

    FIL file;

    if (f_open(&file, filename, FA_READ) != FR_OK) {
        printf("Failed to open file: %s\n", filename);
        BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)"File Open Error", CENTER_MODE);
        return;
    }
    printf("File %s opened successfully\n", filename);

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, &file);
    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);

    printf("Image dimensions: %dx%d\n", cinfo.output_width, cinfo.output_height);

    uint32_t buffer_size = cinfo.output_width * cinfo.output_height * cinfo.output_components;
    jpeg_output_buffer = (uint8_t *)aligned_alloc(4, buffer_size);
    if (!jpeg_output_buffer) {
        printf("Failed to allocate memory for JPEG decoding\n");
        BSP_LCD_DisplayStringAt(0, 0, (uint8_t *)"Memory Error", CENTER_MODE);
        jpeg_destroy_decompress(&cinfo);
        f_close(&file);
        return;
    }
    printf("Memory allocated for JPEG decoding\n");

    while (cinfo.output_scanline < cinfo.output_height) {
        uint8_t *row_pointer[1] = {
            jpeg_output_buffer + cinfo.output_scanline * cinfo.output_width * cinfo.output_components
        };
        if (jpeg_read_scanlines(&cinfo, row_pointer, 1) != 1) {
            printf("Error reading scanline %d\n", cinfo.output_scanline);
            free(jpeg_output_buffer);
            jpeg_destroy_decompress(&cinfo);
            f_close(&file);
            return;
        }
        printf("Line %d decoded\n", cinfo.output_scanline);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    f_close(&file);

    printf("JPEG decompression complete. Rendering image...\n");

    // Display the decompressed image on the LCD
    uint16_t *framebuffer = (uint16_t *)LCD_FB_START_ADDRESS;
    printf("Starting image rendering...\n");

    // Calculate offsets for center-left positioning
    int x_offset = 15; // Align to left
    int y_offset = (BSP_LCD_GetYSize() - cinfo.output_height) / 2;

    for (int y = 0; y < cinfo.output_height; y++) {
        for (int x = 0; x < cinfo.output_width; x++) {
            uint8_t *pixel = &jpeg_output_buffer[(y * cinfo.output_width + x) * 3]; // RGB888
            uint16_t color = ((pixel[0] >> 3) << 11) |  // Red
                             ((pixel[1] >> 2) << 5)  |  // Green
                             (pixel[2] >> 3);           // Blue (RGB565)

            framebuffer[(y + y_offset) * BSP_LCD_GetXSize() + (x + x_offset)] = color;
        }
    }
    printf("Image rendering completed\n");

    // Free the decompressed image buffer
    free(jpeg_output_buffer);
    printf("JPEG image displayed successfully\n");

    // Display text beside the image
//    int text_x = cinfo.output_width + 20; // Offset text slightly to the right of the image
//    int text_y = y_offset;

    BSP_LCD_SetFont(&Font20);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK); // Set text color to black
//    BSP_LCD_DisplayStringAt(text_x, text_y +10, (uint8_t *)"IMG RENDERING", LEFT_MODE);
    printf("image rendering\n");

//    char buffer[50]; // Buffer to hold the generated strings
//
//    // Display the filename dynamically
//    sprintf(buffer, "%s", filename);
    BSP_LCD_DisplayStringAt(0, 75, (uint8_t *)filename, CENTER_MODE);

//    // Display a dynamic message for the image rendering
//    sprintf(buffer, "Displaying %s", filename);
//    BSP_LCD_DisplayStringAt(text_x, text_y + 60, (uint8_t *)buffer, LEFT_MODE);
//
//    BSP_LCD_DisplayStringAt(text_x, text_y + 90, (uint8_t *)"Successful Display", LEFT_MODE);
}

int has_bin_extension(const char *filename) {
    const char *ext = strrchr(filename, '.');
    if (ext != NULL) {
        return (strcmp(ext, ".bin") == 0 || strcmp(ext, ".BIN") == 0);
    }
    return 0;
}

int has_corresponding_jpg(const char *bin_filename, char *jpg_filename) {
    // Change the .bin extension to .jpg
    strcpy(jpg_filename, bin_filename);
    char *ext = strrchr(jpg_filename, '.');
    if (ext) {
        strcpy(ext, ".jpg");
    } else {
        return 0; // No extension found in the filename
    }

    // Check if the file exists on the SD card
    FIL file;
    if (f_open(&file, jpg_filename, FA_READ) == FR_OK) {
        f_close(&file);
        return 1;
    }
    return 0;
}


/* USER CODE END 4 */

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
