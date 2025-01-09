# sensor-drivers-2024-2025

## BNO055 EXAMPLE

Change **BNO055_I2C_ADDR** in header file to match your i2c address.

```c
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO055.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  BNO055_assignI2C(&hi2c4); //Change depending on your configs
  BNO055_setup();

  /* Uncomment and use if you would like to calibrate or re-calibrate BNO. Otherwise, don't need this.
   * If want to save in eeprom: USE_EEPROM
   * If want to save in sd card: USE_SD
   */
  //BNO055_calibrationRoutine(USE_EEPROM);

  /* Uncomment and use if you have ran BNO055_calibrationRoutine() once/before.
   * Keep using this once there are calibration offsets stored.
   * If want to load from eeprom: USE_EEPROM
   * If want to load from sd card: USE_SD
   */
  //BNO055_loadCalibrationData(USE_EEPROM);

  BNO055_vector_t euler, quaternion;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    BNO055_getVectorEuler(&euler);
	  printf("yaw = %f, roll = %f, pitch = %f\r\n", euler.x,euler.y,euler.z*(-1));
	  HAL_Delay(300);

    //for sensor fusion:
    BNO055_getVectorQuaternion(&quaternion);
    printf("w= %f, x= %f, y= %f, z= %f\r\n", quaternion.w,quaternion.x,euler.y,euler.z);
	  HAL_Delay(300);

    BNO055_getVectorGyroscope(&velo);
	  printf("veloN = %f, veloE = %f, veloD = %f\r\n", velo.x,velo.y,velo.z);
	  HAL_Delay(300);*/

	  BNO055_getVectorAccelerometer(&accel);
	  printf("accelX = %f, accelY = %f, accelZ = %f\r\n", accel.x,accel.y,accel.z);
	  HAL_Delay(300);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```