#include "main.h"

SPI_HandleTypeDef hspi1;
uint8_t rx_buffer[4] = {0};
uint8_t tx_buffer[4] = {0};
volatile uint8_t spi_busy = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void update_pid(float left_vel, float right_vel); // Placeholder
int16_t get_encoder_left(void); // Placeholder
int16_t get_encoder_right(void); // Placeholder

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();

  // Start SPI in interrupt mode
  if (HAL_SPI_Receive_IT(&hspi1, rx_buffer, 4) != HAL_OK) {
    Error_Handler(); // Defined in STM32CubeMX
  }

  while (1) {
    // Main loop can handle other tasks (e.g., IMU updates)
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1 && !spi_busy) {
    spi_busy = 1;

    // Decode velocity commands
    int16_t left_vel = (rx_buffer[0] << 8) | rx_buffer[1];
    int16_t right_vel = (rx_buffer[2] << 8) | rx_buffer[3];
    update_pid(left_vel / 100.0f, right_vel / 100.0f);

    // Prepare encoder feedback
    int16_t left_enc = get_encoder_left();
    int16_t right_enc = get_encoder_right();
    tx_buffer[0] = (left_enc >> 8) & 0xFF;
    tx_buffer[1] = left_enc & 0xFF;
    tx_buffer[2] = (right_enc >> 8) & 0xFF;
    tx_buffer[3] = right_enc & 0xFF;

    // Send feedback
    if (HAL_SPI_Transmit_IT(&hspi1, tx_buffer, 4) != HAL_OK) {
      Error_Handler();
    }
    spi_busy = 0;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1 && !spi_busy) {
    // Restart receive for next cycle
    if (HAL_SPI_Receive_IT(&hspi1, rx_buffer, 4) != HAL_OK) {
      Error_Handler();
    }
  }
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  // Configure GPIO pins for SPI (PA4, PA5, PA6, PA7) via STM32CubeMX
}

void update_pid(float left_vel, float right_vel) {
  // Implement PID control here
}

int16_t get_encoder_left(void) {
  return 0; // Replace with actual encoder reading
}

int16_t get_encoder_right(void) {
  return 0; // Replace with actual encoder reading
}

void Error_Handler(void) {
  while (1) {
    // Add error handling (e.g., LED blink)
  }
}
