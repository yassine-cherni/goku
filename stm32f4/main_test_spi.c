#include "main.h"

SPI_HandleTypeDef hspi1;
uint8_t rx_buffer[4];  // Receive velocity commands
uint8_t tx_buffer[4];  // Send encoder feedback

void SystemClock_Config(void);
static void MX_SPI1_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_SPI1_Init();

  while (1) {
    HAL_SPI_Receive_IT(&hspi1, rx_buffer, 4);
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  // Decode velocity commands
  int16_t left_vel = (rx_buffer[0] << 8) | rx_buffer[1];
  int16_t right_vel = (rx_buffer[2] << 8) | rx_buffer[3];
  update_pid(left_vel / 100.0, right_vel / 100.0); // Scale back

  // Send encoder feedback
  int16_t left_enc = get_encoder_left();
  int16_t right_enc = get_encoder_right();
  tx_buffer[0] = (left_enc >> 8) & 0xFF;
  tx_buffer[1] = left_enc & 0xFF;
  tx_buffer[2] = (right_enc >> 8) & 0xFF;
  tx_buffer[3] = right_enc & 0xFF;
  HAL_SPI_Transmit_IT(&hspi1, tx_buffer, 4);
}

static void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  HAL_SPI_Init(&hspi1);
}
