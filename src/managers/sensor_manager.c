/**
 * @brief Write this
 * 
 * @todo Finish DMA functionality if needed for LORA
 * @note DMA currently commented out
 * 
 */


#include "stm32l432xx.h"
#include "sensor_manager.h"
#include "spi_driver.h"
#include "bme280_defs.h"
#include "bme280.h"
#include "rcc_driver.h"
#include "gpio_driver.h"
#include "dma_driver.h"

#define SPI_DMA_RX_EN  1
#define SPI_DMA_TX_EN  1

#define GPIOB_SPI_MOSI_PIN 5U 
#define GPIOB_SPI_MISO_PIN 4U 
#define GPIOB_SPI_SCK_PIN 3U 
#define GPIOB_SPI_NSS_PIN 0U 

#define GPIOA_SPI_MOSI_PIN 7U 
#define GPIOA_SPI_MISO_PIN 6U 
#define GPIOA_SPI_SCK_PIN 1U 
#define GPIOA_SPI_NSS_PIN 4U 

#define GPIO_SPI1_AF_CODE 5U   //AF5
#define GPIO_SPI3_AF_CODE 6U   //AF6


#define SPI_DATA_SIZE_CHAR 8U
#define SPI_PIN_AMOUNT 4U

#define SPI1_DMA_REQ_CODE 1U  // Code is the same for each SPI pin
 


static struct bme280_dev bme280_dev_ctx;    // Struct Containing the BME280 configuration data
static spi_config_t spi_cfg;            // Struct Containing the SPI peripheral configuration data
// static dma_channel_config_t dma_rx_cfg;         // Struct Containing the DMA configuration for the SPIx_RX pin
// static dma_channel_config_t dma_tx_cfg;         // Struct Containing the DMA configuration for the SPIx_TX pin



// static int8_t spi_transfer_dma(DMA_TypeDef* dma_line, SPI_TypeDef* spi_line, const uint8_t* tx_buffer, uint8_t* rx_buffer);
// static int8_t spi_disable_dma(SPI_TypeDef* spi_line, DMA_Channel_TypeDef* dma_channel);
// static int8_t manager_dma_config(DMA_TypeDef* dma_line, const uint8_t* tx_buffer, uint8_t* rx_buffer);
static int8_t manager_rcc_config();
static int8_t manager_spi_config(const SPI_TypeDef* spi_line);
static int8_t manager_bme280_config(void);
static int8_t manager_gpio_config(void);
static int8_t manager_gpio_pin_config(GPIO_TypeDef* gpio_port, uint8_t gpio_pin, gpio_moder_t mode, gpio_ospeedr_t output_speed,
                                      gpio_otyper_t output_type, gpio_pupdr_t pupdr, uint8_t af_code);

static int8_t user_spi_read_blocking(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
static int8_t user_spi_write_blocking(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);




/*========================= SENSOR CONFIG FUNCTIONS =============================*/

/**
 * @brief Initiates Sensor Manager 
 */
int8_t manager_init() {
    int8_t retval;

    retval = manager_rcc_config();
    // cppcheck-suppress knownConditionTrueFalse
    if(retval != SENSOR_OK) return retval;

    retval = manager_gpio_config();
    // cppcheck-suppress knownConditionTrueFalse
    if(retval != SENSOR_OK) return retval;

    retval = manager_spi_config(SPI1);
    // cppcheck-suppress knownConditionTrueFalse
    if(retval != SENSOR_OK) return retval;

    retval = manager_bme280_config();
    // cppcheck-suppress knownConditionTrueFalse
    if(retval != SENSOR_OK) return retval;

    return SENSOR_OK;
}

// /**
//  * @brief Configures the DMA peripheral for SPI transfer
//  */
// static int8_t manager_dma_config(DMA_TypeDef* dma_line, const uint8_t* tx_buffer, uint8_t* rx_buffer){
//   if(!tx_buffer || !rx_buffer) return SENSOR_ERR_INVALID_PARAM;

//   // Consider Changing Hardcoding
//   dma_rx_cfg.dma_channel = DMA1_Channel2;
//   dma_rx_cfg.dma_channel_number = 2U;
//   dma_rx_cfg.dma_channel_request = SPI1_DMA_REQ_CODE;
//   dma_rx_cfg.periph_address = &SPI1->DR;
//   dma_rx_cfg.memory_address = rx_buffer; //change
//   dma_rx_cfg.mem_increment_mode = DMA_MEMORY_INCREMENT_ENABLED;
//   dma_rx_cfg.periph_increment_mode = DMA_PERIPH_INCREMENT_DISABLED;
//   dma_rx_cfg.transfer_direction = DMA_READ_FROM_PERIPHERAL;
//   dma_rx_cfg.data_size = 8U;
//   dma_rx_cfg.channel_priority = DMA_PRIORITY_VERY_HIGH;
//   dma_rx_cfg.mem_size = DMA_MSIZE_8;
//   dma_rx_cfg.periph_size = DMA_PSIZE_8;
//   dma_rx_cfg.circular_mode = DMA_CIRCULAR_DISABLED;
//   dma_rx_cfg.interrupts = DMA_INT_TRANSFER_COMPLETE;


//   // Possibly Change Hardcoding
//   dma_tx_cfg.dma_channel = DMA1_Channel3;
//   dma_tx_cfg.dma_channel_number = 3U;
//   dma_tx_cfg.dma_channel_request = SPI1_DMA_REQ_CODE;
//   dma_tx_cfg.periph_address = &SPI1->DR;
//   dma_tx_cfg.memory_address = tx_buffer; //change
//   dma_tx_cfg.mem_increment_mode = DMA_MEMORY_INCREMENT_ENABLED;
//   dma_tx_cfg.periph_increment_mode = DMA_PERIPH_INCREMENT_DISABLED;
//   dma_tx_cfg.transfer_direction = DMA_READ_FROM_MEMORY;
//   dma_tx_cfg.data_size = 8U;
//   dma_tx_cfg.channel_priority = DMA_PRIORITY_HIGH;
//   dma_tx_cfg.mem_size = DMA_MSIZE_8;
//   dma_tx_cfg.periph_size = DMA_PSIZE_8;
//   dma_tx_cfg.circular_mode = DMA_CIRCULAR_DISABLED;
//   dma_tx_cfg.interrupts = DMA_INT_TRANSFER_COMPLETE;


//   // Configure the DMA line and channel for SPI_TX
//   dma_err_t dma_tx_error = DMA_OK;
//   uint8_t dma_tx_cfg_retval = dma_config_channel(dma_line, &dma_tx_cfg, &dma_tx_error);
//   if(dma_tx_cfg_retval != DMA_OK) return dma_tx_cfg_retval;

//   // Configure the DMA Line and channel for SPI_RX
//   dma_err_t dma_rx_error = DMA_OK;
//   uint8_t dma_rx_cfg_retval = dma_config_channel(dma_line, &dma_rx_cfg, &dma_rx_error);
//   if(dma_rx_cfg_retval != DMA_OK) return dma_rx_cfg_retval;

//   return SENSOR_OK;
// }

/**
 * @brief Configures the SPI peripheral for the sensor
 */
static int8_t manager_spi_config(const SPI_TypeDef* spi_line){
  if(!spi_line) return SENSOR_ERR_INVALID_PARAM;

  spi_cfg.baud_rate_factor = SPI_BAUD_DIV2;               // Maybe Put more thought into this decision
  spi_cfg.clock_phase = SPI_CLK_PHASE0;                   // First data capture edge is the first clock transition
  spi_cfg.clock_polarity = SPI_CLK_IDLE0;                 // Clock signal goes low when idle
  spi_cfg.data_size_in_bits = SPI_DATA_SIZE_CHAR;
  spi_cfg.enable_crc = SPI_CRC_DISABLED;                  // No Cyclic Redundancy Test
  spi_cfg.fifo_rx_threshold = SPI_FIFO_LVL_HALF;          // RXNE generated whenever RXFIFO is half full
  spi_cfg.lsb_first = SPI_MSB_FIRST;                
  spi_cfg.master_selction = SPI_MASTER_CONFIG;            // Configure SPI as Master
  spi_cfg.nss_pulse = SPI_NO_NSS_PULSE;                   // Don't generate NSS pulses
  spi_cfg.simplex_mode = SPI_FULL_DUPLEX;                 // Standard Transmission (not simplex)
  spi_cfg.slave_select_enabled = SPI_SS_OUTPUT_DISABLED;  // Might Change this actually since we need to choose whether to interface with Sensor or LoRa
  spi_cfg.ssm_enabled = SPI_SSM_DISABLED;

  return SENSOR_OK;
}

/**
 * @brief Enables clocks for relevant peripherals
 */
static int8_t manager_rcc_config(){
  // Consider configuring PLL for SPI communication

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Possibly change hardcoding
  //RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // Maybe GPIOA?

  return SENSOR_OK;
}

/**
 * @brief Configures all the GPIO pins required for interfacing with the sensor
 */
static int8_t manager_gpio_config(void){
  //cppcheck-suppress variableScope
  int8_t retval;

  // Configure GPIO for SPI_MOSI, SPI_MISO, SPI_SCK Alternate Functions

  // This Version emitted memcpy calls since it's stored in .rodata and essentially copied in manager_gpio_pin_config
  // const gpio_config_t spi_gpios[] = {
  //   {GPIOB, GPIOA_SPI_MOSI_PIN, (gpio_moder_t)GPIO_MODER_AF, (gpio_ospeedr_t)GPIO_OSPEEDR_VERY_HIGH, (gpio_otyper_t)GPIO_OTYPER_PUSH_PULL, (gpio_pupdr_t)GPIO_PUPDR_NONE, GPIO_SPI1_AF_CODE}, // SPI_MOSI pin configuration
  //   {GPIOB, GPIOA_SPI_MISO_PIN, (gpio_moder_t)GPIO_MODER_AF, (gpio_ospeedr_t)GPIO_OSPEEDR_VERY_HIGH, (gpio_otyper_t)GPIO_OTYPER_PUSH_PULL, (gpio_pupdr_t)GPIO_PUPDR_NONE, GPIO_SPI1_AF_CODE}, // SPI_MISO pin configuration
  //   {GPIOB, GPIOA_SPI_SCK_PIN, (gpio_moder_t)GPIO_MODER_AF, (gpio_ospeedr_t)GPIO_OSPEEDR_VERY_HIGH, (gpio_otyper_t)GPIO_OTYPER_PUSH_PULL, (gpio_pupdr_t)GPIO_PUPDR_NONE, GPIO_SPI1_AF_CODE},  // SPI_SCK pin configuration
  //   {GPIOB, GPIOA_SPI_NSS_PIN, (gpio_moder_t)GPIO_MODER_AF, (gpio_ospeedr_t)GPIO_OSPEEDR_VERY_HIGH, (gpio_otyper_t)GPIO_OTYPER_PUSH_PULL, (gpio_pupdr_t)GPIO_PUPDR_NONE, GPIO_SPI1_AF_CODE},  // SPI_NSS pin configuration
  // };

  //const uint8_t num_pins = sizeof(spi_gpios) / sizeof(spi_gpios[0]);

  // This project is completely baremetal so I want to avoid calling stdlib function
  // This verision avoids memcpy calls - it's a little messier so i might end up writing a utils file and implementing my own memcpy
  GPIO_TypeDef* ports[] = {GPIOB, GPIOB, GPIOB, GPIOB};
  const uint8_t pins[] = {GPIOA_SPI_MOSI_PIN, GPIOA_SPI_MISO_PIN, GPIOA_SPI_SCK_PIN, GPIOA_SPI_NSS_PIN};
  gpio_moder_t modes[] = {GPIO_MODER_AF, GPIO_MODER_AF, GPIO_MODER_AF, GPIO_MODER_AF};
  gpio_ospeedr_t speeds[] = {GPIO_OSPEEDR_VERY_HIGH, GPIO_OSPEEDR_VERY_HIGH, GPIO_OSPEEDR_VERY_HIGH, GPIO_OSPEEDR_VERY_HIGH};
  gpio_otyper_t types[] = {GPIO_OTYPER_PUSH_PULL, GPIO_OTYPER_PUSH_PULL, GPIO_OTYPER_PUSH_PULL, GPIO_OTYPER_PUSH_PULL};
  gpio_pupdr_t pupdrs[] = {GPIO_PUPDR_NONE, GPIO_PUPDR_NONE, GPIO_PUPDR_NONE, GPIO_PUPDR_NONE};
  const uint8_t af_codes[] = {GPIO_SPI1_AF_CODE, GPIO_SPI1_AF_CODE, GPIO_SPI1_AF_CODE, GPIO_SPI1_AF_CODE};



  const uint8_t num_pins = SPI_PIN_AMOUNT;
  for (uint8_t i = 0; i < num_pins; i++){
    retval = manager_gpio_pin_config(
      ports[i],
      pins[i],
      modes[i],
      speeds[i],
      types[i],
      pupdrs[i],
      af_codes[i]
    );
    if(retval != SENSOR_OK){
      // RESET gpio pin
      return retval;
    }
  }

  return SENSOR_OK;
}

/**
 * @brief configures a single gpio pin with the passed settings
 */
static int8_t manager_gpio_pin_config(GPIO_TypeDef* gpio_port, uint8_t gpio_pin, gpio_moder_t mode, gpio_ospeedr_t output_speed,
                                      gpio_otyper_t output_type, gpio_pupdr_t pupdr, uint8_t af_code)
{
  gpio_config_t gpio_pin_cfg;
  gpio_pin_cfg.gpio_peripheral = gpio_port;
  gpio_pin_cfg.gpio_pin = gpio_pin;
  gpio_pin_cfg.mode = mode;
  gpio_pin_cfg.output_speed = output_speed;
  gpio_pin_cfg.output_type = output_type;
  gpio_pin_cfg.pupdr_config = pupdr;
  gpio_pin_cfg.alternate_function_code = af_code;
  
  gpio_error_t gpio_error = GPIO_OK;
  gpio_config_pin(&gpio_pin_cfg, &gpio_error);

  if(gpio_error != GPIO_OK)
    return gpio_error;
  
  return SENSOR_OK;
}
                                    
/**
 * @brief Configures BME280 Registers
 * 
 * @note Not the final function
 * 
 * @retval   0 -> Success.
 * @retval > 0 -> Warning.
 * @retval < 0 -> Fail.
 * 
 * @todo pass a transmission method parameter (polling - dma - interrupts)
 */
static int8_t manager_bme280_config(void){
  // Initialise Intf return value
  int8_t retval;

  // Initialise BME280 dev struct
  
  bme280_dev_ctx.intf = (enum bme280_intf)BME280_SPI_INTF,
  bme280_dev_ctx.intf_ptr = SPI1; //or SPI3 
  bme280_dev_ctx.intf_rslt = (BME280_INTF_RET_TYPE)0U;
  bme280_dev_ctx.read = &user_spi_read_blocking;
  bme280_dev_ctx.write = &user_spi_write_blocking;
  //.delay_us =
  //.calib_data = 

  retval = bme280_init(&bme280_dev_ctx);
  if(retval != BME280_OK) return retval;

  // Initialise settings struct with default values
  const struct bme280_settings settings = {
    .osr_t = BME280_NO_OVERSAMPLING,
    .osr_h = BME280_NO_OVERSAMPLING,
    .osr_p = BME280_NO_OVERSAMPLING,
    .filter = BME280_FILTER_COEFF_OFF,
    .standby_time = BME280_STANDBY_TIME_0_5_MS
  }; 

  uint8_t desired_settings = 
    BME280_SEL_OSR_HUM |
    BME280_SEL_OSR_PRESS |
    BME280_SEL_OSR_TEMP |
    BME280_SEL_FILTER |
    BME280_SEL_STANDBY;

  // Set Sensor Settings
  retval = bme280_set_sensor_settings(desired_settings, &settings, &bme280_dev_ctx);
  if(retval != BME280_OK) return retval;

  // Set Sensor to Forced Mode
  retval = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &bme280_dev_ctx);
  if(retval != BME280_OK) return retval;

  return SENSOR_OK;
}

/*======================= SPI-BME280 INTERFACING HELPERS =======================*/

// /**
//  * @brief An spi-read wrapper function to expose the SPI transfer function to the BME280 API.
//  * It appends the register address to the transmitted data as per the BME280 datasheet &
//  * receives register data in the given reg_data buffer.
//  * 
//  * @param[in] reg_addr This is the address of the bme280 register to read from
//  * @param[out] reg_data This is the data from the bme280 register
//  * @param[in] len The length of the data to receive
//  * @param[in] intf_ptr Points to the interfacing peripheral (e.g. SPI1, SPI3, I2C1)
//  * 
//  *   
//  * @note DMA & SPI must be configured before calling
//  * @note automatically clears the 7th bit of the register address to indicate a read operation to the bme280 sensor
//  */
// int8_t manager_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr){
//   if(!reg_data || !intf_ptr) return SENSOR_ERR_INVALID_PARAM;
//   SPI_TypeDef* spi_line = (SPI_TypeDef*)intf_ptr;

//   reg_addr |= 1U << 7; // Set the 7th bit of the register address - Indicates Read (I don't know what i was thinking when i wrote this)
  
//   // Fill Tx Buffer with dummy bytes to clock out the transfer
//   uint8_t bme280_tx_buffer[len+1];
//   // Append the Register Address to the start of the transfer
//   bme280_tx_buffer[0] = reg_addr; 

//   spi_transfer_dma(DMA1, spi_line, bme280_tx_buffer, reg_data); 

//   return SENSOR_OK;
// }

// /**
//  * @brief An spi-read wrapper function to expose the SPI transfer function to the BME280 API.
//  * It appends the register address to the transmitted data as per the BME280 datasheet &
//  * writes the given reg_data buffer to the given address.
//  * 
//  * @param[in] reg_addr The register address to write to
//  * @param[in] reg_data Pointer to the data to write to the register
//  * @param[in] len The length of the data to write
//  * @param[in] intf_ptr Pointer to the Interfacing Peripheral used to communicate e.g. SPI1, SPI3, I2C1
//  * 
//  * @retval 0 (SENSOR_OK) - Peripherals were succesful starting the transfer of data
//  * @retval <0 (SENSOR_ERR_X) - An error code
//  */
// int8_t manager_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
//   if(!reg_data || !intf_ptr) return SENSOR_ERR_INVALID_PARAM;

//   // Cast intf_ptr to an SPI peripheral pointer
//   SPI_TypeDef* spi_line = (SPI_TypeDef*)intf_ptr;

//   reg_addr &= ~(1U << 7); // Clear the 7th bit of the register address

//   // Initialise Transfer Buffer
//   uint8_t bme280_tx_buffer[len+1];
//   // Append the Register Address to the start
//   bme280_tx_buffer[0] = reg_addr;
//   // Fill the buffer with the rest of the data
//   for(uint32_t i = 0; i < len; i++){
//     bme280_tx_buffer[i+1] = reg_data[i];
//   }
  
//   uint8_t dummy_rx_buffer[1];

//   // Transfer with DMA
//   spi_transfer_dma(DMA1, spi_line, bme280_tx_buffer, dummy_rx_buffer); // Hardcoding here

//   return SENSOR_OK;
// }

/**
 * @brief Writes data to a given register address - blocking
 */
static int8_t user_spi_write_blocking(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
  if(!reg_data || !intf_ptr) return SENSOR_ERR_INVALID_PARAM;

  // Cast intf_ptr to an SPI peripheral pointer
  SPI_TypeDef* spi_line = (SPI_TypeDef*)intf_ptr;

  reg_addr &= ~(1U << 7); // Clear the 7th bit of the register address

  // Initialise Transfer Buffer
  uint8_t bme280_tx_buffer[len+1];
  // Append the Register Address to the start
  bme280_tx_buffer[0] = reg_addr;
  // Fill the buffer with the rest of the data
  for(uint32_t i = 0; i < len; i++){
    bme280_tx_buffer[i+1] = reg_data[i];
  }

  spi_data_packet_t data_packet = {
    .buffer = (const void*)bme280_tx_buffer,
    .buffer_size = len + 1,
    .data_size = 8U
  };

  // Transfer with DMA
  spi_transmit_polling(spi_line, &data_packet);

  return SENSOR_OK;
}

/**
 * @brief Wrapper - Read Function for the BME280 API using the spi_transfer_polling() function
 */
static int8_t user_spi_read_blocking(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr){
  if(!reg_data || !intf_ptr) return SENSOR_ERR_INVALID_PARAM;

  SPI_TypeDef* spi_line = (SPI_TypeDef*)intf_ptr;

  // Set the 7th address bit to 1 - Indicates Read
  reg_addr |= 1 << 7U;

  uint8_t tx_buffer[len+1];
  tx_buffer[0] = reg_addr;

  // Fill the tx_buffer with dummy bytes to clock out the transfer
  for(uint32_t i = 0; i < len; i++){
    tx_buffer[i+1] = 0;
  }

  uint8_t retval = spi_transfer_polling(spi_line, tx_buffer, reg_data, len+1, 8U);
  if(retval != SPI_OK) return retval;


  return SENSOR_OK;
}

/**
 * @brief This function reads and returns sensor data from the bme280 module
 * @param[out] sensor_data Temperature, Barometric Pressure and Humidity sensor data
 * 
 * @retval Returns Sensor Data
 */
int8_t manager_read_sensor_data(uint8_t compensate_sensor, struct bme280_data* sensor_data, struct bme280_dev* dev){
  if(!sensor_data || !dev) return SENSOR_ERR_INVALID_PARAM;

  int8_t retval = bme280_get_sensor_data(compensate_sensor, sensor_data, dev);
  if (retval != BME280_OK) return retval;

  return SENSOR_OK;
}


/*======================= SPI-DMA INTERFACING FUNCTIONS =======================*/

// /**
//  * @brief initiates DMA transfer for the SPI peripheral
//  */
// static int8_t spi_transfer_dma(DMA_TypeDef* dma_line, SPI_TypeDef* spi_line, const uint8_t* tx_buffer, uint8_t* rx_buffer){
//   if(!spi_line || !tx_buffer || !rx_buffer) return SENSOR_ERR_INVALID_PARAM;
//   // The following is the disabling procedure for SPI+DMA transfers
//   spi_set_dmarxen(spi_line, SPI_DMA_RX_EN);
//   // Enable DMA streams for Tx and Rx in DMA registers, if the streams are used.
//   manager_dma_config(dma_line, tx_buffer, rx_buffer);
//   spi_set_dmatxen(spi_line, SPI_DMA_TX_EN);
//   spi_enable(spi_line);

//   return SENSOR_OK;
// }

// /**
//  * @brief disables DMA transfer for the SPI peripheral
//  */
// static int8_t spi_disable_dma(SPI_TypeDef* spi_line, DMA_Channel_TypeDef* dma_channel){
//   if(!spi_line) return SENSOR_ERR_INVALID_PARAM;

//   uint8_t rxonly_mode = 0;

//   // SPI-DMA disable procedure
//   //1. Disable DMA Tx and Rx Streams
//   dma_err_t dma_disable_err = DMA_OK;
//   dma_disable_channel(dma_channel, &dma_disable_err);
//   if(dma_disable_err != DMA_OK) return dma_disable_err;
//   // 2. SPI Disable Procedure
//   spi_disable_nonblocking(spi_line, rxonly_mode);
//   // 3. Disable DMA Tx and Rx buffers
//   spi_disable_dmatxen(spi_line);
//   spi_disable_dmarxen(spi_line);

//   return SENSOR_OK;
// }



