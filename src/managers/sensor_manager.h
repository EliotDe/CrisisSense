#ifndef LOGGING_MANAGER_H
#define LOGGING_MANAGER_H

#include "stm32l432xx.h"
#include "bme280_defs.h"

typedef enum{
  SENSOR_OK,
  SENSOR_ERR = -1,
  SENSOR_ERR_INVALID_PARAM = -2
} sensor_err_t;

// /**
//  * @brief Reads data from a given register address - non-blocking
//  * 
//  * @param[in] reg_addr Register Address to Read From
//  * @param[out] reg_data Data in the Register
//  * @param[in] len Length of the data transfer
//  * @param[in] intf_ptr Pointer to the interfacing peripheral (SPI, I2C)
//  * 
//  * @retval 0 (SENSOR_OK) If Read is Succesful
//  * @retval <0 (SENSOR_ERR_X) If an error has occured
//  * 
//  * @note The BME280 API expects blocking functions, if you're passing a pointer to this function for BME280 read() calls
//  * make sure the read completes before the API calls it again.
//  */
// int8_t user_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);

// /**
//  * @brief Writes data to a given register address - non-blocking
//  * 
//  * @param reg_addr Register Address to Write to
//  * @param reg_data The data to write to the register
//  * @param len The length of the data to write
//  * @param intf_ptr Pointer to the interfacing peripheral (SPI, I2C)
//  * 
//  * @note The BME280 API expects blocking functions, if you're passing a pointer to this function for BME280 write() calls
//  * make sure the write completes before the API calls it again.
//  */
// int8_t user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

// /**
//  * @brief Reads data from a given register address - blocking
//  * 
//  * @param[in] reg_addr Register Address to Read From
//  * @param[out] reg_data Data in the Register
//  * @param[in] len Length of the data transfer
//  * @param[in] intf_ptr Pointer to the interfacing peripheral (SPI, I2C)
//  * 
//  * @retval 0 (SENSOR_OK) If Read is Succesful
//  * @retval <0 (SENSOR_ERR_X) If an error has occured
//  */
// int8_t user_spi_read_blocking(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);

// /**
//  * @brief Writes data to a given register address - blocking
//  * 
//  * @param reg_addr Register Address to write to 
//  * @param reg_data data to write to the register
//  * @param len Length of the data to write
//  * @param intf_ptr Interfacing Device (SPI or I2C) pointer
//  * 
//  * @retval 0 (SENSOR_OK) Everything went as expected
//  * @retval <0 (SENSOR_ERR_X) An error occured
//  */
// int8_t user_spi_write_blocking(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief Reads Sensor Data
 */
int8_t manager_read_sensor_data(uint8_t compensate_sensor, char* sensor_data);

int8_t sensor_manager_init();

int8_t dwt_init(void);

void user_delay_us(uint32_t period, void* intf_ptr);

#endif