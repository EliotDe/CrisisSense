#include "unity/src/unity.h"
#include "src/drivers/usart_driver.h"
#include "mock_stm32l432xx.h"

// Text fixtures and setup
static USART_TypeDef mock_usart1;
static usart_config_t test_config;

void setUp(void){
    // Reset mock USART before test

    // Initialise test config with valid defaults
    test_config = (usart_config_t) {
        .usart_line = &mock_usart1,
        .baud_rate = 9600,
        .f_clk = 8000000, // 8MHz
        .oversampling = USART_OVER16,
        .word_length = USART_WORD_LENGTH_8,
        .parity = USART_PARITY_NONE,
        .stop_bits = USART_STOP_1,
        .auto_baud = USART_AUTOBAUD_DISABLED
    };
}

void tearDown(void){
    // Cleanup
    mock_usart1.CR1 &= ~USART_CR1_UE;
}

/*============ PARAMETER VALIDATION TESTS ============*/

void test_usart_config_line_null_config_returns_error(void){
    usart_err_t error;

    uint8_t result = usart_config_line(NULL, &error);

    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(USART_ERR_INVALID_PARAM, error);
}

void test_usart_config_line_zero_baud_rate_returns_error(void) {
    usart_err_t error;
    test_config.baud_rate = 0;
    
    uint8_t result = usart_config_line(&test_config, &error);
    
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(USART_ERR_INVALID_PARAM, error);
}

void test_usart_config_line_usart_already_enabled_returns_busy(void) {
    usart_err_t error;
    mock_usart1.CR1 = USART_CR1_UE;  // Set USART as enabled
    
    uint8_t result = usart_config_line(&test_config, &error);
    
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(USART_ERR_BUSY, error);

    tearDown();
}

void test_usart_config_line_invalid_stop_bits_returns_error(void) {
    usart_err_t error;
    test_config.stop_bits = 99;  // Invalid value
    
    uint8_t result = usart_config_line(&test_config, &error);
    
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(USART_ERR_INVALID_PARAM, error);
}

/*============ BAUD RATE CALCULATION TESTS ============*/

// Test the OVER16 baud rate calculation
void test_baud_rate_over16_9600_8mhz(void){
    usart_err_t error = USART_OK;
    test_config.baud_rate = 9600;
    test_config.f_clk = 8000000;
    test_config.oversampling = USART_OVER16;

    uint8_t result = usart_config_line(&test_config, &error);

    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(USART_OK, error);
    // From Reference Manual: USARTDIV = 8000000/9600 = 933 (0x341)
    TEST_ASSERT_EQUAL_HEX(0x341, mock_usart1.BRR);
}

// Test OVER8 baud rate calculation
void test_baud_rate_over8_9600_8mhz(void){
    usart_err_t error = USART_OK;
    test_config.baud_rate = 9600;
    test_config.f_clk = 8000000;
    test_config.oversampling = USART_OVER8;

    uint8_t result = usart_config_line(&test_config, &error);

    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(USART_OK, error);
    TEST_ASSERT_EQUAL_HEX(0x681, mock_usart1.BRR);
}

void test_baud_rate_usartdiv_too_large_returns_overflow(void) {
    usart_err_t error = USART_OK;
    test_config.baud_rate = 1;  // Will cause huge USARTDIV
    test_config.f_clk = 8000000;
    
    uint8_t result = usart_config_line(&test_config, &error);
    
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(USART_ERR_OVERFLOW, error);
}

void test_baud_rate_usartdiv_too_small_returns_error(void) {
    usart_err_t error = USART_OK;
    test_config.baud_rate = 1000000;  // High baud rate
    test_config.f_clk = 8000;         // Low clock
    
    uint8_t result = usart_config_line(&test_config, &error);
    
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(USART_ERR_INVALID_PARAM, error);
}

/*============ REGISTER CONFIGURATION TESTS ============*/

void test_oversampling_over16_clears_over8_bit(void){
    usart_err_t error = USART_OK;
    mock_usart1.CR1 = USART_CR1_OVER8;          // Start with OVER8 set
    test_config.oversampling = USART_OVER16;

    usart_config_line(&test_config, &error);

    TEST_ASSERT_FALSE(mock_usart1.CR1 & USART_CR1_OVER8);
}

void test_oversampling_over8_sets_over8_bit(void) {
    usart_err_t error = USART_OK;
    test_config.oversampling = USART_OVER8;

    usart_config_line(&test_config, &error);

    TEST_ASSERT_TRUE(mock_usart1.CR1 & USART_CR1_OVER8);
}

void test_word_length_8_bit_clears_both_m_bits(void) {
    usart_err_t error = USART_OK;
    mock_usart1.CR1 = USART_CR1_M0 | USART_CR1_M1;  // Start with both set
    test_config.word_length = USART_WORD_LENGTH_8;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_FALSE(mock_usart1.CR1 & USART_CR1_M0);
    TEST_ASSERT_FALSE(mock_usart1.CR1 & USART_CR1_M1);
}

void test_word_length_9_bit_sets_m0_clears_m1(void) {
    usart_err_t error = USART_OK;
    test_config.word_length = USART_WORD_LENGTH_9;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_TRUE(mock_usart1.CR1 & USART_CR1_M0);
    TEST_ASSERT_FALSE(mock_usart1.CR1 & USART_CR1_M1);
}

void test_word_length_7_bit_sets_m1_clears_m0(void) {
    usart_err_t error = USART_OK;
    test_config.word_length = USART_WORD_LENGTH_7;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_FALSE(mock_usart1.CR1 & USART_CR1_M0);
    TEST_ASSERT_TRUE(mock_usart1.CR1 & USART_CR1_M1);
}

void test_parity_none_clears_pce_bit(void) {
    usart_err_t error = USART_OK;
    mock_usart1.CR1 = USART_CR1_PCE;  // Start with parity enabled
    test_config.parity = USART_PARITY_NONE;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_FALSE(mock_usart1.CR1 & USART_CR1_PCE);
}

void test_parity_even_sets_pce_clears_ps(void) {
    usart_err_t error = USART_OK;
    test_config.parity = USART_PARITY_EVEN;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_TRUE(mock_usart1.CR1 & USART_CR1_PCE);
    TEST_ASSERT_FALSE(mock_usart1.CR1 & USART_CR1_PS);
}

void test_parity_odd_sets_both_pce_and_ps(void) {
    usart_err_t error = USART_OK;
    test_config.parity = USART_PARITY_ODD;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_TRUE(mock_usart1.CR1 & USART_CR1_PCE);
    TEST_ASSERT_TRUE(mock_usart1.CR1 & USART_CR1_PS);
}


/*============ AUTO-BAUD TESTS ============*/

void test_auto_baud_disabled_does_not_set_abren(void) {
    usart_err_t error = USART_OK;
    test_config.auto_baud = USART_AUTOBAUD_DISABLED;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_FALSE(mock_usart1.CR2 & USART_CR2_ABREN);
}

void test_auto_baud_start_bit_enables_abren_with_mode_00(void) {
    usart_err_t error = USART_OK;
    test_config.auto_baud = USART_AUTOBAUD_START_BIT;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_TRUE(mock_usart1.CR2 & USART_CR2_ABREN);
    TEST_ASSERT_FALSE(mock_usart1.CR2 & USART_CR2_ABRMODE_Msk);  // Mode 00
}

void test_auto_baud_7f_frame_sets_correct_mode(void) {
    usart_err_t error = USART_OK;
    test_config.auto_baud = USART_AUTOBAUD_7F_FRAME;
    
    usart_config_line(&test_config, &error);
    
    TEST_ASSERT_TRUE(mock_usart1.CR2 & USART_CR2_ABREN);
    TEST_ASSERT_EQUAL(USART_CR2_ABRMODE_1, mock_usart1.CR2 & USART_CR2_ABRMODE_Msk);
}

/*============ "INTEGRATION TESTS" ============*/

void test_successful_configuration_enables_usart(void){
    usart_err_t error = USART_OK;

    uint8_t result = usart_config_line(&test_config, &error);

    TEST_ASSERT_EQUAL(1, result);
    TEST_ASSERT_EQUAL(USART_OK, error);
    TEST_ASSERT_TRUE(mock_usart1.CR1 & USART_CR1_UE);
}

/*============ TEST RUNNER ============*/

int main(void){
    UNITY_BEGIN();

    RUN_TEST(test_usart_config_line_null_config_returns_error);
    RUN_TEST(test_usart_config_line_zero_baud_rate_returns_error);
    RUN_TEST(test_usart_config_line_usart_already_enabled_returns_busy);
    RUN_TEST(test_usart_config_line_invalid_stop_bits_returns_error);
    
    // Baud rate calculation tests
    RUN_TEST(test_baud_rate_over16_9600_8mhz);
    RUN_TEST(test_baud_rate_over8_9600_8mhz);  // This will catch your OVER8 bug!
    RUN_TEST(test_baud_rate_usartdiv_too_large_returns_overflow);
    RUN_TEST(test_baud_rate_usartdiv_too_small_returns_error);
    
    // Register configuration tests
    RUN_TEST(test_oversampling_over16_clears_over8_bit);
    RUN_TEST(test_oversampling_over8_sets_over8_bit);
    RUN_TEST(test_word_length_8_bit_clears_both_m_bits);
    RUN_TEST(test_word_length_9_bit_sets_m0_clears_m1);
    RUN_TEST(test_word_length_7_bit_sets_m1_clears_m0);
    RUN_TEST(test_parity_none_clears_pce_bit);
    RUN_TEST(test_parity_even_sets_pce_clears_ps);
    RUN_TEST(test_parity_odd_sets_both_pce_and_ps);
    
    // Auto baud tests
    RUN_TEST(test_auto_baud_disabled_does_not_set_abren);
    RUN_TEST(test_auto_baud_start_bit_enables_abren_with_mode_00);
    RUN_TEST(test_auto_baud_7f_frame_sets_correct_mode);
    
    // Integration tests
    RUN_TEST(test_successful_configuration_enables_usart);

    return UNITY_END();
}