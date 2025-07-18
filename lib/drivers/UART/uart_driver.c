/**
 * @file uart_driver.c
 *
 * @brief STM32F4 USART2 Driver Implementation.
 *
 * This file provides the implementation for initializing and utilizing USART2
 * for UART communication on the STM32F4 microcontroller. It supports sending
 * single characters and strings through the UART interface.
 *
 * Features:
 * - Initializes USART2 with a baud rate of 115200.
 * - Configures GPIO pins PA2 (TX) and PA3 (RX) for UART communication.
 * - Provides functions for transmitting characters and strings.
 *
 * Dependencies:
 * - STM32F4xx Standard Peripheral Library
 * - GPIO and USART drivers
 *
 * Hardware:
 * - Board: STM32F4-Discovery
 * - USART2 TX: PA2
 * - USART2 RX: PA3
 *
 * @note Ensure proper RCC configurations and GPIO initialization before use.
 *
 * @copyright Radar2000
 * This work is licensed under Creative Commons
 * Attribution-NonCommercial-ShareAlike 4.0 International License.
 */

#include <uart_driver.h>

void uart_init(void) {
  // GPIO Configuration
  GPIO_InitTypeDef GPIO_InitStructure;

  // Enable GPIOA clock (if using PA2 and PA3)
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // Enable USART2 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // Configure USART2_TX (PA2)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

  // Configure USART2_RX (PA3)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  // USART2 Configuration
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);

  // Enable USART2
  USART_Cmd(USART2, ENABLE);
}

/**
 * @brief Transmits a single character over USART2.
 *
 * This function waits until the USART2 transmit buffer is empty before sending
 * the specified character over the USART2 interface.
 *
 * @param c The character to send.
 */
static void uart_send_char(char c) {
  // Wait until transmit buffer is empty
  while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
  }
  USART_SendData(USART2, c);
}

/**
 * @brief Transmits a string over USART1.
 *
 * @param str Pointer to the null-terminated string to send.
 */
static void uart_send_string(const char *str) {
  while (*str) {
    uart_send_char(*str++);
  }
}

// -------------- Helper Functions for Custom Print Formatted -------------- //
/**
 * @brief Print a signed long integer (decimal) without recursion.
 */
static void print_signed_long(long val) {
  char buffer[12]; // Enough for "-2147483648\0"
  int i = 0;
  int is_negative = (val < 0);

  if (is_negative) {
    val = -val; // Make positive
  }

  // Convert in reverse
  if (val == 0) {
    buffer[i++] = '0';
  } else {
    while (val > 0) {
      buffer[i++] = (char)('0' + (val % 10));
      val /= 10;
    }
  }

  // Add sign if negative
  if (is_negative) {
    buffer[i++] = '-';
  }

  // Terminate
  buffer[i] = '\0';

  // Reverse the string
  for (int start = 0, end = i - 1; start < end; start++, end--) {
    char tmp = buffer[start];
    buffer[start] = buffer[end];
    buffer[end] = tmp;
  }

  // Send
  uart_send_string(buffer);
}

/**
 * @brief Print an unsigned long integer (decimal) without recursion.
 */
static void print_unsigned_long(unsigned long val) {
  char buffer[11]; // Enough for "4294967295\0"
  int i = 0;

  if (val == 0) {
    buffer[i++] = '0';
  } else {
    while (val > 0) {
      buffer[i++] = (char)('0' + (val % 10));
      val /= 10;
    }
  }

  // Terminate
  buffer[i] = '\0';

  // Reverse the string
  for (int start = 0, end = i - 1; start < end; start++, end--) {
    char tmp = buffer[start];
    buffer[start] = buffer[end];
    buffer[end] = tmp;
  }

  uart_send_string(buffer);
}

/**
 * @brief Print an unsigned long integer as hexadecimal without recursion.
 */
static void print_hex(unsigned long val) {
  char buffer[9]; // 8 hex digits + '\0'
  int i = 0;

  if (val == 0) {
    buffer[i++] = '0';
  } else {
    while (val > 0) {
      unsigned long nibble = val & 0xF;
      if (nibble < 10) {
        buffer[i++] = (char)('0' + nibble);
      } else {
        buffer[i++] = (char)('A' + (nibble - 10));
      }
      val >>= 4;
    }
  }

  buffer[i] = '\0';

  // Reverse
  for (int start = 0, end = i - 1; start < end; start++, end--) {
    char tmp = buffer[start];
    buffer[start] = buffer[end];
    buffer[end] = tmp;
  }

  uart_send_string(buffer);
}

/**
 * @brief Print a floating-point number with a fixed number of decimal places
 *        (kDecimalPlaces), avoiding recursion.
 */
static void print_float(double value) {
  const int kDecimalPlaces = 6;

  // Handle sign
  if (value < 0.0) {
    uart_send_char('-');
    value = -value;
  }

  // Integer part
  long int_part = (long)value;

  // Fractional part
  double fractional = value - (double)int_part;

  // Print integer part using the signed integer helper
  print_signed_long(int_part);

  // Print decimal point
  uart_send_char('.');

  // Multiply fractional by 10^kDecimalPlaces
  double power = 1.0;
  for (int i = 0; i < kDecimalPlaces; i++) {
    power *= 10.0;
  }

  // Apply rounding
  fractional *= power;
  fractional += 0.5;
  unsigned long frac_part = (unsigned long)fractional;
  if (frac_part >= (unsigned long)power) {
    // e.g. 0.9999 => 1.0000
    frac_part = (unsigned long)power - 1;
  }

  // Print fractional with leading zeros if necessary
  // E.g. if kDecimalPlaces=2 and frac_part=3 => "03"
  for (int i = kDecimalPlaces - 1; i >= 0; i--) {
    unsigned long divisor = 1;
    for (int j = 0; j < i; j++) {
      divisor *= 10;
    }
    unsigned digit = (frac_part / divisor) % 10;
    uart_send_char((char)('0' + digit));
  }
}

/**
 * @brief Convert an integer (signed or unsigned) to a string and send it over
 * UART. Handles signed decimal, unsigned decimal, and hexadecimal.
 *
 * @param value  The integer value to print (pass as long for convenience).
 * @param format The format specifier (PRINT_SIGNED_DEC, PRINT_UNSIGNED_DEC,
 * PRINT_HEX).
 */
static void uart_print_type(double value, PrintFormat_t format) {
  switch (format) {
  case PRINT_SIGNED_DEC:
    print_signed_long((long)value);
    break;

  case PRINT_UNSIGNED_DEC:
    print_unsigned_long((unsigned long)value);
    break;

  case PRINT_HEX:
    print_hex((unsigned long)value);
    break;

  case PRINT_FLOAT:
    print_float(value);
    break;
  }
}

// ------------------ Minimal Custom Print Formatted ------------------ //

void uart_print(const char *format, ...) {
  va_list args;
  va_start(args, format);

  while (*format) {
    if (*format == '%') {
      format++;
      switch (*format) {
      case 'd': {
        int val = va_arg(args, int);
        uart_print_type((long)val, PRINT_SIGNED_DEC);
        break;
      }
      case 'u': {
        unsigned int val = va_arg(args, unsigned int);
        uart_print_type((long)val, PRINT_UNSIGNED_DEC);
        break;
      }
      case 'x': {
        unsigned int val = va_arg(args, unsigned int);
        uart_print_type((long)val, PRINT_HEX);
        break;
      }
      case 'f': {
        double val = va_arg(args, double);
        uart_print_type(val, PRINT_FLOAT);
        break;
      }
      case 'c': {
        char c = (char)va_arg(args, int);
        uart_send_char(c);
        break;
      }
      case 's': {
        const char *s = va_arg(args, char *);
        if (s) {
          uart_send_string(s);
        } else {
          uart_send_string("(null)");
        }
        break;
      }
      default: {
        // Unknown specifier, just print it as normal char
        uart_send_char(*format);
        break;
      }
      }
    } else {
      // Normal character
      uart_send_char(*format);
    }
    format++;
  }

  va_end(args);
}
