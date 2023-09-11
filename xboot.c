/************************************************************************/
/* STM32 compatible serial bootloader for the OrangeRX ORX Transmitter  */
/* Module.                                                              */
/* Code based on:                                                       */
/* Xboot by Alex Forencich.        <alex@alexforencich.com>             */
/* OpenSky_BL by fishpepper.       fishpepper <AT> gmail.com            */
/* Adapted by sloped-soarer. https://github.com/sloped-soarer           */
/************************************************************************/

#include "xboot.h"
#include "main.h"
#include <avr/pgmspace.h>

// ATxmega32d4 has flash page size of 256 Bytes.
// STM32F103xx which we emulate has 1024 Bytes.
// We will need to erase 4 pages for any page given by STM bootloader.
#define PAGECOUNT_BOOTLOADER ((BOOT_SECTION_SIZE) / (BOOT_SECTION_PAGE_SIZE))
#define PAGECOUNT_APP      ((APP_SECTION_SIZE) / (SPM_PAGESIZE))

#define USART_SET_BAUD_115K2(usartx)  _USART_SET_BAUD(usartx, 131,-3)

#define _USART_SET_BAUD(usartx, bsel, bscale) \
  { usartx.BAUDCTRLA = bsel & 0xFF; usartx.BAUDCTRLB = ((int8_t) bscale << USART_BSCALE_gp) | (bsel >> 8); }

#define USART_SET_MODE_8E1(usartx) \
  { usartx.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_EVEN_gc | (0 << USART_SBMODE_bp) | USART_CHSIZE_8BIT_gc; } // 8E1

#define USART_ENABLE_TX(usartxn) usartxn.CTRLB |= USART_TXEN_bm;

#define USART_PURGE_RX(usartxn) { while (usartxn.STATUS & USART_RXCIF_bm) (void) usartxn.DATA; }

#define USART_ENABLE_RX(usartxn) { \
    usartxn.CTRLB |= USART_RXEN_bm; \
    USART_PURGE_RX(usartxn); \
  } // Enable RX. Flush RX.

#define USART_DISABLE_TX(usartxn) { \
    usartxn.CTRLB &= ~USART_TXEN_bm; \
  } // Disable TX.

#define USART_DISABLE_RX(usartxn) { \
	usartxn.CTRLB &= ~USART_RXEN_bm; \
  } // Disabling RX flushes buffer and clears RXCIF.

// LED tied high
#define BL_LED_PIN  1
#define BL_LED_PORT PORTD
#define LED_ON  BL_LED_PORT.OUTCLR = (1 << BL_LED_PIN)
#define LED_OFF BL_LED_PORT.OUTSET = (1 << BL_LED_PIN)
#define LED_TOGGLE LED_PORT.OUTTGL = (1 << BL_LED_PIN)

// "BIND" button to ground.
#define BIND_PIN 2
#define BIND_PORT PORTD
#define BIND_PIN_ACTIVE ((BIND_PORT.IN & (1 << BIND_PIN)) == 0)

// BOOTLOADER ACTIVATION aka "ID" button to ground.
#define BL_PIN 0
#define BL_PORT PORTD
#define BL_PIN_ACTIVE ((BL_PORT.IN & (1 << BL_PIN)) == 0)
#define BL_PIN_CTRL_REG  token_paste3(BL_PORT.PIN, BL_PIN, CTRL) // e.g. "PORTx.PINnCTRL"

// Beeper
#define BEEPER_PIN 5
#define BEEPER_PORT PORTC
#define BEEPER_ON  BEEPER_PORT.OUTSET = (1 << BEEPER_PIN)
#define BEEPER_OFF BEEPER_PORT.OUTCLR = (1 << BEEPER_PIN)

#define TXD_PIN PIN3_bm;

/* Function Prototypes */

void led_init(void);
void uart_putc(char ch);
char uart_getc(void);
void appStart(void);
static uint8_t bootloader_decode_address(uint16_t *address);
void flash_read(uint16_t address, uint8_t *buf, uint16_t len);
uint8_t flash_write_data(uint16_t address, uint8_t *buf, uint16_t len);
uint8_t flash_erase_page_1KB(uint8_t page);

#ifdef __AVR_XMEGA__
unsigned char comm_mode;
#else // __AVR_XMEGA__
// Force data section on atmega
// Seems to be a bug in newer versions of gcc
// this ensures .bss is placed after .data
unsigned char comm_mode = 1;
#endif // __AVR_XMEGA__

#define BUFFER_SIZE  SPM_PAGESIZE * 2 // To be safe, we don't know if the programmer will make us cross a 256 page boundary.
uint8_t buffer[BUFFER_SIZE];

int main(void)
{
  myfuncptr_t jump_helper;
  uint8_t state = 0;
  uint8_t command = 0;
  uint8_t rx = 0;
  uint16_t address;
  uint8_t *data_ptr = 0;
  uint8_t checksum;
  uint8_t len = 0;
  uint8_t page_n = 0;

#ifdef __AVR_XMEGA__
  /* Okay so we've arrived here.
   * The possible sources of ATxmega reset are :-
   * Power on reset.
   * External reset.
   * Watchdog reset.
   * Brownout reset.
   * PDI reset.
   * Software reset.
   * or we could have jumped here
   * or strayed from some dubious code.
   * After a reset condition everything is reset except SRAM.
   * Clock is 2MHz.
   */

  // Bootloader does not use watchdog so must be the Application.
  // If Brownout reset and application needs to run then treat as watchdog reset.
  if(RST.STATUS & (RST_WDRF_bm | RST_BORF_bm)) appStart();//

  // Check reset conditions.
  // RST.STATUS bits 6 and 7 aren't used according to the manual (note bit 6 is defined as RST_SDRF_bm (Spike Detection) in nearly every header).

  register uint8_t mask = RST_SRF_bm | RST_PDIRF_bm | RST_WDRF_bm | RST_BORF_bm | RST_EXTRF_bm | RST_PORF_bm;

#ifdef RST_SDRF_bm
  mask |= RST_SDRF_bm;
#endif

  // No valid reset condition so do a software reset.
  // Best way to call bootloader is via a Software Reset.
  if (! (RST.STATUS & mask))
  {
    _PROTECTED_WRITE(RST.CTRL, RST_SWRST_bm);
    while(1); // Should not execute according to the manual.
  }
#endif

  // check if we have to enter the bootloader
  // or jump to the application
  // wait some time for the voltage level on i/o to
  // stabilize

  BL_PORT.DIRCLR = (1 << BL_PIN);
  BL_PIN_CTRL_REG = PORT_OPC_PULLUP_gc; // Pullup

  _delay_ms(100);

  if (!(BL_PIN_ACTIVE))
  {
    appStart();
  }

  // LED_init
  BL_LED_PORT.DIRSET = (1 << BL_LED_PIN);
  LED_ON;

  // Beeper init
  BEEPER_PORT.DIRSET = (1 << BEEPER_PIN);
  BEEPER_ON;

  // Enable Crystal Oscillator so we can go fast and accurate.
  // Enable external oscillator (16MHz)
  OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_256CLK_gc;
  OSC.CTRL |= OSC_XOSCEN_bm;
  while ((OSC.STATUS & OSC_XOSCRDY_bm) == 0)
    /* wait */;
  // Enable PLL (2* 16MHz Crystal = 32MHz)
  OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | 2;
  OSC.CTRL |= OSC_PLLEN_bm;
  while ((OSC.STATUS & OSC_PLLRDY_bm) == 0)
    /* wait */;
  // Switch to PLL clock
  CPU_CCP = 0xD8;
  CLK.CTRL = CLK_SCLKSEL_RC2M_gc;
  CPU_CCP = 0xD8;
  CLK.CTRL = CLK_SCLKSEL_PLL_gc;

  OSC.CTRL &= ~OSC_RC2MEN_bm; // Disable 2MHz oscillator which was enabled after reset.

  // Manual says set TXD pin high and output.
  PORTC.OUTSET = TXD_PIN;
  PORTC.DIRSET = TXD_PIN;

  USART_SET_BAUD_115K2(USARTC0);
  USART_SET_MODE_8E1(USARTC0);
  USART_ENABLE_TX(USARTC0);
  USART_ENABLE_RX(USARTC0);

  _delay_ms(1000);
  BEEPER_OFF;

  // the bootloader enable pin was active or
  // there was no valid code uploaded yet -> enter bootloader mode
  for (;;)
  {
    // uart_putc_d(state);
    // do main statemachine
    switch (state)
    {
      default:
      case (0):
        // fetch command byte
        command = uart_getc();
        if (command == BOOTLOADER_COMMAND_INIT)
        {
          // init sequence, send ack
          uart_putc(BOOTLOADER_RESPONSE_ACK);
        }
        else
        {
          // real command
          state = 1;
        }
        break;

      case (1):
        // check command checksum (inverted)
        rx = uart_getc();
        // NOTE: ~x seems to be calculated in uint16_t !
        if (rx == (command ^ 0xFF))
        {
          // fine, valid command -> decode
          switch (command)
          {
            // unknown or unsupported command
            default:
              // invalid command, abort
              state = 0xFF;
              break;

              // all known commands
            case (BOOTLOADER_COMMAND_GET):
            case (BOOTLOADER_COMMAND_GET_VERSION):
            case (BOOTLOADER_COMMAND_GET_ID):
            case (BOOTLOADER_COMMAND_READ_MEMORY):
            case (BOOTLOADER_COMMAND_GO):
            case (BOOTLOADER_COMMAND_WRITE_MEMORY):
            case (BOOTLOADER_COMMAND_ERASE):
              // send ACK and continue with command handler
              uart_putc(BOOTLOADER_RESPONSE_ACK);
              state = 10 + command;
              break;
          }
        }
        else
        {
          // mismatch - this was either a comm error or we are
          // in the middle of a command, retry with the current byte as cmd byte:
          if (rx == BOOTLOADER_COMMAND_INIT)
          {
            // init sequence, send ack
            uart_putc(BOOTLOADER_RESPONSE_ACK);
            state = 0;
          }
          else
          {
            // real command
            command = rx;
          }
        }
        break;

        // send GET response
      case (10 + BOOTLOADER_COMMAND_GET):
        // number of command bytes that will follow
        uart_putc(7);
        // version
        uart_putc(BOOTLOADER_VERSION);
        // send supported commands
        uart_putc(BOOTLOADER_COMMAND_GET);
        uart_putc(BOOTLOADER_COMMAND_GET_VERSION);
        uart_putc(BOOTLOADER_COMMAND_GET_ID);
        uart_putc(BOOTLOADER_COMMAND_READ_MEMORY);
        uart_putc(BOOTLOADER_COMMAND_GO);
        uart_putc(BOOTLOADER_COMMAND_WRITE_MEMORY);
        uart_putc(BOOTLOADER_COMMAND_ERASE);
        // send ack
        uart_putc(BOOTLOADER_RESPONSE_ACK);
        // wait for next command
        state = 0;
        break;

        // send GET_ID response
      case (10 + BOOTLOADER_COMMAND_GET_ID):
        // number of response bytes to follow
        uart_putc(1);
        // send product id of an F1 chip with the same flash size 32KB.
        uart_putc(BOOTLOADER_DEVICE_ID >> 8);
        uart_putc(BOOTLOADER_DEVICE_ID & 0xFF);
        // send ack
        uart_putc(BOOTLOADER_RESPONSE_ACK);
        // wait for next command
        state = 0;
        break;

        // send GET_VERSION response
      case (10 + BOOTLOADER_COMMAND_GET_VERSION):
        // bootloader version
        uart_putc(BOOTLOADER_VERSION);
        // send option bytes
        uart_putc(0x00);
        uart_putc(0x00);
        // send ack
        uart_putc(BOOTLOADER_RESPONSE_ACK);
        // wait for next command
        state = 0;
        break;

        // send READ_MEMORY response
      case (10 + BOOTLOADER_COMMAND_READ_MEMORY):
        if (!bootloader_decode_address(&address))
        {
          // abort now
          state = 0xFF;
          break;
        }

        // addresss is valid, send ack
        uart_putc(BOOTLOADER_RESPONSE_ACK);

        // fetch data
        len = uart_getc();
        checksum = uart_getc();

        // verify checksum
        if (len != (checksum ^ 0xFF))
        {
          // checksum invalid -> abort here
          state = 0xFF;
          break;
        }

        // checksum test passed, send ack
        uart_putc(BOOTLOADER_RESPONSE_ACK);

        // fetch flash content (len+1 bytes!)
        flash_read(address, buffer, ((uint16_t) len) + 1);

        // send len+1 bytes
        data_ptr = &buffer[0];

        do
        {
          uart_putc(*data_ptr++);
        }
        while (len--);

        // wait for next command
        state = 0;
        break;

        // send GO response
      case (10 + BOOTLOADER_COMMAND_GO):
        if (!bootloader_decode_address(&address))
        {
          // abort now
          state = 0xFF;
          break;
        }

        // addresss is valid, send ack
        uart_putc(BOOTLOADER_RESPONSE_ACK);

        // now jump to user application given by address
        // ToDo maybe there is a use for this.

        jump_helper = (myfuncptr_t) address;
        jump_helper();
//        appStart();

        // wait for next command
        state = 0;
        break;

        // send WRITE_MEMORY response
      case (10 + BOOTLOADER_COMMAND_WRITE_MEMORY):
        if (!bootloader_decode_address(&address))
        {
          // abort now
          state = 0xFF;
          break;
        }

        // addresss is valid, send ack
        uart_putc(BOOTLOADER_RESPONSE_ACK);

//      uint32_t address = address / SPM_PAGESIZE;

        // INITIALISE BUFFER WITH 0xFF
        for (uint16_t i = 0; i < BUFFER_SIZE; i++)
        {
          buffer[i] = 0xFF;
        }

        // fetch len
        len = uart_getc();
        checksum = len;

        // place to store data
        data_ptr = &buffer[(address & (SPM_PAGESIZE - 1))]; // Start address may be mid-page and then overlap page boundary.

        // retrieve len data bytes
        do
        {
          rx = uart_getc();
          *data_ptr++ = rx;
          checksum ^= rx;
        }
        while (len--);

        // verify checksum
        rx = uart_getc();
        if (checksum != rx)
        {
          // checksum invalid -> abort here
          state = 0xFF;
          break;
        }

        // checksum ok  - flash data
        Flash_ProgramPage(address & ~(SPM_PAGESIZE - 1), &buffer[0], 0); // NO erase -- first page

        if ((address & (SPM_PAGESIZE - 1)) != 0)
        {
          LED_TOGGLE;
          Flash_ProgramPage(SPM_PAGESIZE + (address & (SPM_PAGESIZE - 1)),
              &buffer[SPM_PAGESIZE], 0); //  -- second page
        }
        // done
        uart_putc(BOOTLOADER_RESPONSE_ACK);

        // wait for next command
        state = 0;
        break;

        // send ERASE response
      case (10 + BOOTLOADER_COMMAND_ERASE):
        // get number of pages to be erased
        page_n = uart_getc();
        checksum = page_n;

        if ((page_n == 0xFF) && (uart_getc() == 0x00))
        {
          // special case, full flash erase
          Flash_EraseApplicationSection();
          Flash_WaitForSPM();

          // execute suceeded!
          uart_putc(BOOTLOADER_RESPONSE_ACK);

          // wait for next command
          state = 0;
          break;
        }

        // fetch n+1 pages to be erased.
        data_ptr = &buffer[0];
        len = page_n;

        do
        {
          rx = uart_getc();
          *data_ptr++ = rx;
          checksum ^= rx;
        }
        while (len--);

        // fetch checksum
        rx = uart_getc();

        if (rx != checksum)
        {
          // checksum mismatch, abort
          state = 0xFF;
          break;
        }

        // fine, the n+1 of page numbers to be erased are now in buffer[]
        // execute the erase of the page(s).
        // 1KB pages
        data_ptr = &buffer[0];
        len = page_n;

        do
        {
          if (!flash_erase_page_1KB(*data_ptr++))
          {
            state = 0xFF;
            break;
          }
        }
        while (len--);

        // execute suceeded!
        uart_putc(BOOTLOADER_RESPONSE_ACK);

        // wait for next command
        state = 0;
        break;

        // ABORT STATE - send nack and goto idle
      case (0xFF):
        uart_putc(BOOTLOADER_RESPONSE_NACK);
        state = 0;
        break;
    }
  }
}

uint8_t flash_erase_page_1KB(uint8_t page)
{
  // We are passed 1KB page numbers.
  // We need addresses in the page.

#define PAGE_MULT 1024/SPM_PAGESIZE

  if ((page * PAGE_MULT) + (PAGE_MULT - 1) >= PAGECOUNT_APP) return 0;

  for (uint8_t i = 0; i < PAGE_MULT; i++)
  {
    Flash_EraseApplicationPage(
        (uint32_t)(((page * PAGE_MULT) + i) * SPM_PAGESIZE));
    Flash_WaitForSPM();
  }
  return 1;
}

void appStart()
{
  // Undo some Bootloader functions
  // deactivate BEEPER
  BEEPER_OFF;
  BEEPER_PORT.DIRCLR = (1 << BEEPER_PIN);

  BL_PIN_CTRL_REG = 0;

  USART_DISABLE_TX(USARTC0);
  USART_DISABLE_RX(USARTC0);
  PORTC.DIRCLR = TXD_PIN;

  myfuncptr_t call_app;
  call_app = 0;

  if (pgm_read_byte((uint16_t) call_app) != 0xFF)
  {
    LED_OFF;
    (*call_app)();
  }
}

static uint8_t bootloader_decode_address(uint16_t *address)
{
  uint8_t rx;
  uint8_t checksum = 0;

  // as we only accept 16bit addresses
  // stm32 uses 0x0800xxyy as address for flash
  // ignore the first two bytes
  checksum ^= uart_getc();
  checksum ^= uart_getc();

  // high byte of 16bit
  rx = uart_getc();
  *address = rx;
  checksum ^= rx;

  // low byte of 16bit
  rx = uart_getc();
  *address = ((*address) << 8) | rx;
  checksum ^= rx;

  // read address checksum
  rx = uart_getc();

  // verify checksum
  if (rx != checksum)
  {
    // checksum invalid -> abort here
    return 0;
  }

  // verify if this is within memory bounds:
  if ((*address) > (APP_SECTION_END))
  { //FLASH_SIZE
    return 0;
  }

  // everything is fine
  return 1;
}

// NOTE: this will read len+1 bytes to buffer buf
void flash_read(uint16_t address, uint8_t *buf, uint16_t len)
{

  // copy len bytes to buf
  while (len--)
  {
    *buf++ = Flash_ReadByte(address++);
  }
}

void uart_putc(char ch)
{
  while (!(USARTC0.STATUS & USART_DREIF_bm));
  USARTC0.DATA = ch;
}

char uart_getc()
{
  uint8_t ch;

  while (!(USARTC0.STATUS & USART_RXCIF_bm));
  ch = USARTC0.DATA;
  return ch;
}

