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

/* The main function is in init9, which removes the interrupt vector table */
/* we don't need. It is also 'naked', which means the compiler does not    */
/* generate any entry or exit code itself. */
int main(void) __attribute__ ((OS_main)) __attribute__ ((noreturn)) __attribute__ ((section (".init9")));

//void led_init(void);
void uart_putc(char ch);
char uart_getc(void);
void appStart(void);
static uint8_t bootloader_decode_address(uint16_t *address);
//void flash_read(uint16_t address, uint8_t *buf, uint16_t len);
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


  /* Okay so we've arrived here.
   * The possible sources of ATxmega reset are :-
   * Power on reset.
   * External reset.
   * Brownout reset.
   * Watchdog reset.
   * PDI reset.
   * Software reset.
   * or we could have jumped here
   * or strayed from some dubious code.
   * After a reset condition everything is reset except SRAM.
   * Clock is 2MHz.
   */


  if(! (RST.STATUS & RST_PORF_bm))  appStart();//

  // Check if we have to enter the bootloader
  // or jump to the application
  // wait some time for the voltage level on i/o to
  // stabilise

  BL_PORT.DIRCLR = (1 << BL_PIN);
  BL_PIN_CTRL_REG = PORT_OPC_PULLUP_gc; // Pullup

  _delay_ms(100);

  if (! (BL_PIN_ACTIVE))  appStart();

  // LED_init
  BL_LED_PORT.DIRSET = (1 << BL_LED_PIN);
  LED_ON;

  // Beeper init
  BEEPER_PORT.DIRSET = (1 << BEEPER_PIN);
  BEEPER_ON;


#if 1
#if (F_CPU != 32000000 && F_CPU != 2000000)
#error Oscillator can only be 2 or 32 MHz.
#endif // F_CPU

#if F_CPU == 2000000
#if defined (USE_DFLL) && defined (DFLLRC2M) // x32e5 does not have 2MHz DFLL.
  OSC.CTRL |= OSC_RC32KEN_bm;
  while(!(OSC.STATUS & OSC_RC32KRDY_bm));
  DFLLRC2M.CTRL = DFLL_ENABLE_bm;
#endif // USE_DFLL
#endif // F_CPU == 2000000

#if F_CPU == 32000000
  OSC.CTRL |= OSC_RC32MEN_bm; // Enable the internal 32MHz oscillator.
  while(!(OSC.STATUS & OSC_RC32MRDY_bm)); // Wait for 32MHz oscillator to stabilise.
  _PROTECTED_WRITE(CLK.CTRL, CLK_SCLKSEL_RC32M_gc); // Select RC32MHz as source.
#ifdef USE_DFLL
  OSC.CTRL |= OSC_RC32KEN_bm;
  while(!(OSC.STATUS & OSC_RC32KRDY_bm));
  DFLLRC32M.CTRL = DFLL_ENABLE_bm;
#endif // USE_DFLL
#endif // F_CPU == 32000000
#endif


//#define CLOCK_OUT
  #if defined (CLOCK_OUT)
    // Output ClkPER on PC7.
    PORTCFG.CLKEVOUT = PORTCFG_CLKOUT_PC7_gc;
    PORTC.DIRSET = PIN7_bm;
    while (1);
  #endif


  // Manual says set TXD pin high and output.
  PORTC.OUTSET = TXD_PIN;
  PORTC.DIRSET = TXD_PIN;

  USART_SET_BAUD_115K2(USARTC0);
  USART_SET_MODE_8E1(USARTC0);
  USART_ENABLE_TX(USARTC0);
  USART_ENABLE_RX(USARTC0);

  _delay_ms(100);
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

        // read direct from flash to usart.
        do uart_putc(Flash_ReadByte(address++));
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
  myfuncptr_t call_app;
  call_app = 0;

  if (pgm_read_byte((uint16_t) call_app) != 0xFF)
  {
    // Undo Bootloader configuration.
    // deactivate BEEPER
    BEEPER_OFF;
    BEEPER_PORT.DIRCLR = (1 << BEEPER_PIN);
    BL_PIN_CTRL_REG = 0;
    USART_DISABLE_TX(USARTC0);
    USART_DISABLE_RX(USARTC0);
    PORTC.DIRCLR = TXD_PIN;
    LED_OFF;
    RST.STATUS |= RST_PORF_bm;
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

#if 0
// NOTE: this will read len+1 bytes to buffer buf
void flash_read(uint16_t address, uint8_t *buf, uint16_t len)
{

  // copy len bytes to buf
  while (len--)
  {
    *buf++ = Flash_ReadByte(address++);
  }
}
#endif

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

