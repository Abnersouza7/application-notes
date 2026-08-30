/*
 * AN0003 verification images for the NUCLEO-H503RB.
 *
 * Two images are built from this one source. Each prints a banner over the
 * ST-LINK virtual COM port and then a heartbeat line once a second, and blinks
 * the user LED. The only difference between them is the identifying letter and
 * the blink rate, which is enough to prove from the outside that a firmware
 * update actually took effect.
 *
 *   make            -> build/an0002_image_a.bin  and  build/an0002_image_b.bin
 *
 * Bare metal on purpose: no HAL, no CubeMX, no CMSIS. Everything the program
 * touches is written out below, so the whole example is one readable file.
 *
 * Board wiring, taken from the NUCLEO-H503RB schematic:
 *   PA4  USART3_TX (AF13)  ->  ST-LINK VCP, appears as a COM port on the host
 *   PA5  user LED (LD2), active high
 *
 * Serial format: 115200 baud, 8 data bits, no parity, 1 stop bit.
 *
 * Copyright (c) 2026 Binho Inc. Released under the MIT license; see LICENSE.
 */

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Which image is this?  IMAGE_ID is passed by the Makefile as 'A' or 'B'.
 * ---------------------------------------------------------------------- */
#ifndef IMAGE_ID
#define IMAGE_ID 'A'
#endif

#if IMAGE_ID == 'A'
#define IMAGE_NAME   "IMAGE A"
#define BLINK_PERIOD 1000u /* ms, a slow blink */
#else
#define IMAGE_NAME   "IMAGE B"
#define BLINK_PERIOD 200u /* ms, a fast blink */
#endif

/* -------------------------------------------------------------------------
 * Register definitions. Addresses and bit positions are from RM0492 and the
 * CMSIS device header for the STM32H503.
 * ---------------------------------------------------------------------- */
#define REG(addr) (*(volatile uint32_t *)(addr))

#define RCC_BASE    0x44020C00u
#define RCC_CR      REG(RCC_BASE + 0x00u)
#define RCC_AHB2ENR REG(RCC_BASE + 0x8Cu)
#define RCC_APB1LENR REG(RCC_BASE + 0x9Cu)

#define RCC_AHB2ENR_GPIOAEN   (1u << 0)
#define RCC_APB1LENR_USART3EN (1u << 18)
#define RCC_CR_HSIDIV_Pos     3u

#define GPIOA_BASE    0x42020000u
#define GPIOA_MODER   REG(GPIOA_BASE + 0x00u)
#define GPIOA_OSPEEDR REG(GPIOA_BASE + 0x08u)
#define GPIOA_BSRR    REG(GPIOA_BASE + 0x18u)
#define GPIOA_AFRL    REG(GPIOA_BASE + 0x20u)

#define USART3_BASE 0x40004800u
#define USART3_CR1  REG(USART3_BASE + 0x00u)
#define USART3_CR2  REG(USART3_BASE + 0x04u)
#define USART3_CR3  REG(USART3_BASE + 0x08u)
#define USART3_BRR  REG(USART3_BASE + 0x0Cu)
#define USART3_ISR  REG(USART3_BASE + 0x1Cu)
#define USART3_TDR  REG(USART3_BASE + 0x28u)

#define USART_CR1_UE   (1u << 0)
#define USART_CR1_TE   (1u << 3)
#define USART_ISR_TXE  (1u << 7)
#define USART_ISR_TC   (1u << 6)

/* SysTick lives in the Cortex-M33 core, not in the device peripherals. */
#define SYSTICK_CTRL REG(0xE000E010u)
#define SYSTICK_LOAD REG(0xE000E014u)
#define SYSTICK_VAL  REG(0xE000E018u)
#define SYSTICK_CTRL_ENABLE    (1u << 0)
#define SYSTICK_CTRL_CLKSOURCE (1u << 2)
#define SYSTICK_CTRL_COUNTFLAG (1u << 16)

#define HSI_HZ  64000000u
#define BAUD    115200u
#define LED_PIN 5u
#define TX_PIN  4u

/* -------------------------------------------------------------------------
 * Clocks
 *
 * The program does not reconfigure the clock tree. At reset the STM32H5 runs
 * from the HSI with the bus prescalers at 1, so the USART3 kernel clock is the
 * HSI divided by HSIDIV. Reading that divider instead of assuming it keeps the
 * baud rate correct whatever the reset default is, and avoids having to think
 * about flash wait states, which changing the clock would require.
 * ---------------------------------------------------------------------- */
static uint32_t pclk_hz(void)
{
    uint32_t hsidiv = (RCC_CR >> RCC_CR_HSIDIV_Pos) & 0x3u; /* 0..3 -> /1 /2 /4 /8 */
    return HSI_HZ >> hsidiv;
}

/* -------------------------------------------------------------------------
 * Timing
 * ---------------------------------------------------------------------- */
static void systick_init(uint32_t hz)
{
    SYSTICK_LOAD = (hz / 1000u) - 1u; /* wrap once per millisecond */
    SYSTICK_VAL = 0u;
    SYSTICK_CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_ENABLE;
}

/* Polls COUNTFLAG, which is set each time the counter wraps and cleared on
   read. No interrupts, so no vector table entries beyond the reset handler. */
static void delay_ms(uint32_t ms)
{
    while (ms--) {
        while ((SYSTICK_CTRL & SYSTICK_CTRL_COUNTFLAG) == 0u) {
            /* wait for the 1 ms wrap */
        }
    }
}

/* -------------------------------------------------------------------------
 * Serial output
 * ---------------------------------------------------------------------- */
static void uart_init(uint32_t hz)
{
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC_APB1LENR |= RCC_APB1LENR_USART3EN;
    /* A clock enable takes effect a cycle or two later. Read the registers back
       before touching the peripherals, which is the documented way to stall
       long enough. */
    (void)RCC_AHB2ENR;
    (void)RCC_APB1LENR;

    /* PA4 to alternate function 13, USART3_TX. Low output speed is ample for
       115200 and keeps the edges gentle. */
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (TX_PIN * 2u))) | (2u << (TX_PIN * 2u));
    GPIOA_OSPEEDR &= ~(3u << (TX_PIN * 2u));
    GPIOA_AFRL = (GPIOA_AFRL & ~(0xFu << (TX_PIN * 4u))) | (13u << (TX_PIN * 4u));

    /* Everything is set explicitly rather than left at its reset value, so the
       frame format is visible here: 8 data bits (M1:M0 = 0), no parity
       (PCE = 0), one stop bit (CR2 STOP = 0), no flow control. */
    USART3_CR1 = 0u;
    USART3_CR2 = 0u;
    USART3_CR3 = 0u;
    /* Oversampling by 16, so BRR is simply the rounded clock divisor. */
    USART3_BRR = (hz + BAUD / 2u) / BAUD;
    USART3_CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void uart_putc(char c)
{
    while ((USART3_ISR & USART_ISR_TXE) == 0u) {
    }
    USART3_TDR = (uint32_t)(uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') {
            uart_putc('\r'); /* terminals expect CRLF */
        }
        uart_putc(*s++);
    }
}

static void uart_putu(uint32_t v)
{
    char buf[11];
    int i = 0;
    if (v == 0u) {
        uart_putc('0');
        return;
    }
    while (v && i < (int)sizeof(buf)) {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (i--) {
        uart_putc(buf[i]);
    }
}

static void uart_flush(void)
{
    while ((USART3_ISR & USART_ISR_TC) == 0u) {
    }
}

/* -------------------------------------------------------------------------
 * LED
 * ---------------------------------------------------------------------- */
static void led_init(void)
{
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    (void)RCC_AHB2ENR;
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (LED_PIN * 2u))) | (1u << (LED_PIN * 2u));
}

static void led_set(int on)
{
    GPIOA_BSRR = on ? (1u << LED_PIN) : (1u << (LED_PIN + 16u));
}

/* -------------------------------------------------------------------------
 * Program
 * ---------------------------------------------------------------------- */
static void banner(uint32_t hz)
{
    uart_puts("\n");
    uart_puts("========================================\n");
    uart_puts("  Binho  |  AN0003\n");
    uart_puts("  Programming STM32 microcontrollers\n");
    uart_puts("  over SPI with the Binho Pulsar\n");
    uart_puts("========================================\n");
    uart_puts("  Running: " IMAGE_NAME "\n");
    uart_puts("  Kernel clock: ");
    uart_putu(hz / 1000000u);
    uart_puts(" MHz\n");
    /* Computed from the same constant that drives the LED, so the reported
       rate cannot drift away from the observed one. */
    uart_puts("  LED blink: ");
    uart_putu(1000u / BLINK_PERIOD);
    uart_puts(" Hz\n");
    uart_puts("========================================\n");
    uart_flush();
}

int main(void)
{
    uint32_t hz = pclk_hz();
    uint32_t seconds = 0u;
    uint32_t elapsed = 0u;
    int led = 0;

    led_init();
    uart_init(hz);
    systick_init(hz);

    banner(hz);

    for (;;) {
        led ^= 1;
        led_set(led);
        delay_ms(BLINK_PERIOD / 2u);

        /* The heartbeat is what a user sees when the terminal is opened after
           reset, so the running image is always identifiable. */
        elapsed += BLINK_PERIOD / 2u;
        if (elapsed >= 1000u) {
            elapsed -= 1000u;
            uart_puts("  " IMAGE_NAME "  alive, ");
            uart_putu(++seconds);
            uart_puts(" s\n");
            uart_flush();
        }
    }
}

/* -------------------------------------------------------------------------
 * Startup
 *
 * A Cortex-M33 needs only two vector table entries to boot: the initial stack
 * pointer and the reset handler. The remaining handlers are folded onto one
 * trap so that a fault stops visibly rather than running on.
 * ---------------------------------------------------------------------- */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

void Reset_Handler(void);

static void Default_Handler(void)
{
    for (;;) {
    }
}

__attribute__((section(".isr_vector"), used))
void (*const vector_table[])(void) = {
    (void (*)(void)) & _estack, /* initial stack pointer */
    Reset_Handler,
    Default_Handler, /* NMI */
    Default_Handler, /* HardFault */
    Default_Handler, /* MemManage */
    Default_Handler, /* BusFault */
    Default_Handler, /* UsageFault */
};

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata) {
        *dst++ = *src++;
    }
    for (dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0u;
    }

    (void)main();

    for (;;) {
    }
}
