/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Raspberry Pi & Pi 2 memory-mapped GPIO-based programmer.
 * 
 * Copyright (C) 2015 Tony DiCola <tony@tonydicola.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "ac_cfg.h"

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "avrdude.h"
#include "avr.h"
#include "pindefs.h"
#include "pgm.h"
#include "bitbang.h"


// Raspberry Pi & Pi 2 memory-mapped GPIO helpers for fast access to GPIO.
// Adapted from code by Gert van Loo & Dom: 
//   http://elinux.org/RPi_Low-level_peripherals#GPIO_Code_examples
// And code from the RPi.GPIO library:
//   http://sourceforge.net/p/raspberry-gpio-python/
#define MMIO_SUCCESS 0
#define MMIO_ERROR_DEVMEM -1
#define MMIO_ERROR_MMAP -2
#define MMIO_ERROR_OFFSET -3
#define GPIO_BASE_OFFSET 0x200000
#define GPIO_LENGTH 4096

volatile uint32_t* pi_mmio_gpio = NULL;

static inline void pi_mmio_set_input(const unsigned int gpio_number) {
  // Set GPIO register to 000 for specified GPIO number.
  *(pi_mmio_gpio+((gpio_number)/10)) &= ~(7<<(((gpio_number)%10)*3));
}

static inline void pi_mmio_set_output(const unsigned int gpio_number) {
  // First set to 000 using input function.
  pi_mmio_set_input(gpio_number);
  // Next set bit 0 to 1 to set output.
  *(pi_mmio_gpio+((gpio_number)/10)) |=  (1<<(((gpio_number)%10)*3));
}

static inline void pi_mmio_set_high(const unsigned int gpio_number) {
  *(pi_mmio_gpio+7) = 1 << gpio_number;
}

static inline void pi_mmio_set_low(const unsigned int gpio_number) {
  *(pi_mmio_gpio+10) = 1 << gpio_number;
}

static inline bool pi_mmio_input(const unsigned int gpio_number) {
  return *(pi_mmio_gpio+13) & (1 << gpio_number);
}

static int pi_mmio_init(void) {
  if (pi_mmio_gpio == NULL) {
    // Check for GPIO and peripheral addresses from device tree.
    // Adapted from code in the RPi.GPIO library at:
    //   http://sourceforge.net/p/raspberry-gpio-python/
    FILE *fp = fopen("/proc/device-tree/soc/ranges", "rb");
    if (fp == NULL) {
      return MMIO_ERROR_OFFSET;
    }
    fseek(fp, 4, SEEK_SET);
    unsigned char buf[4];
    if (fread(buf, 1, sizeof(buf), fp) != sizeof(buf)) {
      return MMIO_ERROR_OFFSET;
    }
    uint32_t peri_base = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0;
    uint32_t gpio_base = peri_base + GPIO_BASE_OFFSET;
    fclose(fp);

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
      // Error opening /dev/mem.  Probably not running as root.
      return MMIO_ERROR_DEVMEM;
    }
    // Map GPIO memory to location in process space.
    pi_mmio_gpio = (uint32_t*)mmap(NULL, GPIO_LENGTH, PROT_READ | PROT_WRITE, MAP_SHARED, fd, gpio_base);
    close(fd);
    if (pi_mmio_gpio == MAP_FAILED) {
      // Don't save the result if the memory mapping failed.
      pi_mmio_gpio = NULL;
      return MMIO_ERROR_MMAP;
    }
  }
  return MMIO_SUCCESS;
}

// End Pi memory-mapped GPIO helpers.

#define N_GPIO (PIN_MAX + 1)

static int pigpio_setpin(PROGRAMMER * pgm, int pinfunc, int value)
{
  int pin = pgm->pinno[pinfunc];

  if (pin & PIN_INVERSE)
  {
    value  = !value;
    pin   &= PIN_MASK;
  }

  if (value)
    pi_mmio_set_high(pin);
  else
    pi_mmio_set_low(pin);

  if (pgm->ispdelay > 1)
    bitbang_delay(pgm->ispdelay);

  return 0;
}

static int pigpio_getpin(PROGRAMMER * pgm, int pinfunc)
{
  unsigned char invert=0;
  int pin = pgm->pinno[pinfunc];

  if (pin & PIN_INVERSE)
  {
    invert = 1;
    pin   &= PIN_MASK;
  }

  if (pi_mmio_input(pin))
    return 1-invert;
  else
    return 0+invert;
}

static int pigpio_highpulsepin(PROGRAMMER * pgm, int pinfunc)
{
  pigpio_setpin(pgm, pinfunc, 1);
  pigpio_setpin(pgm, pinfunc, 0);

  return 0;
}

static void pigpio_display(PROGRAMMER *pgm, const char *p)
{
    fprintf(stderr, "%sPin assignment  : gpio{n}\n",p);
    pgm_display_generic_mask(pgm, p, SHOW_AVR_PINS);
}

static void pigpio_enable(PROGRAMMER *pgm)
{
  // No implementation.
}

static void pigpio_disable(PROGRAMMER *pgm)
{
  // No implementation.
}

static void pigpio_powerup(PROGRAMMER *pgm)
{
  // No implementation.
}

static void pigpio_powerdown(PROGRAMMER *pgm)
{
  // No implementation.
}

static int pigpio_open(PROGRAMMER *pgm, char *port)
{
  int i, pin, r;

  bitbang_check_prerequisites(pgm);

  r = pi_mmio_init();
  if (r == MMIO_ERROR_DEVMEM) {
    fprintf(stderr, "ERROR: Couldn't access memory for GPIO, make sure to run as root!\n");
    return -1;
  }
  else if (r != MMIO_SUCCESS) {
    fprintf(stderr, "ERROR: Failed to initialize initializing memory mapped GPIO: %d\n", r);
    return -1;
  }

  // Pin 0 behavior is the same as the linuxgpio driver (see comment in its
  // open function.
  for (i=0; i<N_PINS; i++) {
    if ( (pgm->pinno[i] & PIN_MASK) != 0 ||
         i == PIN_AVR_RESET ||
         i == PIN_AVR_SCK   ||
         i == PIN_AVR_MOSI  ||
         i == PIN_AVR_MISO ) {
      pin = pgm->pinno[i] & PIN_MASK;
      if (i == PIN_AVR_MISO)
        pi_mmio_set_input(pin);
      else
        pi_mmio_set_output(pin);
    }
  }

  return 0;
}

static void pigpio_close(PROGRAMMER *pgm)
{
  int reset = pgm->pinno[PIN_AVR_RESET] & PIN_MASK;
  int sck   = pgm->pinno[PIN_AVR_SCK]   & PIN_MASK;
  int mosi  = pgm->pinno[PIN_AVR_MOSI]  & PIN_MASK;
  int miso  = pgm->pinno[PIN_AVR_MISO]  & PIN_MASK;

  // Configure all pins as inputs.  Make sure to configure reset pin last to
  // prevent conflicts when AVR firmware starts (noted from linuxgpio programmer).
  pi_mmio_set_input(sck);
  pi_mmio_set_input(mosi);
  pi_mmio_set_input(miso);
  pi_mmio_set_input(reset);
}

void pigpio_initpgm(PROGRAMMER *pgm)
{
  strcpy(pgm->type, "pigpio");

  pgm_fill_old_pins(pgm);

  pgm->rdy_led        = bitbang_rdy_led;
  pgm->err_led        = bitbang_err_led;
  pgm->pgm_led        = bitbang_pgm_led;
  pgm->vfy_led        = bitbang_vfy_led;
  pgm->initialize     = bitbang_initialize;
  pgm->display        = pigpio_display;
  pgm->enable         = pigpio_enable;
  pgm->disable        = pigpio_disable;
  pgm->powerup        = pigpio_powerup;
  pgm->powerdown      = pigpio_powerdown;
  pgm->program_enable = bitbang_program_enable;
  pgm->chip_erase     = bitbang_chip_erase;
  pgm->cmd            = bitbang_cmd;
  pgm->open           = pigpio_open;
  pgm->close          = pigpio_close;
  pgm->setpin         = pigpio_setpin;
  pgm->getpin         = pigpio_getpin;
  pgm->highpulsepin   = pigpio_highpulsepin;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;
}

const char pigpio_desc[] = "Raspberry Pi & Pi 2 memory-mapped GPIO bitbang programmer.";
