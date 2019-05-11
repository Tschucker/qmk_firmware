#include "config.h"
#include <stdbool.h>
#include "pincontrol.h"
#include "debug.h"
#define USE_LUFA_TWI

#ifdef USE_LUFA_TWI
#include "lib/lufa/LUFA/Drivers/Peripheral/TWI.h"
#include "lib/lufa/LUFA/Drivers/Peripheral/AVR8/TWI_AVR8.c"
#else
#include "i2cmaster.h"
#endif


// Controls the MCP23008 16 pin I/O expander
static bool initialized;
static uint8_t reinit_counter;

//#define i2cAddress 0x27 //2nd chip 0x20 // Configurable with jumpers
#define i2cTimeout 200 // milliseconds
enum mcp23008_registers {
	IODirectionA = 0x00,
  InputPolarityA = 0x01,
  InterruptOnChangeA = 0x02,
  DefaultValueA = 0x03,
  InterruptControlA = 0x04,
  IOConfigurationA = 0x05,
  PullUpA = 0x06,
  InterruptFlagA = 0x07,
  InterruptCaptureA = 0x08,
  IOPortA = 0x09,
  OutputLatchA = 0x0a,
};
#define MCP23008_INT_ERR 255

#ifdef USE_LUFA_TWI
static const char *twi_err_str(uint8_t res) {
  switch (res) {
    case TWI_ERROR_NoError: return "OK";
    case TWI_ERROR_BusFault: return "BUSFAULT";
    case TWI_ERROR_BusCaptureTimeout: return "BUSTIMEOUT";
    case TWI_ERROR_SlaveResponseTimeout: return "SLAVETIMEOUT";
    case TWI_ERROR_SlaveNotReady: return "SLAVENOTREADY";
    case TWI_ERROR_SlaveNAK: return "SLAVENAK";
    default: return "UNKNOWN";
  }
}
#endif

static inline bool _set_register(unsigned char i2cAddress, enum mcp23008_registers reg, unsigned char val) {
#ifdef USE_LUFA_TWI
  uint8_t addr = reg;
  uint8_t result = TWI_WritePacket(i2cAddress << 1, i2cTimeout, &addr, sizeof(addr),
                                   &val, sizeof(val));
  if (result) {
    xprintf("mcp: set_register %d = %d failed: %s\n", reg, val, twi_err_str(result));
  }
  return result == 0;
#else
  bool success = false;
  if (i2c_start_write(i2cAddress)) {
    xprintf("mcp: start_write failed\n");
    goto done;
  }

  if (i2c_write((unsigned char)reg)) {
    xprintf("mcp: write reg addr %d failed\n", reg);
    goto done;
  }

  success = i2c_write(val) == 0;
  if (!success) {
    xprintf("mcp: write reg addr %d val = %d failed\n", reg, val);
  }
done:
  i2c_stop();
  return success;
#endif
}
#define set_reg(i2cAddress, reg, val) if (!_set_register(i2cAddress, reg, val)) { goto done; }

bool mcp23008_enable_interrupts(unsigned char i2cAddress) {
  bool success = false;
  // Configure interrupt pins to mirror each other and OR
  // the interrupts from both ports.
  set_reg(i2cAddress, IOConfigurationA, 0b00000000);

  // We want interrupts to fire when the buttons toggle
  set_reg(i2cAddress, InterruptControlA, 0xff);

  // And enable interrupts
  set_reg(i2cAddress, InterruptOnChangeA, 0xfe); // Note: A0 is floating

  success = true;
done:
  return success;
}

bool mcp23008_init(unsigned char i2cAddress) {
  initialized = false;

  // Set all the pins as inputs
  set_reg(i2cAddress, IODirectionA, 0xff);

  // Read key presses (logic low) as 0s
  set_reg(i2cAddress, InputPolarityA, 0x00);

  // Turn on internal pull-ups; we're adding our own
  set_reg(i2cAddress, PullUpA, 0xff);

  // Disable interrupts
  set_reg(i2cAddress, InterruptOnChangeA, 0x0);

  initialized = true;
done:
  if (!initialized) {
    dprint("failed to init mcp\n");
  } else {
    dprint("mcp initialized!\n");
  }
  return initialized;
}

bool mcp23008_make_ready(unsigned char i2cAddress) {
  if (initialized) {
    return true;
  }
  // This will roll over 1 every 255 matrix scans
  if (reinit_counter++ != 0) {
    return false;
  }
  return mcp23008_init(i2cAddress);
}

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
// Read all 8 inputs and return them
uint8_t mcp23008_read(unsigned char i2cAddress) {
  uint8_t pins = 0;

  if (!initialized) {
    return 0;
  }

#ifdef USE_LUFA_TWI
  uint8_t addr = IOPortA;
  uint8_t buf[1];
  uint8_t result = TWI_ReadPacket(i2cAddress << 1, i2cTimeout, &addr,
                                  sizeof(addr), buf, sizeof(buf));
  if (result) {
    xprintf("mcp: read pins failed: %s\n", twi_err_str(result));
    initialized = false;
    return 0;
  }
  pins = reverse(buf[0]);
  return pins; //need to concatinate and not in the matrix code
#else

  if (i2c_start_write(i2cAddress)) {
    initialized = false;
    goto done;
  }

  if (i2c_write(IOPortA)) {
    initialized = false;
    goto done;
  }

  if (i2c_start_read(i2cAddress)) {
    initialized = false;
    goto done;
  }

  // Read PortA
  pins = i2c_readNck();

done:
  i2c_stop();

  if (!initialized) {
    dprint("failed to read mcp, will re-init\n");
    return 0;
  }
#endif

  return reverse(pins);
}
