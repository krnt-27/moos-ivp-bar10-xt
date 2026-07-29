/************************************************************/
/*    NAME: Labs247                                         */
/*    ORGN: MIT                                             */
/*    FILE: KellerLD.cpp                                    */
/*    DATE: July 2026                                       */
/*    INFO: Linux I2C driver for Keller 4LD-9LD series      */
/*          pressure/temperature sensors (Bar10XT)          */
/*    REF:  BlueRobotics_KellerLD_Library (Arduino)         */
/************************************************************/

#include "KellerLD.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <cmath>
#include <iostream>

using namespace std;

//---------------------------------------------------------
// Constructor

KellerLD::KellerLD()
{
  m_fd            = -1;
  m_addr          = KELLERLD_DEFAULT_ADDR;
  m_bus           = KELLERLD_DEFAULT_BUS;
  m_fluid_density = 1029.0f;  // seawater default
  m_T_degc        = 0.0f;
  m_cust_id0      = 0;
  m_cust_id1      = 0;
  m_initialized   = false;
  m_connected     = false;
  m_error_msg     = "";

  equipment = 0;
  place     = 0;
  file      = 0;
  mode      = 0;
  year      = 0;
  month     = 0;
  day       = 0;
  code      = 0;
  P         = 0;
  P_bar     = 0.0f;
  P_mode    = 0.0f;
  P_min     = 0.0f;
  P_max     = 0.0f;
}

//---------------------------------------------------------
// Destructor

KellerLD::~KellerLD()
{
  if(m_fd >= 0) {
    close(m_fd);
    m_fd = -1;
  }
}

//---------------------------------------------------------
// Procedure: i2cWrite()

bool KellerLD::i2cWrite(uint8_t byte)
{
  if(m_fd < 0)
    return false;

  int ret = ::write(m_fd, &byte, 1);
  if(ret != 1) {
    m_error_msg = "I2C write failed";
    return false;
  }
  return true;
}

//---------------------------------------------------------
// Procedure: i2cRead()

bool KellerLD::i2cRead(uint8_t *buf, int len)
{
  if(m_fd < 0)
    return false;

  int ret = ::read(m_fd, buf, len);
  if(ret != len) {
    m_error_msg = "I2C read failed: expected " + to_string(len)
                + " bytes, got " + to_string(ret);
    return false;
  }
  return true;
}

//---------------------------------------------------------
// Procedure: readMemoryMap()
//   Read a 16-bit value from the sensor's memory map register.
//   Protocol: write register address, wait 1ms, read 3 bytes
//   (1 status + 2 data bytes).

uint16_t KellerLD::readMemoryMap(uint8_t mtp_address)
{
  if(!i2cWrite(mtp_address))
    return 0;

  usleep(1000);  // 1ms delay for response

  uint8_t buf[3];
  if(!i2cRead(buf, 3))
    return 0;

  // buf[0] = status byte (ignored here)
  // buf[1] = MSB, buf[2] = LSB
  return (uint16_t(buf[1]) << 8) | buf[2];
}

//---------------------------------------------------------
// Procedure: init()

bool KellerLD::init(const string &bus, uint8_t addr)
{
  m_bus  = bus;
  m_addr = addr;
  m_initialized = false;
  m_connected   = false;

  // Open I2C bus
  m_fd = open(m_bus.c_str(), O_RDWR);
  if(m_fd < 0) {
    m_error_msg = "Failed to open I2C bus: " + m_bus;
    return false;
  }

  // Set I2C slave address
  if(ioctl(m_fd, I2C_SLAVE, m_addr) < 0) {
    m_error_msg = "Failed to set I2C address: 0x"
                + to_string(m_addr);
    close(m_fd);
    m_fd = -1;
    return false;
  }

  // Read customer ID registers
  m_cust_id0 = readMemoryMap(KELLERLD_CUST_ID0);
  m_cust_id1 = readMemoryMap(KELLERLD_CUST_ID1);

  code      = (uint32_t(m_cust_id1) << 16) | m_cust_id0;
  equipment = m_cust_id0 >> 10;
  place     = m_cust_id0 & 0x01FF;  // lower 9 bits
  file      = m_cust_id1;

  // Check if sensor is connected (equipment code 63 = not connected)
  if(equipment >= 63) {
    m_error_msg = "No sensor detected (equipment code = "
                + to_string(equipment) + ")";
    close(m_fd);
    m_fd = -1;
    return false;
  }

  // Read scaling register 0 (date and mode info)
  uint16_t scaling0 = readMemoryMap(KELLERLD_SCALING0);

  mode  = scaling0 & 0x03;
  year  = scaling0 >> 11;
  month = (scaling0 & 0x0780) >> 7;
  day   = (scaling0 & 0x007C) >> 2;

  // Handle P-mode pressure offset (to vacuum pressure)
  if(mode == 0) {
    // PR mode, Vented Gauge. Zero when front pressure == rear pressure
    P_mode = 1.01325f;
  } else if(mode == 1) {
    // PA mode, Sealed Gauge. Zero at 1.0 bar
    P_mode = 1.0f;
  } else {
    // PAA mode, Absolute. Zero at vacuum
    P_mode = 0.0f;
  }

  // Read P_min (scaling registers 1 and 2 form an IEEE 754 float)
  uint32_t scaling12 = (uint32_t(readMemoryMap(KELLERLD_SCALING1)) << 16)
                      | readMemoryMap(KELLERLD_SCALING2);
  memcpy(&P_min, &scaling12, sizeof(float));

  // Read P_max (scaling registers 3 and 4 form an IEEE 754 float)
  uint32_t scaling34 = (uint32_t(readMemoryMap(KELLERLD_SCALING3)) << 16)
                      | readMemoryMap(KELLERLD_SCALING4);
  memcpy(&P_max, &scaling34, sizeof(float));

  m_initialized = true;
  m_connected   = true;
  m_error_msg   = "";

  return true;
}

//---------------------------------------------------------
// Procedure: setFluidDensity()

void KellerLD::setFluidDensity(float density)
{
  m_fluid_density = density;
}

//---------------------------------------------------------
// Procedure: read()
//   Request measurement: write 0xAC, wait 9ms, read 5 bytes
//   Byte 0: status
//   Bytes 1-2: raw pressure (16-bit unsigned)
//   Bytes 3-4: raw temperature (16-bit unsigned)

bool KellerLD::read()
{
  if(m_fd < 0 || !m_initialized) {
    m_error_msg = "Sensor not initialized";
    m_connected = false;
    return false;
  }

  // Send measurement request
  if(!i2cWrite(KELLERLD_REQUEST)) {
    m_connected = false;
    return false;
  }

  // Wait for conversion (max 9ms per datasheet)
  usleep(9000);

  // Read 5 bytes: status + pressure(2) + temperature(2)
  uint8_t buf[5];
  if(!i2cRead(buf, 5)) {
    m_connected = false;
    return false;
  }

  uint8_t status = buf[0];

  // Validate status byte (0b01BMoEXX):
  //   bit 7    reserved, must be 0
  //   bit 6    reserved, must be 1
  //   bits 4-3 operating mode, must be 00 (normal measurement)
  //   bit 2    memory/EEPROM checksum error, must be 0
  // BUSY (bit 5) and don't-care bits (1-0) are masked out
  if((status & 0xDC) != 0x40) {
    m_error_msg = "Invalid status byte: 0x"
                + to_string(status);
    m_connected = false;
    return false;
  }

  uint16_t P_raw = (uint16_t(buf[1]) << 8) | buf[2];
  uint16_t T_raw = (uint16_t(buf[3]) << 8) | buf[4];

  P = P_raw;
  P_bar = (float(P) - 16384.0f) * (P_max - P_min) / 32768.0f
        + P_min + P_mode;
  m_T_degc = ((T_raw >> 4) - 24) * 0.05f - 50.0f;

  m_connected = true;
  m_error_msg = "";
  return true;
}

//---------------------------------------------------------
// Getters

bool KellerLD::isInitialized() const
{
  return m_initialized;
}

bool KellerLD::isConnected() const
{
  return m_connected;
}

float KellerLD::pressure(float conversion) const
{
  return P_bar * 1000.0f * conversion;
}

float KellerLD::temperature() const
{
  return m_T_degc;
}

float KellerLD::depth() const
{
  // depth = (P_pascals - atmospheric) / (rho * g)
  return (pressure(KellerLD::Pa) - 101325.0f)
       / (m_fluid_density * 9.80665f);
}

float KellerLD::altitude() const
{
  return (1.0f - pow(pressure() / 1013.25f, 0.190284f))
       * 145366.45f * 0.3048f;
}

float KellerLD::range() const
{
  return P_max - P_min;
}

std::string KellerLD::getErrorMsg() const
{
  return m_error_msg;
}
