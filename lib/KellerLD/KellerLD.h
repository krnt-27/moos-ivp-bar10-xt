/************************************************************/
/*    NAME: Labs247                                         */
/*    ORGN: MIT                                             */
/*    FILE: KellerLD.h                                      */
/*    DATE: July 2026                                       */
/*    INFO: Linux I2C driver for Keller 4LD-9LD series      */
/*          pressure/temperature sensors (Bar10XT)          */
/*    REF:  BlueRobotics_KellerLD_Library (Arduino)         */
/************************************************************/

#ifndef KELLERLD_H
#define KELLERLD_H

#include <cstdint>
#include <string>

// Keller LD I2C Defines
#define KELLERLD_DEFAULT_ADDR   0x40
#define KELLERLD_DEFAULT_BUS    "/dev/i2c-1"

// Command and register addresses
#define KELLERLD_REQUEST        0xAC
#define KELLERLD_CUST_ID0       0x00
#define KELLERLD_CUST_ID1       0x01
#define KELLERLD_SCALING0       0x12
#define KELLERLD_SCALING1       0x13
#define KELLERLD_SCALING2       0x14
#define KELLERLD_SCALING3       0x15
#define KELLERLD_SCALING4       0x16

class KellerLD
{
public:
  // Unit conversion factors (from mbar)
  static constexpr float Pa   = 100.0f;
  static constexpr float bar  = 0.001f;
  static constexpr float mbar = 1.0f;

  KellerLD();
  ~KellerLD();

  // -------------------------------------------------------
  // Initialization and configuration
  // -------------------------------------------------------

  /** Opens the I2C bus and reads calibration data from sensor.
   *  @param bus    I2C bus device path, e.g. "/dev/i2c-1"
   *  @param addr   7-bit I2C address, default 0x40
   *  @return true if sensor was detected and initialized
   */
  bool init(const std::string &bus = KELLERLD_DEFAULT_BUS,
            uint8_t addr = KELLERLD_DEFAULT_ADDR);

  /** Set the fluid density in kg/m^3 for depth calculation.
   *  Default is 1029 (seawater). Use 997 for freshwater.
   */
  void setFluidDensity(float density);

  // -------------------------------------------------------
  // Reading data
  // -------------------------------------------------------

  /** Reads a pressure/temperature measurement from the sensor.
   *  Takes ~10ms due to sensor conversion time.
   *  @return true if read succeeded and data is valid
   */
  bool read();

  // -------------------------------------------------------
  // Getters
  // -------------------------------------------------------

  /** Returns true if sensor is initialized and detected. */
  bool isInitialized() const;

  /** Returns true if last read() was successful. */
  bool isConnected() const;

  /** Pressure in mbar (or mbar * conversion factor). */
  float pressure(float conversion = 1.0f) const;

  /** Temperature in degrees Celsius. */
  float temperature() const;

  /** Depth in meters. Valid for incompressible liquids only. */
  float depth() const;

  /** Altitude in meters. Valid for air only. */
  float altitude() const;

  /** Pressure range of sensor in bar. */
  float range() const;

  /** Get error message from last failed operation. */
  std::string getErrorMsg() const;

  // -------------------------------------------------------
  // Sensor metadata (populated after init())
  // -------------------------------------------------------
  uint16_t equipment;
  uint16_t place;
  uint16_t file;

  uint8_t  mode;
  uint16_t year;
  uint8_t  month;
  uint8_t  day;

  uint32_t code;

  uint16_t P;         // Raw pressure value
  float    P_bar;     // Pressure in bar
  float    P_mode;    // Pressure mode offset
  float    P_min;     // Minimum pressure (bar)
  float    P_max;     // Maximum pressure (bar)

private:
  /** Read a 16-bit value from the sensor memory map. */
  uint16_t readMemoryMap(uint8_t mtp_address);

  /** Write a single byte to the I2C device. */
  bool i2cWrite(uint8_t byte);

  /** Read n bytes from the I2C device into buf. */
  bool i2cRead(uint8_t *buf, int len);

  int         m_fd;             // I2C file descriptor
  uint8_t     m_addr;           // I2C slave address
  std::string m_bus;            // I2C bus device path

  float       m_fluid_density;  // kg/m^3
  float       m_T_degc;         // Last temperature reading (C)

  uint16_t    m_cust_id0;
  uint16_t    m_cust_id1;

  bool        m_initialized;
  bool        m_connected;
  std::string m_error_msg;
};

#endif
