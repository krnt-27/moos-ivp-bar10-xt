/************************************************************/
/*    NAME: Labs247                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Bar10XT.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <sstream>
#include <iomanip>
#include "MBUtils.h"
#include "ACTable.h"
#include "Bar10XT.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

Bar10XT::Bar10XT()
{
  // Default configuration
  m_i2c_bus       = "/dev/i2c-1";
  m_i2c_addr      = 0x40;
  m_fluid_density = 1029.0;  // seawater kg/m^3

  // State initialization
  m_sensor_initialized = false;
  m_pressure           = 0.0;
  m_temperature        = 0.0;
  m_depth              = 0.0;
  m_read_count         = 0;
  m_error_count        = 0;
}

//---------------------------------------------------------
// Destructor

Bar10XT::~Bar10XT()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Bar10XT::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Bar10XT::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Bar10XT::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_sensor_initialized) {
    if(m_sensor.read()) {
      m_pressure    = m_sensor.pressure();
      m_temperature = m_sensor.temperature();
      m_depth       = m_sensor.depth();
      m_read_count++;

      // Publish sensor data to MOOSDB
      Notify("BAR10XT_PRESSURE",    m_pressure);
      Notify("BAR10XT_TEMPERATURE", m_temperature);
      Notify("BAR10XT_DEPTH",       m_depth);
      Notify("BAR10XT_STATUS",      "OK");
    } else {
      m_error_count++;
      reportRunWarning("Sensor read failed: " + m_sensor.getErrorMsg());
      Notify("BAR10XT_STATUS", "ERROR");
    }
  } else {
    // Try to re-initialize sensor if not yet initialized
    if(m_sensor.init(m_i2c_bus, m_i2c_addr)) {
      m_sensor.setFluidDensity(m_fluid_density);
      m_sensor_initialized = true;
      retractRunWarning("Sensor not initialized");
      reportEvent("Bar10XT sensor initialized successfully");
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Bar10XT::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "i2c_bus") {
      m_i2c_bus = value;
      handled = true;
    }
    else if(param == "i2c_addr") {
      // Accept hex (0x40) or decimal (64)
      m_i2c_addr = (uint8_t)strtol(value.c_str(), NULL, 0);
      handled = true;
    }
    else if(param == "fluid_density") {
      m_fluid_density = atof(value.c_str());
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  // Initialize sensor
  reportEvent("Initializing Bar10XT on " + m_i2c_bus
             + " addr=0x" + to_string(m_i2c_addr));

  if(m_sensor.init(m_i2c_bus, m_i2c_addr)) {
    m_sensor.setFluidDensity(m_fluid_density);
    m_sensor_initialized = true;
    reportEvent("Bar10XT sensor initialized successfully");
    reportEvent("  Equipment: " + to_string(m_sensor.equipment));
    reportEvent("  P_min: " + to_string(m_sensor.P_min) + " bar");
    reportEvent("  P_max: " + to_string(m_sensor.P_max) + " bar");
    reportEvent("  Range: " + to_string(m_sensor.range()) + " bar");
    reportEvent("  Mode:  " + to_string(m_sensor.mode));

    string date_str = to_string(m_sensor.year + 2000) + "-"
                    + to_string(m_sensor.month) + "-"
                    + to_string(m_sensor.day);
    reportEvent("  Mfg Date: " + date_str);
  } else {
    m_sensor_initialized = false;
    reportRunWarning("Sensor init failed: " + m_sensor.getErrorMsg());
    reportRunWarning("Will retry initialization during Iterate()");
  }

  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Bar10XT::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // No subscriptions needed for a sensor driver
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Bar10XT::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "  iBar10XT Sensor Status                    " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << endl;

  // Configuration
  m_msgs << "Configuration:" << endl;
  m_msgs << "  I2C Bus:        " << m_i2c_bus << endl;

  std::ostringstream addr_ss;
  addr_ss << "0x" << std::hex << std::uppercase
          << (int)m_i2c_addr;
  m_msgs << "  I2C Address:    " << addr_ss.str() << endl;
  m_msgs << "  Fluid Density:  " << m_fluid_density << " kg/m^3" << endl;
  m_msgs << endl;

  // Sensor status
  if(!m_sensor_initialized) {
    m_msgs << "  *** SENSOR NOT INITIALIZED ***" << endl;
    m_msgs << "  Error: " << m_sensor.getErrorMsg() << endl;
    return(true);
  }

  // Sensor info
  ACTable info_tab(2);
  info_tab << "Parameter | Value";
  info_tab.addHeaderLines();
  info_tab << "Equipment"  << to_string(m_sensor.equipment);
  info_tab << "P_min"      << (to_string(m_sensor.P_min) + " bar");
  info_tab << "P_max"      << (to_string(m_sensor.P_max) + " bar");
  info_tab << "Range"      << (to_string(m_sensor.range()) + " bar");

  string mode_str;
  if(m_sensor.mode == 0) mode_str = "PR (Vented Gauge)";
  else if(m_sensor.mode == 1) mode_str = "PA (Sealed Gauge)";
  else mode_str = "PAA (Absolute)";
  info_tab << "Mode" << mode_str;

  string date_str = to_string(m_sensor.year + 2000) + "-"
                  + to_string(m_sensor.month) + "-"
                  + to_string(m_sensor.day);
  info_tab << "Mfg Date" << date_str;
  m_msgs << info_tab.getFormattedString() << endl;
  m_msgs << endl;

  // Live data
  m_msgs << "Live Readings:" << endl;
  ACTable data_tab(2);
  data_tab << "Measurement | Value";
  data_tab.addHeaderLines();

  std::ostringstream p_ss, t_ss, d_ss;
  p_ss << std::fixed << std::setprecision(2) << m_pressure;
  t_ss << std::fixed << std::setprecision(2) << m_temperature;
  d_ss << std::fixed << std::setprecision(3) << m_depth;

  data_tab << "Pressure"    << (p_ss.str() + " mbar");
  data_tab << "Temperature" << (t_ss.str() + " C");
  data_tab << "Depth"       << (d_ss.str() + " m");
  data_tab << "Connected"   << (m_sensor.isConnected() ? "YES" : "NO");
  m_msgs << data_tab.getFormattedString() << endl;
  m_msgs << endl;

  // Statistics
  m_msgs << "Statistics:" << endl;
  m_msgs << "  Reads:  " << m_read_count << endl;
  m_msgs << "  Errors: " << m_error_count << endl;

  return(true);
}
