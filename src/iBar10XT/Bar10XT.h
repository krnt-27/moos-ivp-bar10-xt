/************************************************************/
/*    NAME: Labs247                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Bar10XT.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Bar10XT_HEADER
#define Bar10XT_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "KellerLD.h"

class Bar10XT : public AppCastingMOOSApp
{
 public:
   Bar10XT();
   ~Bar10XT();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
   std::string m_i2c_bus;
   uint8_t     m_i2c_addr;
   double      m_fluid_density;

 private: // State variables
   KellerLD    m_sensor;
   bool        m_sensor_initialized;
   double      m_pressure;      // mbar
   double      m_temperature;   // deg C
   double      m_depth;         // meters
   unsigned int m_read_count;
   unsigned int m_error_count;
};

#endif 
