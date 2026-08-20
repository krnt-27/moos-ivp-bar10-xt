/************************************************************/
/*    NAME: labs247                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SerialWitMotion.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SerialWitMotion_HEADER
#define SerialWitMotion_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <string>
#include <termios.h>
#include <vector>
#include "MadgwickAHRS.h"
#include "SimpleKalman.h"
#include "HWT9053.h"

class SerialWitMotion : public AppCastingMOOSApp
{
 public:
   SerialWitMotion();
   ~SerialWitMotion();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Serial methods
   bool openSerialPort();
   void readSerialPort();
   void pollSensor();
   bool syncReadModbus(unsigned char addr, unsigned char count);

 private: // Processing methods
   void processCalibration();
   void publishSensorData();

 private: // Configuration variables
   std::string m_port_name;
   int m_baudrate;
   double m_heading_correction;

 private: // State variables
   int m_serial_fd;
   int m_poll_counter;
   double m_last_reconnect_time;
   int m_poll_state; // 0=Main, 1=Quat, 2=Temp, 3=Wait

   unsigned char m_rx_buffer[1024];
   int m_rx_buffer_len;

   MadgwickAHRS m_madgwick;
   SensorKalmanFilter m_kalman;
   
   int m_calibration_counter;
   double m_acc_correction[3];
   double m_yaw_static;
   std::vector<double> m_calib_ax, m_calib_ay, m_calib_az, m_calib_yaw;
   
   double m_yaw_mean_buffer[3];
   int m_yaw_mean_idx;
   bool m_calibrated;
   
   // Latest processed values
   double m_last_yaw_raw;
};

#endif
