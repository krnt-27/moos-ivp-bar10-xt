/************************************************************/
/*    NAME: labs247                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SerialWitMotion.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SerialWitMotion.h"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <cmath>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

//---------------------------------------------------------
// Constructor()

SerialWitMotion::SerialWitMotion()
{
  m_port_name = "/dev/ttyUSB0";
  m_baudrate = 9600;
  m_serial_fd = -1;
  m_poll_counter = 0;
  m_last_reconnect_time = 0;
  m_rx_buffer_len = 0;
  
  m_heading_correction = 20.0;
  m_calibration_counter = 0;
  m_calibrated = false;
  m_acc_correction[0] = 0.0;
  m_acc_correction[1] = 0.0;
  m_acc_correction[2] = 0.0;
  m_yaw_static = 0.0;
  m_yaw_mean_idx = 0;
  m_yaw_mean_buffer[0] = 0.0;
  m_yaw_mean_buffer[1] = 0.0;
  m_yaw_mean_buffer[2] = 0.0;
  
  m_madgwick = MadgwickAHRS(10.0f, 0.1f); // 10Hz sample freq
}

//---------------------------------------------------------
// Destructor

SerialWitMotion::~SerialWitMotion()
{
  if (m_serial_fd >= 0) {
    close(m_serial_fd);
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool SerialWitMotion::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

     if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool SerialWitMotion::OnConnectToServer()
{
   registerVariables();
   if (m_serial_fd < 0) {
     openSerialPort();
   }
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool SerialWitMotion::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  if (m_serial_fd < 0) {
    if (MOOSTime() - m_last_reconnect_time > 1.0) {
      m_last_reconnect_time = MOOSTime();
      if (openSerialPort()) {
        reportEvent("Successfully reconnected to " + m_port_name);
        retractRunWarning("Serial port read error. Disconnected.");
        retractRunWarning("Serial port returned EOF. Disconnected.");
        retractRunWarning("Serial port write error. Disconnected.");
        retractRunWarning("Unable to open port: " + m_port_name);
      }
    }
  }

  if (m_serial_fd >= 0) {
    // Send polling request periodically
    m_poll_counter++;
    if (m_poll_counter >= 1) { // Poll at AppTick frequency
      HWT9053.data.validMain = false;
      HWT9053.data.validQuat = false;
      HWT9053.data.validTemp = false;
      pollSensor();
      m_poll_counter = 0;
    }
    
    // We already called readSerialPort inside syncReadModbus, but call it again just in case
    readSerialPort();
    
    // Process data if valid main block received
    if (HWT9053.data.validMain) {
        processCalibration();
        
        // Correct acceleration based on calibration
        double ax = HWT9053.data.accelX + m_acc_correction[0];
        double ay = HWT9053.data.accelY + m_acc_correction[1];
        double az = HWT9053.data.accelZ + m_acc_correction[2];
        
        double gx = HWT9053.data.gyroX;
        double gy = HWT9053.data.gyroY;
        double gz = HWT9053.data.gyroZ;
        
        double mx = HWT9053.data.magX;
        double my = HWT9053.data.magY;
        double mz = HWT9053.data.magZ;
        
        // Run Madgwick AHRS filter
        m_madgwick.updateMARG(gx, gy, gz, ax, ay, az, mx, my, mz);
        
        double roll_madgwick = m_madgwick.getRoll();
        double pitch_madgwick = m_madgwick.getPitch();
        double yaw_madgwick = m_madgwick.getYaw() + m_heading_correction;
        m_last_yaw_raw = yaw_madgwick;
        
        // Kalman Filtering
        double filtered_ax = m_kalman.acc_x.update(ax);
        double filtered_ay = m_kalman.acc_y.update(ay);
        double filtered_az = m_kalman.acc_z.update(az);
        double filtered_gx = m_kalman.gyro_x.update(gx);
        double filtered_gy = m_kalman.gyro_y.update(gy);
        double filtered_gz = m_kalman.gyro_z.update(gz);
        double filtered_mx = m_kalman.mag_x.update(mx);
        double filtered_my = m_kalman.mag_y.update(my);
        double filtered_mz = m_kalman.mag_z.update(mz);
        
        double filtered_roll = m_kalman.filterAngle(roll_madgwick, m_kalman.roll_sin, m_kalman.roll_cos);
        double filtered_pitch = m_kalman.filterAngle(pitch_madgwick, m_kalman.pitch_sin, m_kalman.pitch_cos);
        double filtered_yaw = m_kalman.filterAngle(yaw_madgwick, m_kalman.yaw_sin, m_kalman.yaw_cos);
        
        // Update yaw mean buffer
        m_yaw_mean_buffer[m_yaw_mean_idx] = filtered_yaw;
        m_yaw_mean_idx = (m_yaw_mean_idx + 1) % 3;
        double yaw_mean = (m_yaw_mean_buffer[0] + m_yaw_mean_buffer[1] + m_yaw_mean_buffer[2]) / 3.0;

        // Populate final values
        HWT9053.data.accelX = filtered_ax;
        HWT9053.data.accelY = filtered_ay;
        HWT9053.data.accelZ = filtered_az;
        HWT9053.data.gyroX = filtered_gx;
        HWT9053.data.gyroY = filtered_gy;
        HWT9053.data.gyroZ = filtered_gz;
        HWT9053.data.magX = filtered_mx;
        HWT9053.data.magY = filtered_my;
        HWT9053.data.magZ = filtered_mz;
        
        // Publish
        publishSensorData();
        
        // Clear flags for next iteration
        HWT9053.data.validMain = false;
    }

  } else {
    reportRunWarning("Serial port not open.");
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
void SerialWitMotion::processCalibration()
{
    if (m_calibration_counter < 265) {
        // Collect samples for offset calculation
        m_calib_ax.push_back(HWT9053.data.accelX - 1.0);
        m_calib_ay.push_back(HWT9053.data.accelY - 1.0);
        m_calib_az.push_back(HWT9053.data.accelZ - 1.0);
        m_calib_yaw.push_back(HWT9053.data.yaw);
        m_calibrated = false;
        m_calibration_counter++;
        
        if (m_calibration_counter == 265) {
            // Compute mean offsets
            double sum_ax = std::accumulate(m_calib_ax.begin(), m_calib_ax.end(), 0.0);
            double sum_ay = std::accumulate(m_calib_ay.begin(), m_calib_ay.end(), 0.0);
            double sum_az = std::accumulate(m_calib_az.begin(), m_calib_az.end(), 0.0);
            double sum_yaw = std::accumulate(m_calib_yaw.begin(), m_calib_yaw.end(), 0.0);
            
            m_acc_correction[0] = -1.0 * ((sum_ax / 265.0) + 1.0);
            m_acc_correction[1] = -1.0 * ((sum_ay / 265.0) + 1.0);
            m_acc_correction[2] = -1.0 * (sum_az / 265.0);
            
            m_yaw_static = (sum_yaw / 265.0) + m_heading_correction;
            
            m_calib_ax.clear();
            m_calib_ay.clear();
            m_calib_az.clear();
            m_calib_yaw.clear();
            m_calibrated = true;
        }
    }
}

//---------------------------------------------------------
void SerialWitMotion::publishSensorData()
{
    double yaw_mean = (m_yaw_mean_buffer[0] + m_yaw_mean_buffer[1] + m_yaw_mean_buffer[2]) / 3.0;
    string status = m_calibrated ? "Calibrated" : "Calibrating";

    // Get Madgwick filtered values (consistent with Python pipeline_imu)
    double roll_madgwick = m_madgwick.getRoll();
    double pitch_madgwick = m_madgwick.getPitch();
    double yaw_madgwick = m_madgwick.getYaw() + m_heading_correction;

    // Publish individual variables (using Madgwick filtered values like Python)
    Notify("INS_ROLL", roll_madgwick);
    Notify("INS_PITCH", pitch_madgwick);
    Notify("INS_YAW", yaw_madgwick);
    
    Notify("INS_ACCEL_X", HWT9053.data.accelX);
    Notify("INS_ACCEL_Y", HWT9053.data.accelY);
    Notify("INS_ACCEL_Z", HWT9053.data.accelZ);
    
    Notify("INS_GYRO_X", HWT9053.data.gyroX);
    Notify("INS_GYRO_Y", HWT9053.data.gyroY);
    Notify("INS_GYRO_Z", HWT9053.data.gyroZ);
    
    if (HWT9053.data.validQuat) {
        Notify("INS_QUAT_W", HWT9053.data.q0);
        Notify("INS_QUAT_X", HWT9053.data.q1);
        Notify("INS_QUAT_Y", HWT9053.data.q2);
        Notify("INS_QUAT_Z", HWT9053.data.q3);
    }
    
    if (HWT9053.data.validTemp) {
        Notify("INS_TEMPERATURE", HWT9053.data.temperature);
        Notify("INS_RAW_TEMPERATURE", HWT9053.data.raw_temperature);
    }
    
    Notify("INS_STATUS", status);

    // Publish JSON string using nlohmann/json (consistent with Python pipeline_imu format)
    json data_json;
    data_json["accelerationX"] = HWT9053.data.accelX;
    data_json["accelerationY"] = HWT9053.data.accelY;
    data_json["accelerationZ"] = HWT9053.data.accelZ;
    data_json["angular_velocity_x"] = HWT9053.data.gyroX;
    data_json["angular_velocity_y"] = HWT9053.data.gyroY;
    data_json["angular_velocity_z"] = HWT9053.data.gyroZ;
    data_json["magnetic_field_x_uT"] = HWT9053.data.magX;
    data_json["magnetic_field_y_uT"] = HWT9053.data.magY;
    data_json["magnetic_field_z_uT"] = HWT9053.data.magZ;
    data_json["roll"] = HWT9053.data.roll;
    data_json["pitch"] = HWT9053.data.pitch;
    data_json["yaw"] = HWT9053.data.yaw;
    data_json["roll_madgwick"] = roll_madgwick;
    data_json["pitch_madgwick"] = pitch_madgwick;
    data_json["yaw_madgwick"] = yaw_madgwick;
    data_json["yaw_madgwick_raw"] = yaw_madgwick;
    data_json["yaw_madgwick_mean"] = yaw_mean;
    
    if (HWT9053.data.validQuat) {
        data_json["quaternion_q0"] = HWT9053.data.q0;
        data_json["quaternion_q1"] = HWT9053.data.q1;
        data_json["quaternion_q2"] = HWT9053.data.q2;
        data_json["quaternion_q3"] = HWT9053.data.q3;
    }
    
    if (HWT9053.data.validTemp) {
        data_json["temperature"] = HWT9053.data.temperature;
        data_json["raw_temperature"] = HWT9053.data.raw_temperature;
    }
    
    data_json["status"] = status;
    data_json["yaw_static"] = m_yaw_static;
    
    // Dump JSON as single line (no formatting) for uMS compatibility
    Notify("INS_DATA", data_json.dump(-1));
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool SerialWitMotion::OnStartUp()
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
    string param = toupper(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "PORT") {
      m_port_name = value;
      handled = true;
    }
    else if(param == "BAUDRATE") {
      m_baudrate = atoi(value.c_str());
      handled = true;
    }
    else if(param == "HEADING_CORRECTION") {
      m_heading_correction = atof(value.c_str());
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();	
  
  if (m_serial_fd < 0) {
    openSerialPort();
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void SerialWitMotion::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
}


//------------------------------------------------------------
// Procedure: buildReport()

bool SerialWitMotion::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "WitMotion Sensor Status                     " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(2);
  actab << "Config | Value";
  actab.addHeaderLines();
  actab << "Port" << m_port_name;
  actab << "Baudrate" << intToString(m_baudrate);
  actab << "Heading Corr" << doubleToStringX(m_heading_correction, 2);
  actab << "Serial Open" << (m_serial_fd >= 0 ? "Yes" : "No");
  actab << "Status" << (m_calibrated ? "Calibrated" : "Calibrating (" + intToString(m_calibration_counter) + "/265)");
  m_msgs << actab.getFormattedString();
  m_msgs << endl;

  ACTable actab2(4);
  actab2 << "Axis | Roll/X | Pitch/Y | Yaw/Z";
  actab2.addHeaderLines();
  actab2 << "Angle (Madgwick)" << doubleToStringX(m_madgwick.getRoll(), 2) << doubleToStringX(m_madgwick.getPitch(), 2) << doubleToStringX(m_madgwick.getYaw() + m_heading_correction, 2);
  actab2 << "Accel (g)" << doubleToStringX(HWT9053.data.accelX, 3) << doubleToStringX(HWT9053.data.accelY, 3) << doubleToStringX(HWT9053.data.accelZ, 3);
  actab2 << "Gyro (deg/s)" << doubleToStringX(HWT9053.data.gyroX, 2) << doubleToStringX(HWT9053.data.gyroY, 2) << doubleToStringX(HWT9053.data.gyroZ, 2);
  m_msgs << actab2.getFormattedString();

  return(true);
}

//---------------------------------------------------------
// Procedure: openSerialPort()

bool SerialWitMotion::openSerialPort()
{
  m_serial_fd = open(m_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (m_serial_fd == -1) {
    reportRunWarning("Unable to open port: " + m_port_name);
    return false;
  }
  retractRunWarning("Unable to open port: " + m_port_name);
  
  fcntl(m_serial_fd, F_SETFL, FNDELAY); // Non-blocking read

  struct termios options;
  tcgetattr(m_serial_fd, &options);

  speed_t baud;
  switch (m_baudrate) {
    case 9600: baud = B9600; break;
    case 115200: baud = B115200; break;
    default: baud = B9600; break;
  }
  
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  options.c_cflag |= (CLOCAL | CREAD); // Enable receiver and set local mode
  options.c_cflag &= ~PARENB; // No parity
  options.c_cflag &= ~CSTOPB; // 1 stop bit
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8; // 8 data bits

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
  options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
  options.c_oflag &= ~OPOST; // Raw output

  tcsetattr(m_serial_fd, TCSANOW, &options);
  
  return true;
}

//---------------------------------------------------------
// Procedure: readSerialPort()

void SerialWitMotion::readSerialPort()
{
  if (m_serial_fd < 0) return;
  
  unsigned char rx_buffer[256];
  int rx_length = read(m_serial_fd, (void*)rx_buffer, 255);
  
  if (rx_length > 0) {
    for (int i = 0; i < rx_length; i++) {
      m_rx_buffer[m_rx_buffer_len++] = rx_buffer[i];
      if (m_rx_buffer_len >= 1024) m_rx_buffer_len = 0; // Prevent overflow
    }
    
    // Parse Modbus frames
    int pos = 0;
    while (pos < m_rx_buffer_len) {
        if (m_rx_buffer[pos] == 0x50 && pos + 2 < m_rx_buffer_len) {
            int frameLen = 5 + m_rx_buffer[pos + 2]; // 3 header bytes + count + 2 CRC bytes
            if (pos + frameLen <= m_rx_buffer_len) {
                HWT9053.parseModbusResponse(m_rx_buffer + pos, frameLen);
                pos += frameLen;
            } else {
                break; // Incomplete frame
            }
        } else {
            pos++; // Shift to find start byte
        }
    }
    
    // Move remaining bytes to start
    if (pos > 0) {
        int remaining = m_rx_buffer_len - pos;
        if (remaining > 0) {
            memmove(m_rx_buffer, m_rx_buffer + pos, remaining);
        }
        m_rx_buffer_len = remaining;
    }
  } else if (rx_length < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      reportRunWarning("Serial port read error. Disconnected.");
      close(m_serial_fd);
      m_serial_fd = -1;
    }
  } else if (rx_length == 0) {
    reportRunWarning("Serial port returned EOF. Disconnected.");
    close(m_serial_fd);
    m_serial_fd = -1;
  }
}

//---------------------------------------------------------
// Procedure: syncReadModbus()

bool SerialWitMotion::syncReadModbus(unsigned char addr, unsigned char count)
{
    unsigned char cmd[8];
    HWT9053.buildReadCommand(addr, count, cmd);
    write(m_serial_fd, cmd, 8);
    
    // Wait for response up to 50ms
    double start_time = MOOSTime();
    while (MOOSTime() - start_time < 0.05) {
        readSerialPort();
        
        if (addr == 0x34 && HWT9053.data.validMain) return true;
        if (addr == 0x51 && HWT9053.data.validQuat) return true;
        if (addr == 0x43 && HWT9053.data.validTemp) return true;
        
        usleep(2000); // Sleep 2ms
    }
    return false;
}

//---------------------------------------------------------
// Procedure: pollSensor()

void SerialWitMotion::pollSensor()
{
  if (m_serial_fd < 0) return;
  
  // Poll Main Block
  syncReadModbus(0x34, 15);
  
  // Poll Quaternion
  syncReadModbus(0x51, 4);
  
  // Poll Temperature
  syncReadModbus(0x43, 1);
}
