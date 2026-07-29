/****************************************************************/
/*   NAME: Labs247                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: Bar10XT_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "Bar10XT_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iBar10XT application interfaces with the Blue Robotics    ");
  blk("  Bar10XT pressure/temperature sensor (Keller 4LD series)       ");
  blk("  via I2C. It publishes pressure (mbar), temperature (C),       ");
  blk("  and depth (m) to the MOOSDB.                                  ");
  blk("                                                                ");
  blk("  Default I2C address: 0x40, bus: /dev/i2c-1                    ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iBar10XT file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iBar10XT with the given process name         ");
  blk("      rather than iBar10XT.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iBar10XT.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iBar10XT Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iBar10XT                              ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  // I2C Configuration                                          ");
  blk("  I2C_BUS   = /dev/i2c-1                                       ");
  blk("  I2C_ADDR  = 0x40                                              ");
  blk("                                                                ");
  blk("  // Fluid density in kg/m^3                                    ");
  blk("  // 1029 = seawater (default), 997 = freshwater                ");
  blk("  FLUID_DENSITY = 1029                                          ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iBar10XT INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  None                                                          ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  BAR10XT_PRESSURE    = double (mbar)                           ");
  blk("  BAR10XT_TEMPERATURE = double (deg C)                          ");
  blk("  BAR10XT_DEPTH       = double (meters)                         ");
  blk("  BAR10XT_STATUS      = string (OK / ERROR)                     ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iBar10XT", "gpl");
  exit(0);
}
