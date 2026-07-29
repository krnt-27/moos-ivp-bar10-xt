/************************************************************/
/*    NAME: Labs247                                              */
/*    ORGN: MIT                                             */
/*    FILE: Bar10XTnoAppCasting.h                                          */
/*    DATE:                                                 */
/************************************************************/

#ifndef Bar10XTnoAppCasting_HEADER
#define Bar10XTnoAppCasting_HEADER

#include "MOOS/libMOOS/MOOSLib.h"

class Bar10XTnoAppCasting : public CMOOSApp
{
 public:
   Bar10XTnoAppCasting();
   ~Bar10XTnoAppCasting();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected:
   void RegisterVariables();

 private: // Configuration variables

 private: // State variables
};

#endif 
