#ifndef BOEINGMETROLOGYLIB_LensControls_H
#define BOEINGMETROLOGYLIB_LensControls_H

#include <string>
#include "BoeingMetrologyLib_API.h"
#include "Scanning/Configuration/LensState.h"


namespace BoeingMetrology
{
	namespace DeviceIO
	{
		namespace FocusController
		{
			namespace LC2
			{
				// Controls interface for the ISSI LC-2 lens controller device
				class BOEINGMETROLOGYLIB_API LensControls
				{
				private:
                    static const int RESP_MAX;

					//LC2 API Commands
					const char* VERSION = "ver";					//firmware version
					const char* PING = "ping";						//returns focus, aperture, and zoom, ranges, current positions, AF and IS flags
					const char* DETECT_FOCUS_RANGE = "refRange";	 
					const char* SET_FOCUS = "setFocus2=";			//"setFocus2" restarts motors before setting focus = less drift, "setFocus" = more drift
					const char* MOVE_FOCUS = "moveFocus=";
					const char* STOP_FOCUS = "stopFocus";			//cancels unreachable focus value
					const char* SET_APER = "setAper=";
					const char* MOVE_APER = "moveAper=";
					const char* SOFT_RESTART = "Reboot";			//expect answer
					const char* RESTART = "Restart";				//expect answer "OK"

					//UDP Defaults by Controller OEM
					std::string CONTROLLER_IP = "";	
					unsigned short CONTROLLER_PORT = 1339;		

                    //Command sending and receiving
                    bool checkResponse(const std::string stringValue, const std::string controlType);

                    bool keyCheck(const std::string keyStr);
                    void GetCurrentParams(const std::string stringValue);
                    std::pair<int, int> GetIntRange(const std::string stringValue);
                    std::pair<double, double> GetDoubleRange(const std::string stringValue);
                    bool GetAutomaticFocus(std::string respStr);
                    bool GetIS(std::string respStr);
                    bool GetISactive(const std::string & respStr);

                    bool focusSet = false;
                    bool automaticFocus = false;
                    bool IS = false;
                    bool IsActive = false;                    

                    // Timeout in milliseconds on send and recv commands
                    int timeoutms = 5000;

                public:
                    bool connected = false;

                    // The current state of the lens
                    Scanning::Configuration::LensState lensState;

                    LensControls() {};

                    // Constructor
                    LensControls(const std::string & deviceName, const std::string & ipAddress, const int & timeoutMilliseconds = 5000);					
					
					//PING server and get ranges
					void Connect();

                    // Set the aperture
					void SetAperture(const double targetAperture, double &actualAperture, const int & sleepTimeMs = 0);

                    // Set the focus
                    void SetFocus(const int targetFocus, int &actualFocus, const int & sleepTimeMs = 0);

                    void SetFocusIncrement(const int focusDelta, int &actualFocus);

                    bool isFocusSet(const std::string responseStr);

                    int getFocusMin();
                    int getFocusMax();
				};
			}
		}
	}
}
#endif
