#ifndef BOEINGMETROLOGYLIB_LensState_H
#define BOEINGMETROLOGYLIB_LensState_H

#include <string>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
	namespace Scanning
	{
		namespace Configuration
		{
			// Parameters that represent the lens state at a moment in time
			class BOEINGMETROLOGYLIB_API LensState : public Boeing::Interface::Serializer
			{
            private:
                // Device name
                std::string deviceName = "";

                // IP address
                std::string ipAddress = "";

                // Device's firmware version
                std::string softwareVersion = "";

                // Timestamp of state info
                std::string timestamp = "";

                // A unitless focus value
                int focus = 0;

                // Aperture value.  This is the denominator in the F-stop expression such as f/6.3 or f/11.  
                double aperture = 0.0;

                // The focal length in mm
                int zoom = 0;

                // Focus bounds (unitless)
                std::pair<int, int> focusRange = std::make_pair(0, 0);

                // Aperture bounds
                std::pair<double, double> apertureRange = std::make_pair(0.0, 0.0);

                // Zoom bounds in mm
                std::pair<int, int> zoomRange = std::make_pair(0, 0);

                void SerializeStream(std::ostream &strm) const;

                void DeserializeStream(std::istream &strm);

            public:
                // Constructors
                LensState();
                LensState(const std::string & deviceName, const std::string & ipAddress);

                bool IsValid() const;

                // Set value ranges
                void SetZoomRange(const std::pair<int, int> & zoomRange);
                void SetFocusRange(const std::pair<int, int> & focusRange);
                void SetApertureRange(const std::pair<double, double> & apertureRange);

                // Set the software version
                void SetSoftwareVersion(const std::string & softwareVersion);

                // Set aperture value and apply time stamp
                void SetApertureValue(const double & aperture);

                // Set focus value and apply time stamp
                void SetFocusValue(const int & focus);

                // Set zoom value and apply time stamp
                void SetZoomValue(const int & zoom);

                // Compare a value to a valid range
                bool IsInFocusRange(const int & focusValue) const;
                bool IsInApertureRange(const double & apertureValue) const;

                // Get the range of values
                std::pair<double, double> GetApertureRange() const;
                std::pair<int, int> GetFocusRange() const;

                // Get device name
                std::string GetDeviceName() const;

                // Get IP Address
                std::string GetIPAddress() const;

                // Get values
                int GetFocusValue() const;
                double GetApertureValue() const;
                int GetZoomValue() const;

                // Get the timestamp
                std::string GetTimeStamp() const;

				// Populate the output Json from this object's data.
				// Throws exception on failure.
                virtual void JsonSerialize(Json::Value &jsonNode) const override;

				// Populate this object from the input stream.
				// Throws exception on failure.
                virtual void JsonDeserialize(const Json::Value &jsonNode) override;

				// Write this class's contents to a json-formatted text file
                virtual void SerializeFile(const std::string &fileName) const override;

				// Read a json-formatted text file into this class
                virtual void DeserializeFile(const std::string &fileName) override;

                // Must be comparable and sortable.
                bool operator<(const LensState& rhs) const;
                bool operator==(const LensState& rhs) const;
                bool operator!=(const LensState& rhs) const;
			};
		}
	}
}

#endif
