#ifndef BOEINGMETROLOGYLIB_UNITS_H
#define BOEINGMETROLOGYLIB_UNITS_H

#include "BoeingMetrologyLib_API.h"

namespace BoeingMetrology
{
    // A class to encapsulate distance units and conversion factors
    class BOEINGMETROLOGYLIB_API Units
    {
    public:
        // Enumerate unit types 
        // 0 = MM
        // 1 = Meters
        // 2 = inches
        enum UNITS_ENUM { MM = 0, M = 1, INCHES = 2 };

    private:
        // Curent units
        UNITS_ENUM current = MM;

        // Conversion factors
        const double mmToInches = 0.03937007874015748031496062992126;
        const double inchesToMm = 25.4;

    public:
        // Default units are mm
        Units();

        // Constructor initialize the units
        Units(const UNITS_ENUM & currentIn);

        // Copy constructor
        Units(const Units & src);

        // Assignment operator
        Units operator=(const Units & src)
        {
            this->SetCurrentUnits(src.GetCurrentUnits());
            return *this;
        }

        // Get the current unit type
        UNITS_ENUM GetCurrentUnits() const;

        // Set the current unit type
        void SetCurrentUnits(const UNITS_ENUM & newUnits);

        // Get the input value in the specified units
        double GetValue(const double & value, const UNITS_ENUM & desiredUnits);

        // Get the input value in mm
        double GetMM(const double & value);

        // Get the input value in meters
        double GetM(const double & value);

        // Get the input value in inches
        double GetInches(const double & value);
    };
}

#endif
