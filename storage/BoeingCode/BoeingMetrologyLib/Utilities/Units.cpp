#include "Units.h"
#include <stdexcept>

BoeingMetrology::Units::Units(const UNITS_ENUM & currentIn)
{
    this->SetCurrentUnits(currentIn);
}

BoeingMetrology::Units::Units(const Units & src)
{
    this->SetCurrentUnits(src.GetCurrentUnits());
}

BoeingMetrology::Units::Units()
{
    this->SetCurrentUnits(UNITS_ENUM::MM);
}

BoeingMetrology::Units::UNITS_ENUM BoeingMetrology::Units::GetCurrentUnits() const
{
    return this->current;
}

void BoeingMetrology::Units::SetCurrentUnits(const UNITS_ENUM & newUnits)
{
    this->current = newUnits;
}

double BoeingMetrology::Units::GetValue(const double & value, const UNITS_ENUM & desiredUnits)
{
    if (desiredUnits == MM)
        return this->GetMM(value);
    else if (desiredUnits == M)
        return this->GetM(value);
    else if (desiredUnits == INCHES)
        return this->GetInches(value);
    else 
        throw std::runtime_error("BoeingMetrology::Units: Unrecognized units type");
}

double BoeingMetrology::Units::GetMM(const double & value)
{
    if (current == MM)
        return value;
    else if (current == M)
        return (1000.0 * value);
    else if (current == INCHES)
        return (inchesToMm * value);
    else 
        throw std::runtime_error("BoeingMetrology::Units: Unrecognized units type");
}

double BoeingMetrology::Units::GetM(const double & value)
{
    if (current == MM)
        return (0.001 * value);
    else if (current == M)
        return (value);
    else if (current == INCHES)
        return (inchesToMm * 0.001 * value);
    else 
        throw std::runtime_error("BoeingMetrology::Units: Unrecognized units type");
}

double BoeingMetrology::Units::GetInches(const double & value)
{
    if (current == MM)
        return (mmToInches * value);
    else if (current == M)
        return (mmToInches * 1000.0 * value);
    else if (current == INCHES)
        return (value);
    else 
        throw std::runtime_error("BoeingMetrology::Units: Unrecognized units type");
}
