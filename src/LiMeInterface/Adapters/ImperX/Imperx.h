#pragma once
#include <string>
using namespace std;

namespace ImperX
{

    enum ExposureModeEnumType
    {
        EXPOSURE_OFF,
        TRIGGER_WIDTH,
        TIMED,
        NUM_EXPOSURE_MODES
    };

    const string exposureModeStrings[NUM_EXPOSURE_MODES] = { "Off", "TriggerWidth", "Timed" };

    enum ImageSizeEnumType
    {
        LARGE_IMAGE,
        MEDIUM_IMAGE,
        SMALL_IMAGE
    };

    enum PixelFormatEnumType
    {
        MONO8,
        MONO10,
        NUM_PIXEL_FORMATS
    };

    const string pixelFormatStrings[NUM_PIXEL_FORMATS] = { "Mono8", "Mono10" };

    enum AnalogGainEnumType
    {
        GAIN_1_0X,
        GAIN_1_26X,
        GAIN_1_87X,
        GAIN_3_17X,
        NUM_ANALOG_GAINS
    };

    const string analogGainStrings[NUM_ANALOG_GAINS] = { "Gain_1x0", "Gain_1x26", "Gain_1x87", "Gain_3x17" };

    enum BlackLevelAutoEnumType
    {
        BLACK_LEVEL_AUTO_OFF,
        BLACK_LEVEL_AUTO_ON_CONTINUOUS,
        NUM_BLACK_LEVEL_AUTO_SETTINGS
    };

    const string blackLevelAutoStrings[NUM_BLACK_LEVEL_AUTO_SETTINGS] = { "Off", "On" };

    enum TriggerModeEnumType
    {
        TRIGGER_OFF,
        TRIGGER_ON,
        NUM_TRIGGER_MODES
    };

    const string triggerModeStrings[NUM_TRIGGER_MODES] = { "Off", "On" };

    enum TriggerSourceEnumType
    {
        IN1,
        IN2,
        PULSE_GENERATOR,
        SOFTWARE,
        NUM_TRIGGER_SOURCES
    };

    const string triggerSourceStrings[NUM_TRIGGER_SOURCES] = { "IN1", "IN2", "PulseGenerator", "Software" };

    enum TriggerActivationEnumType
    {
        RISING_EDGE,
        FALLING_EDGE,
        NUM_TRIGGER_ACTIVATIONS
    };

    const string triggerActivationStrings[NUM_TRIGGER_ACTIVATIONS] = { "RisingEdge", "FallingEdge" };

    enum TriggerDebounceEnumType
    {
        DEBOUNCE_DISABLED,
        MICROSECONDS_10,
        MICROSECONDS_50,
        MICROSECONDS_100,
        MICROSECONDS_500,
        MILLISECONDS_1,
        MILLISECONDS_5,
        MILLISECONDS_10,
        NUM_TRIGGER_DEBOUNCES
    };

    const string triggerDebounceStrings[NUM_TRIGGER_DEBOUNCES] = { "Disabled", "TenMicroSeconds", "FiftyMicroSeconds", "OneHundredMicroSeconds",
                                                                   "FiveHundredMicroSeconds", "OneMilliSecond", "FiveMilliSeconds", "TenMilliSeconds" };
}