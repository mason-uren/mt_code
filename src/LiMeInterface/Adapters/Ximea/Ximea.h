#pragma once
#include <string>
#include "xiApi.h"
using namespace std;

namespace Ximea
{
    const unsigned int MAX_IMAGE_HEIGHT = 6004;
    const unsigned int MAX_IMAGE_WIDTH = 7920;

    enum ImageSizeEnumType
    {
        LARGE_IMAGE,
        MEDIUM_IMAGE,
        SMALL_IMAGE
    };

    // Aperture Values
    enum LensApertureValueEnumType
    {
        LENS_APERTURE_VALUE_2_8    , //  2.8
        LENS_APERTURE_VALUE_4_0    , //  4
        LENS_APERTURE_VALUE_5_6    , //  5.6
        LENS_APERTURE_VALUE_8_0    , //  8
        LENS_APERTURE_VALUE_11_0   , //  11
        NUM_LENS_APERTURE_VALUE,
    };

    const double LENS_APERTURE_LOOKUP_TABLE [NUM_LENS_APERTURE_VALUE] = { 2.8, 4.0, 5.6, 8.0, 11.0 };

}