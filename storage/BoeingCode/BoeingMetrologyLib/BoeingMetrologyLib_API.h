/*=========================================================================

  Revision:       $Revision: 4 $
  Last changed:   $Date: 2016-01-11 17:07:31 -0800 (Mon, 11 Jan 2016) $

  Copyright 2016 by:

    Commonwealth Scientific and Industrial Research Organisation (CSIRO)

    Computational Software Engineering and Visualisation Team
    For further information, contact: workspace@csiro.au

  This copyright notice must be included with all copies of the source code.

===========================================================================*/

/**
 * \file
 */

#ifndef BOEINGMETROLOGYLIB_API_H
#define BOEINGMETROLOGYLIB_API_H

/**
 * This block defines the appropriate import / export specifications for
 * the relevant platforms.
 */
#if defined WIN32

    #define BOEINGMETROLOGYLIB_EXPORTSPEC __declspec(dllexport)
    #define BOEINGMETROLOGYLIB_LOCALSPEC
    #define BOEINGMETROLOGYLIB_IMPORTSPEC __declspec(dllimport)

#elif defined __GNUC__

    #if __GNUC__ >= 4
        #define BOEINGMETROLOGYLIB_EXPORTSPEC __attribute__ ((visibility("default")))
        #define BOEINGMETROLOGYLIB_LOCALSPEC  __attribute__ ((visibility("hidden")))
        #define BOEINGMETROLOGYLIB_IMPORTSPEC __attribute__ ((visibility("default")))
    #else
        #define BOEINGMETROLOGYLIB_EXPORTSPEC
        #define BOEINGMETROLOGYLIB_LOCALSPEC
        #define BOEINGMETROLOGYLIB_IMPORTSPEC
    #endif

#else

    #define BOEINGMETROLOGYLIB_EXPORTSPEC
    #define BOEINGMETROLOGYLIB_LOCALSPEC
    #define BOEINGMETROLOGYLIB_IMPORTSPEC

#endif

#ifdef BOEINGMETROLOGYLIB_EXPORT
#define BOEINGMETROLOGYLIB_API BOEINGMETROLOGYLIB_EXPORTSPEC
#else
#define BOEINGMETROLOGYLIB_API BOEINGMETROLOGYLIB_IMPORTSPEC
#endif

#endif


