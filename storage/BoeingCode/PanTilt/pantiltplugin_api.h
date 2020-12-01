/**
 * \file
 */

#ifndef PANTILTPLUGIN_API_H
#define PANTILTPLUGIN_API_H

#include "Workspace/api_workspace.h"


#ifdef PANTILTPLUGIN_EXPORT
    #define PANTILTPLUGIN_API CSIRO_EXPORTSPEC
#else
    #define PANTILTPLUGIN_API CSIRO_IMPORTSPEC
#endif

#endif


