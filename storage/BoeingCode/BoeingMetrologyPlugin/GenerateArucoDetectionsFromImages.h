/*============================================================================

  Copyright 2016 by:

    The Boeing Company

============================================================================*/


#ifndef BoeingMetrology_GenerateArucoDetectionsFromImages_H
#define BoeingMetrology_GenerateArucoDetectionsFromImages_H

#include <cassert>

#include "Workspace/DataExecution/DataObjects/typedobject.h"
#include "Workspace/DataExecution/DataObjects/datafactorytraits.h"
#include "Workspace/DataExecution/DataObjects/typeddatafactory.h"
#include "Workspace/DataExecution/Operations/typedoperationfactory.h"
#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "boeingmetrologyplugin.h"

namespace BoeingMetrology
{
	class GenerateArucoDetectionsFromImagesImpl;

	class BOEINGMETROLOGYPLUGIN_API GenerateArucoDetectionsFromImages : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
		Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::GenerateArucoDetectionsFromImages)

			GenerateArucoDetectionsFromImagesImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
		GenerateArucoDetectionsFromImages(const GenerateArucoDetectionsFromImages&) = delete;
		GenerateArucoDetectionsFromImages& operator=(const GenerateArucoDetectionsFromImages&) = delete;

    protected:
        virtual bool  execute() override;

    public:
		GenerateArucoDetectionsFromImages();
		virtual ~GenerateArucoDetectionsFromImages();
    };
}

DECLARE_WORKSPACE_OPERATION_FACTORY(BoeingMetrology::GenerateArucoDetectionsFromImages, BOEINGMETROLOGYPLUGIN_API)

#endif

