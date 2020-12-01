/*============================================================================

  Copyright 2016 by:

    The Boeing Company

============================================================================*/

#ifndef BoeingMetrology_ChArucoBoardMultiDetector_H
#define BoeingMetrology_ChArucoBoardMultiDetector_H

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

	class ChArucoBoardMultiDetectorImpl;

	class BOEINGMETROLOGYPLUGIN_API ChArucoBoardMultiDetector : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
		Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::ChArucoBoardMultiDetector)

			ChArucoBoardMultiDetectorImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
		ChArucoBoardMultiDetector(const ChArucoBoardMultiDetector&) = delete;
		ChArucoBoardMultiDetector& operator=(const ChArucoBoardMultiDetector&) = delete;

    protected:
        virtual bool  execute() override;

    public:
		ChArucoBoardMultiDetector();
		virtual ~ChArucoBoardMultiDetector();
    };
}

DECLARE_WORKSPACE_OPERATION_FACTORY(BoeingMetrology::ChArucoBoardMultiDetector, BOEINGMETROLOGYPLUGIN_API)

#endif

