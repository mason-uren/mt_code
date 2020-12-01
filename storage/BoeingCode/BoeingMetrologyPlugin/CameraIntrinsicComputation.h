/*============================================================================

  Copyright 2016 by:

    The Boeing Company

============================================================================*/


#ifndef BoeingMetrology_CameraIntrinsicComputation_H
#define BoeingMetrology_CameraIntrinsicComputation_H

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
	class CameraIntrinsicComputationImpl;


	class BOEINGMETROLOGYPLUGIN_API CameraIntrinsicComputation : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
		Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::CameraIntrinsicComputation)

			CameraIntrinsicComputationImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
		CameraIntrinsicComputation(const CameraIntrinsicComputation&) = delete;
		CameraIntrinsicComputation& operator=(const CameraIntrinsicComputation&) = delete;

    protected:
        virtual bool  execute() override;

    public:
		CameraIntrinsicComputation();
		virtual ~CameraIntrinsicComputation();
    };
}

DECLARE_WORKSPACE_OPERATION_FACTORY(BoeingMetrology::CameraIntrinsicComputation, BOEINGMETROLOGYPLUGIN_API)

#endif

