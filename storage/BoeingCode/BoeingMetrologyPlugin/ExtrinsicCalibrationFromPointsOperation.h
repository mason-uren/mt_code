/*============================================================================

Copyright 2016 by:

The Boeing Company

============================================================================*/

#ifndef BoeingMetrology_ExtrinsicCalibrationFromPointsOperation_H
#define BoeingMetrology_ExtrinsicCalibrationFromPointsOperation_H
#include <cassert>
#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "boeingmetrologyplugin.h"


namespace BoeingMetrology
{
	class ExtrinsicCalibrationFromPointsOperationImpl;

	class BOEINGMETROLOGYPLUGIN_API ExtrinsicCalibrationFromPointsOperation : public CSIRO::DataExecution::Operation
	{
		// Allow string translation to work properly
		Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::ExtrinsicCalibrationFromPointsOperation)

			ExtrinsicCalibrationFromPointsOperationImpl*  pImpl_;

		// Prevent copy and assignment - these should not be implemented
		ExtrinsicCalibrationFromPointsOperation(const ExtrinsicCalibrationFromPointsOperation&) = delete;
		ExtrinsicCalibrationFromPointsOperation& operator=(const ExtrinsicCalibrationFromPointsOperation&) = delete;

	protected:
		virtual bool  execute();

	public:
		ExtrinsicCalibrationFromPointsOperation();
		virtual ~ExtrinsicCalibrationFromPointsOperation();
	};
}

	DECLARE_WORKSPACE_OPERATION_FACTORY(BoeingMetrology::ExtrinsicCalibrationFromPointsOperation, BOEINGMETROLOGYPLUGIN_API)

#endif

