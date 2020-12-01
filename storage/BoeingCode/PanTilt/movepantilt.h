/**
 * \file
 */

#ifndef BOEINGMETROLOGY_MOVEPANTILT_H
#define BOEINGMETROLOGY_MOVEPANTILT_H

#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "pantiltplugin.h"


namespace BoeingMetrology
{
    class MovePanTiltImpl;

    /**
     * \brief Put a one-line description of your operation here
     *
     * Add a more detailed description of your operation here
     * or remove these lines if the brief description above
     * is sufficient.
     */
    class PANTILTPLUGIN_API MovePanTilt : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::MovePanTilt)

        MovePanTiltImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
        MovePanTilt(const MovePanTilt&);
        MovePanTilt& operator=(const MovePanTilt&);

    protected:
        virtual bool  execute();

    public:
        MovePanTilt();
        virtual ~MovePanTilt();
    };
}

DECLARE_WORKSPACE_OPERATION_FACTORY(BoeingMetrology::MovePanTilt, PANTILTPLUGIN_API)

#endif

