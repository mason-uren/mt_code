/**
 * \file
 */

#ifndef BOEINGMETROLOGY_GETANGLES_H
#define BOEINGMETROLOGY_GETANGLES_H

#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "pantiltplugin.h"


namespace BoeingMetrology
{
    class GetAnglesImpl;

    /**
     * \brief Put a one-line description of your operation here
     *
     * Add a more detailed description of your operation here
     * or remove these lines if the brief description above
     * is sufficient.
     */
    class PANTILTPLUGIN_API GetAngles : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::GetAngles)

        GetAnglesImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
        GetAngles(const GetAngles&);
        GetAngles& operator=(const GetAngles&);

    protected:
        virtual bool  execute();

    public:
        GetAngles();
        virtual ~GetAngles();
    };
}

DECLARE_WORKSPACE_OPERATION_FACTORY(BoeingMetrology::GetAngles, PANTILTPLUGIN_API)

#endif

