/**
 * \file
 */

#ifndef BOEINGMETROLOGY_EXECUTECOMMAND_H
#define BOEINGMETROLOGY_EXECUTECOMMAND_H

#include "Workspace/DataExecution/Operations/operation.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "pantiltplugin.h"


namespace BoeingMetrology
{
    class ExecuteCommandImpl;

    /**
     * \brief Put a one-line description of your operation here
     *
     * Add a more detailed description of your operation here
     * or remove these lines if the brief description above
     * is sufficient.
     */
    class PANTILTPLUGIN_API ExecuteCommand : public CSIRO::DataExecution::Operation
    {
        // Allow string translation to work properly
        Q_DECLARE_TR_FUNCTIONS(BoeingMetrology::ExecuteCommand)

        ExecuteCommandImpl*  pImpl_;

        // Prevent copy and assignment - these should not be implemented
        ExecuteCommand(const ExecuteCommand&);
        ExecuteCommand& operator=(const ExecuteCommand&);

    protected:
        virtual bool  execute();

    public:
        ExecuteCommand();
        virtual ~ExecuteCommand();
    };
}

DECLARE_WORKSPACE_OPERATION_FACTORY(BoeingMetrology::ExecuteCommand, PANTILTPLUGIN_API)

#endif

