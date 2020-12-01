/**
 * \file
 */

#ifndef BOEINGMETROLOGY_PANTILTPLUGIN_H
#define BOEINGMETROLOGY_PANTILTPLUGIN_H

#include "Workspace/Application/workspaceplugin.h"

#include "pantiltplugin_api.h"


namespace BoeingMetrology
{
    class PanTiltPluginImpl;

    /**
     * \brief Put a one-line description of your plugin here
     *
     * Add a more detailed description of your plugin here
     * or remove these lines if the brief description above
     * is sufficient.
     */
    class PANTILTPLUGIN_API PanTiltPlugin : public CSIRO::Application::WorkspacePlugin
    {
        PanTiltPluginImpl*  pImpl_;

        PanTiltPlugin();
        ~PanTiltPlugin();

        // Prevent copying and assignment
        PanTiltPlugin(const PanTiltPlugin&);
        PanTiltPlugin& operator=(const PanTiltPlugin&);

    protected:
        virtual const CSIRO::DataExecution::DataFactory*       getAliasedDataFactory(const QString& dataType) const;
        virtual const CSIRO::DataExecution::OperationFactory*  getAliasedOperationFactory(const QString& opType) const;

    public:
        static PanTiltPlugin&  getInstance();
        
        QStringList  getCustomWidgetPaths() const;

        virtual bool  setup();
    };
}

#endif
