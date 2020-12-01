#include <QString>
#include <QStringList>

#include "Workspace/DataExecution/DataObjects/datafactorytraits.h"
#include "Workspace/DataExecution/Operations/operationfactorytraits.h"

#include "pantiltplugin.h"
#include "executecommand.h"
#include "getangles.h"
#include "movepantilt.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

namespace BoeingMetrology
{
    /**
     * \internal
     */
    class PanTiltPluginImpl
    {
    public:
        // You can add, remove or modify anything in here without
        // breaking binary compatibility. It starts as empty, but
        // leave it here in case at some time in the future you
        // want to add some data for your plugin without breaking
        // binary compatibility.
    };


    /**
     *
     */
    PanTiltPlugin::PanTiltPlugin() :
            CSIRO::Application::WorkspacePlugin("PanTilt",
                                                "PanTilt",
                                                TOSTRING(PANTILTPLUGIN_VERSION))
    {
        pImpl_ = new PanTiltPluginImpl;
    }


    /**
     *
     */
    PanTiltPlugin::~PanTiltPlugin()
    {
        delete pImpl_;
    }


    /**
     * \return  The singleton instance of this plugin.
     */
    PanTiltPlugin&  PanTiltPlugin::getInstance()
    {
        // This is a Singleton pattern. There will only ever be one
        // instance of the plugin across the entire application.
        static PanTiltPlugin plugin;
        return plugin;
    }


    /**
     *
     */
    bool  PanTiltPlugin::setup()
    {
        // Add your data factories like this:
        //addFactory( CSIRO::DataExecution::DataFactoryTraits<MyDataType>::getInstance() );

        // Add your operation factories like this:
        //addFactory( CSIRO::DataExecution::OperationFactoryTraits<MyOperation>::getInstance() );
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<MovePanTilt>::getInstance());
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<GetAngles>::getInstance());
        addFactory(CSIRO::DataExecution::OperationFactoryTraits<ExecuteCommand>::getInstance());

        // Add your widget factories like this:
        //addFactory( MyNamespace::MyWidgetFactory::getInstance() );

        return true;
    }


    /**
     *
     */
    const CSIRO::DataExecution::OperationFactory*  PanTiltPlugin::getAliasedOperationFactory(const QString& opType) const
    {
        // If you rename an operation, you can provide backwards
        // compatibility using something like this (don't forget to
        // include namespaces in the names if relevant):
        //if (opType == "SomeOperationName")
        //    return &CSIRO::DataExecution::OperationFactoryTraits<NewOperationName>::getInstance();

        // If you make use of opType, you can delete the following Q_UNUSED line
        Q_UNUSED(opType);

        // If we get here, opType is not something we renamed, so return a
        // a null pointer to tell the caller
        return nullptr;
    }


    /**
     *
     */
    const CSIRO::DataExecution::DataFactory*  PanTiltPlugin::getAliasedDataFactory(const QString& dataType) const
    {
        // If you rename a data type, you can provide backwards
        // compatibility using something like this (don't forget to
        // include namespaces in the names if relevant):
        //if (dataType == "SomeDataType")
        //    return &CSIRO::DataExecution::DataFactoryTraits<NewDataType>::getInstance();

        // If you make use of dataType, you can delete the following Q_UNUSED line
        Q_UNUSED(dataType);

        // If we get here, dataType is not something we renamed, so return a
        // a null pointer to tell the caller
        return nullptr;
    }
    
    
    /**
     *
     */
    QStringList  PanTiltPlugin::getCustomWidgetPaths() const
    {
        QStringList result;
        result.push_back("widgets:PanTilt");
        return result;
    }
    
}

#ifndef CSIRO_STATIC_BUILD
extern "C"
{
    CSIRO_EXPORTSPEC CSIRO::Application::WorkspacePlugin* getWorkspacePlugin()
    {
        return &BoeingMetrology::PanTiltPlugin::getInstance();
    }
    
    /**
     *	\return The version string for the Workspace build we've been built against
     */
    CSIRO_EXPORTSPEC const char* builtAgainstWorkspace()
    {
        #define STRINGIFY(x) #x
        #define TOSTRING(x) STRINGIFY(x)
        return TOSTRING(CSIRO_WORKSPACE_VERSION_CHECK);
    }
}
#endif
