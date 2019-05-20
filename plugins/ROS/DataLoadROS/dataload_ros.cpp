#include "dataload_ros.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QProcess>
#include <rosbag/view.h>
#include <sys/sysinfo.h>
#include <QSettings>
#include <QElapsedTimer>

#include "../dialog_select_ros_topics.h"
#include "../shape_shifter_factory.hpp"
#include "../rule_editing.h"
#include "../dialog_with_itemlist.h"

DataLoadROS::DataLoadROS()
{
    _extensions.push_back( "bag");
}

void StrCat(const std::string& a, const std::string& b,  std::string& out)
{
    out.clear();
    out.reserve(a.size() + b.size());
    out.assign(a);
    out.append(b);
}

const std::vector<const char*> &DataLoadROS::compatibleFileExtensions() const
{
    return _extensions;
}

size_t getAvailableRAM()
{
    struct sysinfo info;
    sysinfo(&info);
    return info.freeram;
}

std::vector<std::pair<QString,QString>> DataLoadROS::getAndRegisterAllTopics()
{
    std::vector<std::pair<QString,QString>> all_topics;
    rosbag::View bag_view ( *_bag, ros::TIME_MIN, ros::TIME_MAX, true );

    RosIntrospectionFactory::reset();

    for(auto& conn: bag_view.getConnections() )
    {
        const auto&  topic      =  conn->topic;
        const auto&  md5sum     =  conn->md5sum;
        const auto&  datatype   =  conn->datatype;
        const auto&  definition =  conn->msg_def;

        all_topics.push_back( std::make_pair(QString( topic.c_str()), QString( datatype.c_str()) ) );
        _introspection_parser.registerSchema(
                    topic, md5sum, RosIntrospection::ROSType(datatype), definition);
        RosIntrospectionFactory::registerMessage(topic, md5sum, datatype, definition);
    }
    return all_topics;
}

void DataLoadROS::storeMessageInstancesAsUserDefined(PlotDataMapRef& plot_map,
                                                     const std::string& prefix)
{
    using namespace RosIntrospection;

    rosbag::View bag_view ( *_bag, ros::TIME_MIN, ros::TIME_MAX, false );

    RenamedValues renamed_value;

    std::string prefixed_name;

    PlotDataAny& plot_consecutive = plot_map.addUserDefined( "__consecutive_message_instances__" )->second;

    for(const rosbag::MessageInstance& msg_instance: bag_view )
    {
        const std::string& topic_name  = msg_instance.getTopic();
        double msg_time = msg_instance.getTime().toSec();
        auto data_point = PlotDataAny::Point(msg_time, nonstd::any(msg_instance) );
        plot_consecutive.pushBack( data_point );

        if( prefix.empty() == false)
        {
            StrCat(prefix, topic_name, prefixed_name);
        }
        const std::string* key_ptr = prefix.empty() ? &topic_name : &prefixed_name;

        auto plot_pair = plot_map.user_defined.find( *key_ptr );

        if( plot_pair == plot_map.user_defined.end() )
        {
            plot_pair = plot_map.addUserDefined( *key_ptr );
        }
        PlotDataAny& plot_raw = plot_pair->second;
        plot_raw.pushBack( data_point );
    }
}

PlotDataMapRef DataLoadROS::readDataFromFile(const QString &file_name, bool use_previous_configuration)
{
    if( _bag ) _bag->close();

    _bag = std::make_shared<rosbag::Bag>();
    _introspection_parser.clear();

    try{
        _bag->open( file_name.toStdString(), rosbag::bagmode::Read );
    }
    catch( rosbag::BagException&  ex)
    {
        QMessageBox::warning(nullptr, tr("Error"),
                             QString("rosbag::open thrown an exception:\n")+
                             QString(ex.what()) );
        return PlotDataMapRef();
    }

    auto all_topics = getAndRegisterAllTopics();

    //----------------------------------
    QSettings settings;

    _use_renaming_rules = settings.value("DataLoadROS/use_renaming").toBool();

    if( _default_topic_names.empty() )
    {
        // if _default_topic_names is empty (xmlLoad didn't work) use QSettings.
        QVariant def = settings.value("DataLoadROS/default_topics");
        if( !def.isNull() && def.isValid())
        {
            _default_topic_names = def.toStringList();
        }
    }

    DialogSelectRosTopics* dialog = new DialogSelectRosTopics( all_topics, _default_topic_names );

    if( !use_previous_configuration )
    {
        if( dialog->exec() == static_cast<int>(QDialog::Accepted) )
        {
            _default_topic_names = dialog->getSelectedItems();
            settings.setValue("DataLoadROS/default_topics", _default_topic_names);
            settings.setValue("DataLoadROS/use_renaming", _use_renaming_rules);
        }
        else{
            return PlotDataMapRef();
        }
    }

    bool use_header_stamp = dialog->checkBoxTimestamp()->isChecked();

    _use_renaming_rules = dialog->checkBoxUseRenamingRules()->isChecked();

    if( _use_renaming_rules )
    {
        _introspection_parser.addRules( RuleEditing::getRenamingRules() );
    }

    const std::string prefix    = dialog->prefix().toStdString();

    //-----------------------------------
    std::set<std::string> topic_selected;
    for(const auto& topic: _default_topic_names)
    {
        topic_selected.insert( topic.toStdString() );
    }

    QProgressDialog progress_dialog;
    progress_dialog.setLabelText("Loading... please wait");
    progress_dialog.setWindowModality( Qt::ApplicationModal );

    rosbag::View bag_view_selected ( true );
    bag_view_selected.addQuery( *_bag, [topic_selected](rosbag::ConnectionInfo const* connection)
    {
        return topic_selected.find( connection->topic ) != topic_selected.end();
    } );
    progress_dialog.setRange(0, bag_view_selected.size()-1);
    progress_dialog.show();

    PlotDataMapRef plot_map;

    std::vector<uint8_t> buffer;

    int msg_count = 0;
    std::string prefixed_name;
    QElapsedTimer timer;
    timer.start();

    _introspection_parser.setMaxArrayPolicy(
                dialog->maxArraySize(),
                dialog->discardEntireArrayIfTooLarge() );

    for(const rosbag::MessageInstance& msg_instance: bag_view_selected )
    {
        const std::string& topic_name  = msg_instance.getTopic();
        const size_t msg_size  = msg_instance.size();

        buffer.resize(msg_size);

        if( msg_count++ %100 == 0)
        {
            progress_dialog.setValue( msg_count );
            QApplication::processEvents();

            if( progress_dialog.wasCanceled() ) {
                return PlotDataMapRef();
            }
        }

        ros::serialization::OStream stream(buffer.data(), buffer.size());
        msg_instance.write(stream);

        const double msg_time = msg_instance.getTime().toSec();

        RawMessage buffer_view( buffer );
        _introspection_parser.pushRawMessage( topic_name, buffer_view, msg_time );
    }

    _introspection_parser.extractData(plot_map, prefix);

    storeMessageInstancesAsUserDefined(plot_map, prefix);

    qDebug() << "The loading operation took" << timer.elapsed() << "milliseconds";

    _introspection_parser.showWarnings();

    return plot_map;
}


DataLoadROS::~DataLoadROS()
{

}

QDomElement DataLoadROS::xmlSaveState(QDomDocument &doc) const
{
    QString topics_list = _default_topic_names.join(";");
    QDomElement list_elem = doc.createElement("selected_topics");
    list_elem.setAttribute("list", topics_list );
    return list_elem;
}

bool DataLoadROS::xmlLoadState(QDomElement &parent_element)
{
    QDomElement list_elem = parent_element.firstChildElement( "selected_topics" );
    if( !list_elem.isNull()    )
    {
        if( list_elem.hasAttribute("list") )
        {
            QString topics_list = list_elem.attribute("list");
            _default_topic_names = topics_list.split(";", QString::SkipEmptyParts);
            return true;
        }
    }
    return false;
}


