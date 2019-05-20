#ifndef DIAGNOSTIC_MSG_H
#define DIAGNOSTIC_MSG_H

#include "diagnostic_msgs/DiagnosticArray.h"
#include "ros_messageparser.h"
#include <absl/strings/str_cat.h>


class DisagnosticMsg: public RosMessageParser<diagnostic_msgs::DiagnosticArray>
{
public:

    DisagnosticMsg()
    {
        _header_data.emplace_back( "/header/seq" );
        _header_data.emplace_back( "/header/stamp" );
    }

    virtual void pushRawMessage(const MessageKey& ,
                                const RawMessage& msg,
                                double timestamp) override
    {
        diagnostic_msgs::DiagnosticArray status_array;
        ros::serialization::IStream is( const_cast<uint8_t*>(msg.data()), msg.size() );
        ros::serialization::deserialize(is, status_array);

        if( _use_header_stamp )
        {
            timestamp = status_array.header.stamp.toSec();
        }

        _header_data[0].pushBack( {timestamp, (double)status_array.header.seq} );
        _header_data[1].pushBack( {timestamp, status_array.header.stamp.toSec()} );

        for( const auto& status: status_array.status)
        {
            for( const auto& kv: status.values)
            {
                double val = strtod (kv.value.data(), nullptr);

                std::string status_prefix;
                if( status.hardware_id.empty()){
                    status_prefix = absl::StrCat( "/", status.name,  "/",
                                                  kv.key );
                }
                else {
                    status_prefix = absl::StrCat( "/",status.hardware_id, "/",
                                                  status.name,  "/",
                                                  kv.key );
                }
                auto it = _data.find(status_prefix);
                if( it == _data.end() )
                {
                    it = _data.emplace( std::piecewise_construct,
                                        std::forward_as_tuple(status_prefix),
                                        std::forward_as_tuple(status_prefix)
                                        ).first;
                }
               it->second.pushBack( { timestamp, val } );
            }
        }

    }

    void extractData(PlotDataMapRef& plot_map, const std::string& prefix) override
    {
        for (auto& it: _header_data)
        {
            appendData(plot_map, prefix + it.name(), it);
        }
        for (auto& it: _data)
        {
            appendData(plot_map, prefix + it.first, it.second);
        }
    }

private:
    std::vector<PlotData> _header_data;
    std::unordered_map<std::string,PlotData> _data;
};



#endif // DIAGNOSTIC_MSG_H
