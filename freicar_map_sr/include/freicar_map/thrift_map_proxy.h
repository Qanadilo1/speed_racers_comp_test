/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
 * AUTHOR: Fabien Jenne
 * 
 * Thrift filter implementation
 ******************************************************************************/
/*
changes : made the code conform to google style as much as possible. The class was called RosMap before, now 
called ThriftMapProxy because it's actually a proxy over thrift to get map access. I had to make two sub-classes to 
separate client and server related functions. Doing everything in one class with unrelated name is bad practice.
*/

#ifndef __THRIFT_MAP_PROXY__
#define __THRIFT_MAP_PROXY__

#include <thrift/stdcxx.h>
#include <thrift/transport/TSocket.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TTransportUtils.h>
#include <thrift/transport/TBufferTransports.h>

#include <string>
#include <vector>
#include <thread>
#include <cstdlib>
#include <sstream>
#include <iostream>

#include "map_core/freicar_map.h"
#include "gen-cpp/MapComm.h"

using TSimpleServer = apache::thrift::server::TSimpleServer;
using TTransport = apache::thrift::transport::TTransport;
using MapCommIf = map_thrift::MapCommIf;

namespace freicar
{
namespace map
{

class ThriftMapProxy
{
 public:
   	ThriftMapProxy(const std::string& remote_address = "192.168.168.127", int remote_port = 9091, int local_server_port = 9090);
   	~ThriftMapProxy();
    
    // map functions
	bool LoadMapFromFile(const std::string& path);
    void WriteMapToFile(const std::string& path);
    void PrintMapStats();
    static std::string GetMessageOpName(const map_thrift::MessageOp::type& m_type);

    // server functions
 	void StartMapServer();
    void StopMapServer();
    
    // client functions
    // IMPLEMENT: expose if needed

 private:
    // Map
    std::string file_extension_ = ".aismap";
    map_thrift::MapMessage ReturnMapAsMessage();
    bool ClearMap();
    
    class ThriftClient{
    public:
        ThriftClient(const std::string& address = "192.168.168.127", int port = 9091) : 
                                                        remote_thrift_server_address_(address), 
                                                        remote_thrift_server_port_(port), thrift_client_object_(nullptr)
        {std::cout << "thrift proxy: initialized thrift map client" << std::endl; }
        ~ThriftClient();

        map_thrift::CarMessage CreateCarMessagePoseUpdate(const map_thrift::Pose& c_pose);
        map_thrift::CarMessage CreateCarMessagePoseDelete();
        void SendMapMessage(const map_thrift::MapMessage& mm);
	    void ConnectToRemoteHost();
        bool connected();
    private :
        std::string remote_thrift_server_address_;
        int remote_thrift_server_port_;
        bool connected_to_thrift_server_ = false;
        
        apache::thrift::stdcxx::shared_ptr<TTransport> thrift_transport_client_;
        map_thrift::MapCommClient* thrift_client_object_ = nullptr;
    };
    class ThriftServer{
    public:
        ThriftServer(int port = 9090) : thrift_server_port_(port) {}
	    void InitializeThriftServer();
	    void StopThreadedServer();
        void StartThreadedServer();
    private:
	    void ThriftServerThreadFunction();
        int thrift_server_port_;
        std::thread thrift_server_thread_;
        TSimpleServer* thrift_server_object_;
    };
    // definition of the thrift server message handler
    class MapCommHandler : virtual public MapCommIf {
     public:
        explicit MapCommHandler() {}
        void ping();
        bool sendMapMessage(const map_thrift::MapMessage& map_complete);
        bool sendCarPose(const map_thrift::CarMessage& car_pose);
        bool sendThriftImage(const map_thrift::ThriftImage& image);
    };
    // raw pointers; memory is handled dynamically 
    // using the destructor of the enclosing class
    ThriftClient *client_ = nullptr;
    ThriftServer *server_ = nullptr;

};

} // namespace map
} // namespace freicar
#endif // ROSMAP_H
