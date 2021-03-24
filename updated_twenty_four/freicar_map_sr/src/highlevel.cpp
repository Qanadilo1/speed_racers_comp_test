/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
 * AUTHOR: Fabien Jenne
 *
 * Thrift filter implementation
 *
******************************************************************************/

#include "freicar_map/thrift_map_proxy.h"
#include <ros/package.h>

namespace freicar
{
namespace map
{
using namespace apache::thrift; 
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;
using namespace apache::thrift::server;

ThriftMapProxy::ThriftMapProxy(const std::string& remote_address, int remote_port, int local_server_port) {
    client_ = new ThriftMapProxy::ThriftClient(remote_address, remote_port);
    server_ = new ThriftMapProxy::ThriftServer(local_server_port);
    // create thrift server
    server_->InitializeThriftServer();
    // create car object and initialize to 0
}

ThriftMapProxy::~ThriftMapProxy() {
    std::cout << "shutting down map " << std::endl;
    server_->StopThreadedServer();
//    ClearMap();
    delete server_;
    delete client_;
    server_ = nullptr;
    client_ = nullptr;
}

/* function to load a map from file*/
bool ThriftMapProxy::LoadMapFromFile(const std::string& path){
	bool success = false;
	try {
		apache::thrift::stdcxx::shared_ptr<TFileTransport> transport(
                    new TFileTransport(path));
        apache::thrift::stdcxx::shared_ptr<TProtocol> protocol(
                    new apache::thrift::protocol::TBinaryProtocol(transport));
        map_thrift::MapMessage map_file;
        map_file.read(protocol.get());
        Map::GetInstance().ProcessMapMessage(map_file, MapStatus::LOADED_FROM_FILE);
        success = true;
    } catch(apache::thrift::transport::TEOFException& ex) {
        std::cout << "something went wrong trying to read from map file" << std::endl;
        std::cout << ex.what() << std::endl;
    } catch(apache::thrift::transport::TTransportException &ex) {
		std::cout << "something went wrong trying to read from map file" << std::endl;
        std::cout << ex.what() << std::endl;
	}
	return success;
}

/* function to save the map as map_message to the storage */
void ThriftMapProxy::WriteMapToFile(const std::string& path) {
    map_thrift::MapMessage map_message = ReturnMapAsMessage();
    if (map_message.op == map_thrift::MessageOp::ADD || map_message.op == map_thrift::MessageOp::UPDATE_WHOLE) {            
        std::cout << "thrift proxy: writing received map to file : " << path << std::endl;
        apache::thrift::stdcxx::shared_ptr<TFileTransport> transport(new TFileTransport(path));
        apache::thrift::stdcxx::shared_ptr<TProtocol> protocol(new TBinaryProtocol(transport));
        map_message.write(protocol.get());
    } else {
        std::cout << "thrift proxy: cannot save map of type " + GetMessageOpName(map_message.op) << std::endl;
    }
}
/* stop the local thrift server */
void ThriftMapProxy::StopMapServer(){
    server_->StopThreadedServer();
}

/* start the local thrift server */
void ThriftMapProxy::StartMapServer() {
    server_->StartThreadedServer();   
}

/* a simple helper function to print the Map stats */
void ThriftMapProxy::PrintMapStats() {
    auto stats = Map::GetInstance().GetMapStats();
    std::cout << "thrift proxy: Lanes: " << std::get<0>(stats) << 
				 " LaneGroups: " << std::get<1>(stats) << 
				 " LaneMarkings: " << std::get<2>(stats) << std::endl;
}

/* delete the global map instance */
bool ThriftMapProxy::ClearMap() {
    std::cout << "thrift proxy: deleting internal map" << std::endl;
    bool is_deleted = Map::GetInstance().ClearMap();
    PrintMapStats();
    return is_deleted;
}

/* function which returns the whole internal map as map message */
map_thrift::MapMessage ThriftMapProxy::ReturnMapAsMessage() {
	return Map::GetInstance().AsThriftMessagae();
}

/* function to get the map_thrift::MessageOp name */
std::string ThriftMapProxy::GetMessageOpName(const map_thrift::MessageOp::type& m_type) {
    int t = static_cast<int>(m_type);
    switch (t) {
    case 1:
        return "ADD";
    case 2:
        return "UPDATE_WHOLE";
    case 3:
        return "UPDATE_PART";
    case 4:
        return "DELETE";
    case 5:
        return "POSE_UPDATE";
    case 6:
        return "POSE_DELETE";
    default:
        return"UNKNOWN TYPE";
    }
    return "error parsing message type";
}

/***************************
**Thrift Server  Functions**
****************************/

/* start the thrift server thread */
void ThriftMapProxy::ThriftServer::StartThreadedServer(){
    std::cout << "thrift proxy: starting server thread ..." << std::endl;
    thrift_server_thread_ = std::thread(&ThriftServer::ThriftServerThreadFunction, this);
}

/* function to intialize the thrift server, use Start- StopMapServer for other functionalities. */
void ThriftMapProxy::ThriftServer::InitializeThriftServer() {
    // create the Thrift server
    apache::thrift::stdcxx::shared_ptr<MapCommHandler> handler(
                new ThriftMapProxy::MapCommHandler());
    apache::thrift::stdcxx::shared_ptr<TProcessor> processor(
                new map_thrift::MapCommProcessor(handler));
    apache::thrift::stdcxx::shared_ptr<TServerTransport> serverTransport(
                new TServerSocket(static_cast<int>(thrift_server_port_)));
    apache::thrift::stdcxx::shared_ptr<TTransportFactory> transportFactory(
                new TBufferedTransportFactory());
    apache::thrift::stdcxx::shared_ptr<TProtocolFactory> protocolFactory(
                new TBinaryProtocolFactory());
    thrift_server_object_ = new TSimpleServer(
                processor, serverTransport, transportFactory, protocolFactory);
    std::cout << "thrift proxy: initialized thrift map server" << std::endl;
}

/* thrift server thread function */
void ThriftMapProxy::ThriftServer::ThriftServerThreadFunction() {
    if (thrift_server_object_) {
        std::cout << "thrift proxy: server thread #" << std::this_thread::get_id() << " running on port " << thrift_server_port_ << std::endl;
        try {
            thrift_server_object_->serve();
            std::cout << "thrift proxy: thrift server returned from serve()" << std::endl;
        } catch (...) {
            std::cout << "thrift proxy: something went wrong trying to serve" << std::endl;
        }
    } else {
        std::cout <<  "thrift proxy: TSimpleServer is null_ptr" << std::endl;
    }
}

/* function to stop the thrift server and merge its thread */
void ThriftMapProxy::ThriftServer::StopThreadedServer() {
    if (thrift_server_object_) {
        std::cout << "thrift proxy: stopping thrift server..." << std::endl;
        try {
            thrift_server_object_->stop();
            std::cout << "thrift proxy: stopped serving!" << std::endl;
            thrift_server_thread_.join();
            std::cout << "thrift proxy: joined server thread!" << std::endl;
        } catch (...) {
            std::cout << "thrift proxy: something went wrong trying to stop the server" << std::endl;
        }
    } else {
        std::cout << "thrift proxy: TSimpleServer is null_ptr" << std::endl;
        thrift_server_thread_.join();
    }
}

/**************************
**Thrift Client Functions**
***************************/

/* thrift client destructor */
ThriftMapProxy::ThriftClient::~ThriftClient(){
    if (thrift_client_object_)
    {
        if (thrift_transport_client_->isOpen()){
            try {
                thrift_transport_client_->close();
                std::cout << "thrift proxy: closed thrift transport client" << std::endl;
                connected_to_thrift_server_ = false;
            } catch (TException& tx) {
                std::cout << "thrift proxy: failed to close thrift transport client" << tx.what() << std::endl;
            }
        }
        delete thrift_client_object_;
        thrift_client_object_ = nullptr;
    }
}

/* returns whether we're connected to a remote host */
bool ThriftMapProxy::ThriftClient::connected(){
    return connected_to_thrift_server_;
}

/* function to check if connect to the remote server if not already connected */
void ThriftMapProxy::ThriftClient::ConnectToRemoteHost() {
    if (!connected()) {
        stdcxx::shared_ptr<TTransport> socket(new TSocket(static_cast<std::string>((remote_thrift_server_address_)),
                        					  static_cast<int>(remote_thrift_server_port_)));
        thrift_transport_client_ = stdcxx::shared_ptr<TTransport>(new TBufferedTransport(socket));
        stdcxx::shared_ptr<TProtocol> protocol(new TBinaryProtocol(thrift_transport_client_));
        thrift_client_object_ = new map_thrift::MapCommClient(protocol);

        try {
            thrift_transport_client_->open();
            thrift_client_object_->ping();
            std::cout << "thrift proxy: sent PING to remote server" << std::endl;
            connected_to_thrift_server_ = true;
        } catch (TException& tx) {
            std::cout << "thrift proxy: Couldn't connect to remote Thrift server" << std::endl;
            std::cout << "thrift proxy: ERROR:" << tx.what() << std::endl;
            connected_to_thrift_server_ = false;
        }
    } else {
        try {
            thrift_client_object_->ping();
            std::cout << "thrift proxy: ping to remote server successful" << std::endl;
            connected_to_thrift_server_ = true;
        } catch (TException& tx) {
            std::cout << "thrift proxy: Couldn't connect to remote Thrift server" << std::endl;
            std::cout << "thrift proxy: ERROR: %s" << tx.what() << std::endl;
            connected_to_thrift_server_ = false;
        }
    }
}

/* function to send out a map message from the client side */
void ThriftMapProxy::ThriftClient::SendMapMessage(const map_thrift::MapMessage& mm) {
    if (!connected()) {
        ConnectToRemoteHost();
    }
    if (connected()) {
        try {
            thrift_client_object_->sendMapMessage(mm);
        } catch (TException& tx) {
            std::cout << "thrift proxy: Could not send MapMessage. Catching exception" << std::endl;
            std::cout << tx.what() << std::endl;
        }
    }
}


/******************************************
**Thrift Server Message Handler Functions**
*******************************************/

/* thrift handler function for incoming car messages, don't mind the name */
bool ThriftMapProxy::MapCommHandler::sendCarPose(const map_thrift::CarMessage & car_pose) {
    (void) car_pose;  // added to silence compiler
    std::cout <<  "thrift proxy: receiving CarMessage from remote. Ignoring!" << std::endl;
    return false;
}

/* thrift handler function for incoming image messages, don't mind the name */
bool ThriftMapProxy::MapCommHandler::sendThriftImage(const map_thrift::ThriftImage& image) {
    (void) image;  // added to silence compiler
    std::cout <<  "thrift proxy: receiving ThriftImage from remote. Ignoring!" << std::endl;
    return true;
}
/* thrift handler function for incoming ping messages, don't mind the name */
void ThriftMapProxy::MapCommHandler::ping() {
    std::cout <<  "thrift proxy: received PING from remote" << std::endl;
}
/* thrift handler function for incoming map messages, don't mind the name */
bool ThriftMapProxy::MapCommHandler::sendMapMessage(const map_thrift::MapMessage& map_complete) {
    std::cout << "thrift proxy: receiving " << ThriftMapProxy::GetMessageOpName(map_complete.op) << " map message from remote" << std::endl;
    Map::GetInstance().ProcessMapMessage(map_complete, MapStatus::UPDATED_FROM_REMOTE);
    return true;
}

} // namespace map
} // namespace freicar