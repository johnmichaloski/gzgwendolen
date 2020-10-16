/*
 * DISCLAIMER:
 * This software was produced by the National Institute of Standards
 * and Technology (NIST), an agency of the U.S. government, and by statute is
 * not subject to copyright in the United States.  Recipients of this software
 * assume all responsibility associated with its operation, modification,
 * maintenance, and subsequent redistribution.
 *
 * See NIST Administration Manual 4.09.07 b and Appendix I.
 */

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <errno.h>
#include <future>
#include <fcntl.h>

#include <boost/exception/all.hpp>
#include <boost/regex.hpp>

#include "crclapp/CrclServer.h"
#include "crclapp/Crcl2Rcs.h"
#include "crclapp/NistCrcl.h"
#include "crclapp/Globals.h"

bool CBufferHandler::_bTrace;

//static std::string trim (std::string source, std::string delims = " \t\r\n")
//{
//    std::string result = source.erase(source.find_last_not_of(delims) + 1);
//    return result.erase(0, result.find_first_not_of(delims));
//}

////////////////////////////////////////////////////////////////////////////////
///  CBufferHandler
////////////////////////////////////////////////////////////////////////////////

CBufferHandler::CBufferHandler(CCrclSession * pSession)
{
    this->pSession=pSession;
}
CBufferHandler::~CBufferHandler()
{


}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::AppendBuffer(std::string read)
{
    if (_next.size() > 0) {
        _current.append(_next);
        _next.clear();
    }
    size_t oldsize = _current.size();
    _current.append(read);
    if (_endtag == NonsenseTag()) {
        _endtag = FindLeadingElement(_current);

        if (_endtag.empty()) {
            _endtag = NonsenseTag();
        }
    }
    BufferHandler(_endtag);
}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::SaveMessage(std::string xmlmessage) {
    if (CBufferHandler::_bTrace)
    {
                ROS_DEBUG("===========================================================\n"
                "%s",
                        xmlmessage.c_str());
    }

//    if(CCrclSession::bDebugCrclXML  && xmlmessage.find("GetStatusType") == std::string::npos)
//    {
//        printf("Raw:%s\n",xmlmessage.c_str());
//    }

    CCrclSession::InMessages().addMsgQueue(boost::make_tuple(xmlmessage, pSession));
    pSession->crcl2ros->wake();

}

////////////////////////////////////////////////////////////////////////////////
std::string CBufferHandler::FindLeadingElement(std::string xml)
{
    boost::match_results<std::string::const_iterator> matchResult;
    bool found;
    boost::regex e("<[A-Za-z0-9_]+");
    found = boost::regex_search(xml, matchResult, e);

    if (found) {
        std::string elem(matchResult[0]);
        elem.insert(1, 1, '/');
        elem = Globals.trim(elem);
        elem.append(">"); // not space
        return elem;
    }
    return NonsenseTag();
}

////////////////////////////////////////////////////////////////////////////////
void CBufferHandler::BufferHandler(std::string & endtag)
{    int nMaxConnections;
     int nConnections;

      std::size_t found;

       while ((found = _current.find(endtag)) != std::string::npos) {
           found = found + endtag.size();
           _next = _current.substr(found);
           _current = _current.substr(0, found);
           SaveMessage(_current);
           //_inmsgs.AddMsgQueue(boost::make_tuple(_current, this) );
           _current = _next; // MISSING? when messages are pumped out back to back
           _next.clear();
           endtag = FindLeadingElement(_current);
       }
}
////////////////////////////////////////////////////////////////////////////////
///  CCrclSession
////////////////////////////////////////////////////////////////////////////////
int CCrclSession::bDebugCrclXML;

//CMessageQueueThread *  CCrclSession::_inmsgs= NULL;  // queue with thread notify
CMessageQueue *  CCrclSession::_inmsgs= NULL;  // queue with thread notify

////////////////////////////////////////////////////////////////////////////////
CCrclSession::CCrclSession(double cycletime,
                           std::string name,
                           int port,
                           CCrcl2RosMsg* crcl2ros)   :
    RCS::Thread(cycletime, name)
{

    _remoteport=port;
    RCS::Thread::cycleTime()=cycletime;
    bufHandler = std::shared_ptr<CBufferHandler>( new CBufferHandler(this));
    mSocketServer=new SocketServer(port,(cycletime*1000),
                             (std::shared_ptr<IBufferHandler>) bufHandler.get());


}


////////////////////////////////////////////////////////////////////////////////
void CCrclSession::init()
{


}

////////////////////////////////////////////////////////////////////////////////
int CCrclSession::action()
{
    MultiClientSocket *client = mSocketServer->connectToClients();
    //          if (client != NULL)
    //            sendInitialData(client);

    mSocketServer->readFromClients();


    return 1;
}
////////////////////////////////////////////////////////////////////////////////
void CCrclSession::SyncWrite(std::string str)
{
    mSocketServer->sendToClients(str.c_str());
}
////////////////////////////////////////////////////////////////////////////////
void CCrclSession::stop (bool bWait)
{
    // disconnect clients and delete socket
    delete this->mSocketServer;
}

////////////////////////////////////////////////////////////////////////////////
void CCrclSession::cleanup()
{

}
