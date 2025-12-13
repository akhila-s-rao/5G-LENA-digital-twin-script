/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <ns3/nstime.h>
#include <string>
#include <ostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include "ns3/core-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/antenna-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/lte-module.h"
#include <ns3/radio-environment-map-helper.h>
#include "ns3/config-store-module.h"
#include "ns3/inet-socket-address.h"
#include "ns3/nr-ue-net-device.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-rrc.h"
#include "ns3/nr-eps-bearer.h"
#include "ns3/seq-ts-size-frag-header.h"
#include <iomanip>
#include "ns3/log.h"

#ifndef CELLULAR_NETWORK_FUNCTION_H
#define CELLULAR_NETWORK_FUNCTION_H

namespace ns3 {

// Contains all parameters we are setting. Command line user settable parameters are included in cellular-netwokr-user.cc
// The rest are still set here but not user settable. 
struct Parameters
{
    friend std::ostream& operator<< (std::ostream& os, const Parameters& parameters);

    std::string ns3Dir = "/home/ubuntu/ns-3-dev/";
    std::string digitalTwinScenario = "expeca";

    // Deployment topology parameters
    uint16_t numUes = 3;
    // num of gNodeBs is set at 1 for now
    double BsHeight = 10;
    double ueHeight = 1.5;

    // Simulation parameters
    bool traces = true;
    Time appGenerationTime = Seconds (5);
    Time appStartTime = MilliSeconds (500);
    Time progressInterval = Seconds (1);
    uint32_t randSeed = 13;

    // NR RAN parameters (Reference: 3GPP TR 38.901 V17.0.0 (Release 17)
    // Table 7.8-1 for the power and BW).
    // This example uses a single operational band/BWP
    uint16_t numerologyBwp1 = 1;
    std::string channelScenario = "InH-OfficeOpen";
    double centralFrequencyBand = 3.5e9;
    double bandwidthHz = 40e6;
    // the pattern length needs to be as long as the number of slots in a 10 ms frame
    // So adjust according to your numerology 
    std::string tddPattern
        = "DL|DL|DL|S|UL|DL|DL|DL|S|UL|DL|DL|DL|S|UL|DL|DL|DL|S|UL";
    uint16_t BsTxPower = 20;
    bool enableUlPc = true;
    uint16_t NumberOfRaPreambles = 40;
    bool UseIdealRrc = true;
    
    // Buffer sizes 
    uint32_t rlcTxBuffSize = 200 * 1024; // default is 10240 
    uint32_t tcpUdpBuffSize = 500 * 1024; // default is 131072

    // position and mobility model
    double boundingBoxMinX = -50.0;
    double boundingBoxMaxX = 50.0;
    double boundingBoxMinY = -50.0;
    double boundingBoxMaxY = 50.0;
    double ueMinSpeed = 0.5;
    double ueMaxSpeed = 1.5;

    // Application traffic parameters
    bool traceDelay = true;
    bool traceRtt = true;
    bool traceVr = true;
    std::string traceFolder = ns3Dir + "contrib/vr-app/model/BurstGeneratorTraces/";
    std::vector<std::string> vrTraceFiles {
        "mc_10mbps_30fps.csv",
        "ge_cities_10mbps_30fps.csv",
        "ge_tour_10mbps_30fps.csv",
        "vp_10mbps_30fps.csv",
        "mc_10mbps_60fps.csv",
        "ge_cities_10mbps_60fps.csv",
        "ge_tour_10mbps_60fps.csv",
        "vp_10mbps_60fps.csv"};
    std::string vrTrafficType = "synthetic"; // trace, synthetic, none
    uint16_t vrFrameRate = 30;           // allowed: 30 or 60
    double vrTargetDataRateMbps = 10.0;
    std::string vrAppProfile = "VirusPopper";
    bool createRemMap = false;
    uint8_t vrBearerQci = NrEpsBearer::NGBR_LOW_LAT_EMBB;
    uint8_t controlBearerQci = NrEpsBearer::DGBR_DISCRETE_AUT_LARGE;

    // VR
    uint16_t numUesWithVrApp = 1;
    double vrStartTimeMin = 1;
    double vrStartTimeMax = 3;

    // UDP one way delay probes
    uint32_t delayPacketSize = 1400;
    Time delayInterval = Seconds (0.1);

    // UDP echo
    uint32_t echoPacketSize = 1400;
    Time echoInterPacketInterval = Seconds (0.1);

    void ApplyScenarioDefaults()
    {
        std::string scenario = digitalTwinScenario;
        std::transform(scenario.begin(), scenario.end(), scenario.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (scenario == "expeca")
        {
            digitalTwinScenario = "expeca";
            return;
        }
        // Add here if you want a parameter to be part of the scenario specific setting
        if (scenario == "5gsmart")
        {
            digitalTwinScenario = "5gsmart";
            channelScenario = "InH-OfficeMixed";
            BsHeight = 0.0;
            ueHeight = 0.0;
            numerologyBwp1 = 0;
            centralFrequencyBand = 0.0;
            bandwidthHz = 0.0;
            tddPattern.clear();
            BsTxPower = 0;
            enableUlPc = false;
            NumberOfRaPreambles = 0;
            boundingBoxMinX = 0.0; 
            boundingBoxMaxX = 0.0;
            boundingBoxMinY = 0.0;
            boundingBoxMaxY = 0.0;
            vrBearerQci = 0;
            controlBearerQci = 0;
            return;
        }
        NS_ABORT_MSG("Unknown digital twin scenario: " << digitalTwinScenario);
    }

    // Validate whether the parameters set are acceptable
    bool Validate () const
    {
        NS_ABORT_MSG_IF(!(vrTrafficType == "trace" || vrTrafficType == "synthetic" ||
                          vrTrafficType == "none"),
                        "vrTrafficType must be 'trace', 'synthetic', or 'none'");
        if (vrTrafficType == "trace")
        {
            NS_ABORT_MSG_IF(vrFrameRate != 30 && vrFrameRate != 60,
                            "Trace VR frame rate must be 30 or 60 fps");
        }
        if (vrTrafficType == "synthetic")
        {
            NS_ABORT_MSG_IF(vrFrameRate != 30 && vrFrameRate != 60,
                            "Synthetic VR frame rate must be 30 or 60 fps");
            NS_ABORT_MSG_IF(vrTargetDataRateMbps <= 0.0,
                            "Synthetic VR target data rate must be positive");
        }
        return true;
    }
};

void CellularNetwork (const Parameters& params);

/***************************
 * Global declarations
 ***************************/

const Time appStartWindow = MilliSeconds (500);


#ifdef CELLULAR_NETWORK_IMPLEMENTATION
NodeContainer gnbNodes;
NodeContainer ueNodes;
Ipv4InterfaceContainer ueIpIfaces;
Parameters global_params;
AsciiTraceHelper traceHelper;
Ptr<OutputStreamWrapper> mobStream;
Ptr<OutputStreamWrapper> delayStream;
Ptr<OutputStreamWrapper> rttStream;
Ptr<OutputStreamWrapper> fragmentRxStream;
Ptr<OutputStreamWrapper> burstRxStream;
Ptr<OutputStreamWrapper> ueGroupsStream;
Ptr<OutputStreamWrapper> simInfoStream;
std::unordered_map<uint16_t, uint32_t> g_rttNextSeqPerUe;
std::unordered_map<uint32_t, uint16_t> g_nodeIdToUeId;
#else
extern NodeContainer gnbNodes;
extern NodeContainer ueNodes;
extern Ipv4InterfaceContainer ueIpIfaces;
extern Parameters global_params;
extern AsciiTraceHelper traceHelper;
extern Ptr<OutputStreamWrapper> mobStream;
extern Ptr<OutputStreamWrapper> delayStream;
extern Ptr<OutputStreamWrapper> rttStream;
extern Ptr<OutputStreamWrapper> fragmentRxStream;
extern Ptr<OutputStreamWrapper> burstRxStream;
extern Ptr<OutputStreamWrapper> ueGroupsStream;
extern Ptr<OutputStreamWrapper> simInfoStream;
extern std::unordered_map<uint16_t, uint32_t> g_rttNextSeqPerUe;
extern std::unordered_map<uint32_t, uint16_t> g_nodeIdToUeId;
#endif


/***************************
 * Function Declarations
 ***************************/

uint16_t GetUeIdFromNodeId (uint16_t nodeId);
uint16_t GetNodeIdFromContext (std::string context);
uint16_t GetUeNodeIdFromIpAddr (Address ip_addr, const NodeContainer* ueNodes, const Ipv4InterfaceContainer* ueIpIfaces);
uint16_t GetImsi_from_ueId(uint16_t ueId);
uint16_t GetImsi_from_node(Ptr<ns3::Node> ue_node);    
uint16_t GetCellId_from_ueId(uint16_t ueId);
uint16_t GetCellId_from_ueNode(Ptr<ns3::Node> ue_node);    
void PrintSimInfoToFile();
void NotifyConnectionEstablishedUe (std::string context, uint64_t imsi,
                               uint16_t cellid, uint16_t rnti);
void NotifyConnectionEstablishedEnb (std::string context, uint64_t imsi,
                                uint16_t cellid, uint16_t rnti);
void LogPosition (Ptr<OutputStreamWrapper> stream);
void udpServerTrace(std::pair<uint16_t, uint16_t> DelayPortNums,
                const Ptr<Node> &remoteHost,
                std::string context,
                Ptr<const Packet> packet, 
               const Address &from, const Address &localAddress);
void delayTrace (Ptr<OutputStreamWrapper> stream,
                const Ptr<Node> &remoteHost,
                std::string context,
                Ptr<const Packet> packet, const Address &from, const Address &localAddress);
void rttTrace (Ptr<OutputStreamWrapper> stream,
                std::string context, 
                Ptr<const Packet> packet, const Address &from, const Address &localAddress);
void StampEchoClientPacket(uint16_t ueId,
                           Ptr<const Packet> packet,
                           const Address& local,
                           const Address& remote);
void BurstRx (Ptr<OutputStreamWrapper> stream,
                std::string context, Ptr<const Packet> burst, const Address &from, const Address &to,
         const SeqTsSizeFragHeader &header);
void FragmentRx (Ptr<OutputStreamWrapper> stream,
                std::string context, Ptr<const Packet> fragment, const Address &from, const Address &to,
         const SeqTsSizeFragHeader &header);    
    
    
std::pair<ApplicationContainer, Time> 
InstallUdpEchoApps (const Ptr<Node> &ue,
             UdpEchoClientHelper *echoClient, Time appStartTime,
             const Ptr<UniformRandomVariable> &x,
             Time appGenerationTime);
std::pair<ApplicationContainer, Time> 
InstallUlDelayTrafficApps (const Ptr<Node> &ue,
             UdpClientHelper *ulDelayClient,
             const Ptr<Node> &remoteHost,
             const Ipv4Address &remoteHostAddr, uint16_t remotePort, Time appStartTime,
             const Ptr<UniformRandomVariable> &x,
             Time appGenerationTime);
std::pair<ApplicationContainer, Time> 
InstallDlDelayTrafficApps (const Ptr<Node> &ue,
             const Ipv4Address &ueAddress,
             UdpClientHelper *dlDelayClient,
             const Ptr<Node> &remoteHost,
             uint16_t remotePort, Time appStartTime,
             const Ptr<UniformRandomVariable> &x,
             Time appGenerationTime);
void CreateTraceFiles (void);
    
/***************************
 * Function Definitions
 ***************************/

#ifdef CELLULAR_NETWORK_IMPLEMENTATION

struct UeTraceIds
{
    uint16_t ueId;
    uint64_t imsi;
    uint16_t cellId;
};

inline UeTraceIds
MakeUeTraceIds(uint16_t ueId)
{
    Ptr<Node> ueNode = ueNodes.Get(ueId);
    Ptr<NrUeNetDevice> ueDev = ueNode->GetDevice(0)->GetObject<NrUeNetDevice>();
    NS_ABORT_IF(ueDev == nullptr);
    Ptr<NrUeRrc> rrc = ueDev->GetRrc();
    UeTraceIds ids{ueId, ueDev->GetImsi(), 0};
    if (rrc)
    {
        ids.cellId = rrc->GetCellId();
    }
    return ids;
}

inline UeTraceIds
MakeUeTraceIdsFromContext(const std::string& context)
{
    return MakeUeTraceIds(GetUeIdFromNodeId(GetNodeIdFromContext(context)));
}

uint16_t
GetUeIdFromNodeId(uint16_t nodeId)
{
    auto it = g_nodeIdToUeId.find(nodeId);
    NS_ABORT_MSG_IF(it == g_nodeIdToUeId.end(),
                    "NodeId " << nodeId << " does not correspond to a UE");
    return it->second;
}

uint16_t
GetNodeIdFromContext (std::string context){
  std::string path = context.substr (10, context.length());
  std::string nodeIdStr = path.substr (0, path.find ("/"));
  uint16_t nodeId = stoi(nodeIdStr);
  return (nodeId);
}

uint16_t
GetUeNodeIdFromIpAddr (Address ip_addr, const NodeContainer* ueNodes,
                const Ipv4InterfaceContainer* ueIpIfaces
                ) {
  bool knownSender = false;
  // From the Ip addr get NetDevice and then Node
  // There is probably a way to do this through indexing 
  // but I am looping through the entire UE list to get the right node 
  for (uint32_t ueId = 0; ueId < ueNodes->GetN (); ++ueId){
    Ptr<Node> ue_node = ueNodes->Get (ueId);
    Ptr<Ipv4> ipv4 = ue_node->GetObject<Ipv4> ();
    Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0); 
    Ipv4Address ue_addr = iaddr.GetLocal ();
    
    if (ue_addr == InetSocketAddress::ConvertFrom (ip_addr).GetIpv4 ()) {
      knownSender = true;
      return (ueId);
    }
  }
  NS_ASSERT (knownSender);
  return(666); // the number of the beast
}

uint16_t
GetImsi_from_ueId(uint16_t ueId)
{
    return MakeUeTraceIds(ueId).imsi;
}

uint16_t
GetImsi_from_node(Ptr<ns3::Node> ue_node)
{
    Ptr<NrUeNetDevice> ueDev = ue_node->GetDevice(0)->GetObject<NrUeNetDevice>();
    NS_ABORT_IF(ueDev == nullptr);
    return ueDev->GetImsi();
}

uint16_t
GetCellId_from_ueId(uint16_t ueId)
{
    return MakeUeTraceIds(ueId).cellId;
}

uint16_t
GetCellId_from_ueNode(Ptr<ns3::Node> ue_node)
{
    Ptr<NrUeNetDevice> ueDev = ue_node->GetDevice(0)->GetObject<NrUeNetDevice>();
    NS_ABORT_IF(ueDev == nullptr);
    Ptr<NrUeRrc> rrc = ueDev->GetRrc();
    return rrc ? rrc->GetCellId() : 0;
}
    
    
/**********************************
 * Connection Events 
 **********************************/
    
void
NotifyConnectionEstablishedUe (std::string context, uint64_t imsi,
                               uint16_t cellid, uint16_t rnti)
{
  std::cout << "ConnectionEstablished at "
            << " UE IMSI " << imsi
            << " to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context, uint64_t imsi,
                                uint16_t cellid, uint16_t rnti)
{
  std::cout << "ConnectionEstablished at "
            << " gNB CellId " << cellid
            << " with UE IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}
 

    
    
    
    
/***************************
 * Trace callbacks 
 ***************************/

void 
LogPosition(Ptr<OutputStreamWrapper> stream)
{
    for (uint32_t ueId = 0; ueId < ueNodes.GetN(); ++ueId)
    {
        Ptr<Node> ueNode = ueNodes.Get(ueId);
        Ptr<MobilityModel> mobModel = ueNode->GetObject<MobilityModel>();
        Vector pos = mobModel->GetPosition();
        Vector vel = mobModel->GetVelocity();
        const auto ids = MakeUeTraceIds(ueId);

        *stream->GetStream() << Simulator::Now().GetMicroSeconds() << "\t" << ids.ueId << "\t"
                             << ids.imsi << "\t" << ids.cellId << "\t" << pos.x << "\t" << pos.y
                             << "\t" << pos.z << "\t" << vel.x << "\t" << vel.y << "\t" << vel.z
                             << std::endl;
    }
    Simulator::Schedule(MilliSeconds(500), &LogPosition, stream);
}


    
    
// Trace Callback for UdpServer used by the delay measurement app
// since they both use UdpServers
void
udpServerTrace(std::pair<uint16_t, uint16_t> DelayPortNums,
               const Ptr<Node>& remoteHost,
               std::string context,
               Ptr<const Packet> packet,
               const Address& from,
               const Address& localAddress)
{
    const auto port = InetSocketAddress::ConvertFrom(localAddress).GetPort();
    if (port == DelayPortNums.first || port == DelayPortNums.second)
    {
        delayTrace(delayStream, remoteHost, context, packet, from, localAddress);
    }
} 
    
// This includes both the UL and DL delay trace callbacks 
void
delayTrace(Ptr<OutputStreamWrapper> stream,
           const Ptr<Node>& remoteHost,
           std::string context,
           Ptr<const Packet> packet,
           const Address& from,
           const Address& localAddress)
{
    const uint16_t receiverNodeId = GetNodeIdFromContext(context);
    const uint16_t remoteHostId = remoteHost->GetId();

    Ptr<Packet> packetCopy = packet->Copy();
    SeqTsHeader seqTs;
    if (!packetCopy->PeekHeader(seqTs))
    {
        return;
    }
    packetCopy->RemoveHeader(seqTs);

    std::string direction;
    uint16_t ueId = 0;
    if (receiverNodeId == remoteHostId)
    {
        direction = "UL";
        ueId = GetUeNodeIdFromIpAddr(from, &ueNodes, &ueIpIfaces);
    }
    else
    {
        direction = "DL";
        ueId = GetUeIdFromNodeId(receiverNodeId);
    }

    if (!InetSocketAddress::IsMatchingType(from))
    {
        return;
    }

    const auto ids = MakeUeTraceIds(ueId);
    *stream->GetStream() << Simulator::Now().GetMicroSeconds() << "\t" << direction << "\t"
                         << ids.ueId << "\t" << ids.imsi << "\t" << ids.cellId << "\t"
                         << packetCopy->GetSize() << "\t" << seqTs.GetSeq() << "\t"
                         << packetCopy->GetUid() << "\t" << seqTs.GetTs().GetMicroSeconds() << "\t"
                         << (Simulator::Now() - seqTs.GetTs()).GetMicroSeconds() << std::endl;
}

void
rttTrace(Ptr<OutputStreamWrapper> stream,
         std::string context,
         Ptr<const Packet> packet,
         const Address& from,
         const Address& localAddress)
{
    Ptr<Packet> packetCopy = packet->Copy();
    SeqTsHeader seqTs;
    if (!packetCopy->PeekHeader(seqTs))
    {
        return;
    }
    packetCopy->RemoveHeader(seqTs);
    const auto ids = MakeUeTraceIdsFromContext(context);
    if (!InetSocketAddress::IsMatchingType(from))
    {
        return;
    }
    *stream->GetStream() << Simulator::Now().GetMicroSeconds() << "\t" << ids.ueId << "\t"
                         << ids.imsi << "\t" << ids.cellId << "\t" << packetCopy->GetSize() << "\t"
                         << seqTs.GetSeq() << "\t" << packetCopy->GetUid() << "\t"
                         << seqTs.GetTs().GetMicroSeconds() << "\t"
                         << (Simulator::Now() - seqTs.GetTs()).GetMicroSeconds() << std::endl;
}

void
StampEchoClientPacket(uint16_t ueId,
                      Ptr<const Packet> packet,
                      const Address& local,
                      const Address& remote)
{
    SeqTsHeader header;
    uint32_t& nextSeq = g_rttNextSeqPerUe[ueId];
    header.SetSeq(nextSeq++);
    auto rawPacket = const_cast<Packet*>(PeekPointer(packet));
    rawPacket->AddHeader(header);
}

void
BurstRx (Ptr<OutputStreamWrapper> stream, std::string context,
         Ptr<const Packet> burst, const Address &from, const Address &to,
         const SeqTsSizeFragHeader &header)
{
    uint16_t ueId = 0;
    const uint16_t nodeId = GetNodeIdFromContext(context);
    if (nodeId >= gnbNodes.GetN () && nodeId < (gnbNodes.GetN () + ueNodes.GetN ()))
    {
        // Burst sink installed directly on a UE (downlink VR traffic)
        ueId = GetUeIdFromNodeId(nodeId);
    }
    else
    {
        // Burst sink installed elsewhere (e.g., remote host) so the packet came from a UE
        if (!InetSocketAddress::IsMatchingType(from))
        {
            return;
        }
        ueId = GetUeNodeIdFromIpAddr(from, &ueNodes, &ueIpIfaces);
    }
    const auto ids = MakeUeTraceIds(ueId);
    Time now = Simulator::Now ();
    *stream->GetStream()
        << now.GetMicroSeconds () //tstamp_us
        << "\t" << ids.ueId 
        << "\t" << ids.imsi
        << "\t" << ids.cellId
        << "\t" << header.GetSeq () // burst seqnum
        << "\t" << header.GetSize () //burst size 
        << "\t" << header.GetFrags () // total num of fragments in this burst
        << std::endl;
}    

void
FragmentRx (Ptr<OutputStreamWrapper> stream, std::string context,
            Ptr<const Packet> fragment, const Address &from, const Address &to,
         const SeqTsSizeFragHeader &header)
{
    uint16_t ueId = 0;
    const uint16_t nodeId = GetNodeIdFromContext(context);
    if (nodeId >= gnbNodes.GetN () && nodeId < (gnbNodes.GetN () + ueNodes.GetN ()))
    {
        ueId = GetUeIdFromNodeId(nodeId);
    }
    else
    {
        if (!InetSocketAddress::IsMatchingType(from))
        {
            return;
        }
        ueId = GetUeNodeIdFromIpAddr(from, &ueNodes, &ueIpIfaces);
    }
    const auto ids = MakeUeTraceIds(ueId);
    Time now = Simulator::Now ();
    *stream->GetStream()
        << now.GetMicroSeconds () //tstamp_us
        << "\t" << ids.ueId 
        << "\t" << ids.imsi
        << "\t" << ids.cellId
        << "\t" << header.GetSeq () // burst seq num
        << "\t" << header.GetSize () // burst size
        << "\t" << header.GetFrags () // total num of fragments in this burst 
        << "\t" << header.GetFragSeq () // fragment seq num
        << "\t" << header.GetTs().GetMicroSeconds () // Tx time of the fragment     
        << "\t" << (now - header.GetTs ()).GetMicroSeconds () // delay to receive this fragment
        // NOTE: You cannnot sum fragment delays to get burst delay since 
        // many fragments are not sent one after the other and instead in a burst, 
        // so many fragments could be scheduled together  
        << std::endl;
}    
    
/***********************************************
 * Install client applications
 **********************************************/

std::pair<ApplicationContainer, Time>
InstallUdpEchoApps (const Ptr<Node> &ue,
             UdpEchoClientHelper *echoClient, Time appStartTime,
             const Ptr<UniformRandomVariable> &x,
             Time appGenerationTime)
{
  ApplicationContainer app;
  app = echoClient->Install (ue);
  double start = x->GetValue (appStartTime.GetMilliSeconds (),
                              (appStartTime + appStartWindow).GetMilliSeconds ());
  Time startTime = MilliSeconds (start);
  app.Start (startTime);
  app.Stop (startTime + appGenerationTime);
  return std::make_pair (app, startTime);
}

std::pair<ApplicationContainer, Time>
InstallUlDelayTrafficApps (const Ptr<Node> &ue,
             UdpClientHelper *ulDelayClient,
             const Ptr<Node> &remoteHost,
             const Ipv4Address &remoteHostAddr, uint16_t remotePort, Time appStartTime,
             const Ptr<UniformRandomVariable> &x,
             Time appGenerationTime)
{
  ApplicationContainer app;
  ulDelayClient->SetAttribute ("Remote", AddressValue (InetSocketAddress (remoteHostAddr, remotePort)));
  app = ulDelayClient->Install (ue);

  double start = x->GetValue (appStartTime.GetMilliSeconds (),
                              (appStartTime + appStartWindow).GetMilliSeconds ());
  Time startTime = MilliSeconds (start);
  app.Start (startTime);
  app.Stop (startTime + appGenerationTime);
  return std::make_pair (app, startTime);
}

std::pair<ApplicationContainer, Time>
InstallDlDelayTrafficApps (const Ptr<Node> &ue,
             const Ipv4Address &ueAddress,
             UdpClientHelper *dlDelayClient,
             const Ptr<Node> &remoteHost,
             uint16_t remotePort, Time appStartTime,
             const Ptr<UniformRandomVariable> &x,
             Time appGenerationTime)
{
  ApplicationContainer app;
  dlDelayClient->SetAttribute ("Remote", AddressValue (InetSocketAddress (ueAddress, remotePort)));
  app = dlDelayClient->Install (remoteHost);

  double start = x->GetValue (appStartTime.GetMilliSeconds (),
                              (appStartTime + appStartWindow).GetMilliSeconds ());
  Time startTime = MilliSeconds (start);
  app.Start (startTime);
  app.Stop (startTime + appGenerationTime);
  return std::make_pair (app, startTime);
}
  
 /***********************************************
 * Create trace files and write column names
 **********************************************/
void CreateTraceFiles (void)
{
    // Position and velocity trace
    mobStream = traceHelper.CreateFileStream ("mobility_trace.txt");
    *mobStream->GetStream() 
          << "tstamp_us\t" << "ueId\t" << "IMSI\t" << "cellId\t"
          << "pos_x\t" << "pos_y\t" << "pos_z\t"
          << "vel_x\t" << "vel_y\t" << "vel_z" <<std::endl;
  
    simInfoStream = traceHelper.CreateFileStream ("sim_info.txt"); 
    
    if(global_params.traceDelay)
    {
        delayStream = traceHelper.CreateFileStream ("delay_trace.txt");
        *delayStream->GetStream()
              << "tstamp_us\t" << "dir\t" << "ueId\t" << "IMSI\t" << "cellId\t"
              << "pktSize\t" << "seqNum\t" << "pktUid\t" << "txTstamp_us\t" << "delay" << std::endl;
    }
    if(global_params.traceRtt)
    {
        rttStream = traceHelper.CreateFileStream ("rtt_trace.txt");
        *rttStream->GetStream()
              << "tstamp_us\t" << "ueId\t" << "IMSI\t" << "cellId\t"
              << "pktSize\t" << "seqNum\t" << "pktUid\t" << "txTstamp_us\t" << "delay" << std::endl;
    }
    if(global_params.traceVr)
    {
        fragmentRxStream = traceHelper.CreateFileStream ("vrFragment_trace.txt");
        *fragmentRxStream->GetStream()
              << "tstamp_us\t" << "ueId\t" << "IMSI\t" << "cellId\t"
              << "burstSeqNum\t" << "burstSize\t" << "numFragsInBurst\t" << "fragSeqNum\t" << "txTstamp_us\t" << "delay" << std::endl;  
        burstRxStream = traceHelper.CreateFileStream ("vrBurst_trace.txt");
        *burstRxStream->GetStream()
              << "tstamp_us\t" << "ueId\t" << "IMSI\t" << "cellId\t"
              << "burstSeqNum\t" << "burstSize\t" << "numFragsInBurst" << std::endl;
    }

}    
    
// Print the scenario parameters into a file for the parsing and visualisation scripts to use 
void PrintSimInfoToFile()
{
    std::cout << "Inside PrintSimInfoToFile function that prints to sim_info.txt file" << std::endl;
    *simInfoStream->GetStream() << "parameter,value\n";
    *simInfoStream->GetStream() << "gnb_count," << gnbNodes.GetN() << "\n";
    *simInfoStream->GetStream() << "ue_count," << ueNodes.GetN() << "\n";
    *simInfoStream->GetStream()
        << "simulation_time_seconds," << global_params.appGenerationTime.GetSeconds() << "\n";
    *simInfoStream->GetStream() << "rand_seed," << global_params.randSeed << "\n";
    *simInfoStream->GetStream()
        << "delay_app_installed," << (global_params.traceDelay ? 1 : 0) << "\n";
    if (global_params.traceDelay)
    {
        *simInfoStream->GetStream() << "delay_pkt_interval_seconds,"
                                    << global_params.delayInterval.As(Time::S) << "\n";
    }
    *simInfoStream->GetStream() << "rtt_app_installed," << (global_params.traceRtt ? 1 : 0) << "\n";
    *simInfoStream->GetStream() << "vr_app_installed," << (global_params.traceVr ? 1 : 0) << "\n";
    std::cout << "Exiting PrintSimInfoToFile function that prints to sim_info.txt file" << std::endl;
}

std::ostream&
operator<< (std::ostream& os, const Parameters& parameters)
{
    os << "Simulation parameters:\n"
       << "  numUes: " << parameters.numUes << "\n"
       << "  numUesWithVrApp: " << parameters.numUesWithVrApp << "\n"
       << "  centralFrequencyHz: " << parameters.centralFrequencyBand << "\n"
       << "  bandwidthHz: " << parameters.bandwidthHz << "\n"
       << "  numerology: " << parameters.numerologyBwp1 << "\n"
       << "  tddPattern: " << parameters.tddPattern << "\n"
       << "  BsTxPower: " << parameters.BsTxPower << " dBm\n"
       << "  traceDelay: " << parameters.traceDelay << "\n"
       << "  traceRtt: " << parameters.traceRtt << "\n"
       << "  traceVr: " << parameters.traceVr << "\n";
    return os;
}

#endif // CELLULAR_NETWORK_IMPLEMENTATION

} // namespace ns3

#endif // CELLULAR_NETWORK_FUNCTION_H
