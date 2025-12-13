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
#include <cmath>
#include <filesystem>
#include <algorithm>
// ns3 specific
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-radio-environment-map-helper.h"
#include "ns3/point-to-point-module.h"
#include <ns3/radio-environment-map-helper.h>
#include <iomanip>
#include "ns3/log.h"
// ns3 VR app
#include "ns3/seq-ts-size-frag-header.h"
#include "ns3/bursty-helper.h"
#include "ns3/burst-sink-helper.h"
#include "ns3/trace-file-burst-generator.h"

#define CELLULAR_NETWORK_IMPLEMENTATION
#include "cellular-network.h"
#undef CELLULAR_NETWORK_IMPLEMENTATION

NS_LOG_COMPONENT_DEFINE ("CellularNetwork");



namespace ns3 {

    
// Call this function with params containing all the parameters to setup the simulation
void CellularNetwork(const Parameters& params)
{
    
    
    /****************************************************
    *                   Startup things
    *****************************************************/
 
    // Validate the parameter settings  
    params.Validate ();

    // Set random seeds and initialize random stream
    RngSeedManager::SetSeed (params.randSeed+1); 
    RngSeedManager::SetRun (params.randSeed);
    int64_t randomStream = 1;
    
    // Keep a copy for helper routines that still read global parameters/traces
    global_params = params;

    // Set buffer sizes
    Config::SetDefault ("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue (params.rlcTxBuffSize)); 
    Config::SetDefault ("ns3::NrRlcAm::MaxTxBufferSize", UintegerValue (params.rlcTxBuffSize)); 
    Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (params.tcpUdpBuffSize));
    Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (params.tcpUdpBuffSize));
    Config::SetDefault ("ns3::UdpSocket::RcvBufSize", UintegerValue (params.tcpUdpBuffSize));
    
    
    // Create user created trace files with corresponding column names
    CreateTraceFiles ();
    
    uint16_t vrTraceFileIndex = 0;
    
    /****************************************************
    * UE and gNodeB creation
    *****************************************************/

    gnbNodes = NodeContainer();
    gnbNodes.Create(1);

    ueNodes = NodeContainer();
    ueNodes.Create(params.numUes);


    /*********************************************************
    * Position Bounding box and Mobility model
    **********************************************************/ 
    
    // gNB is fixed
    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    gnbPos->Add(Vector(0.0, 0.0, params.BsHeight));
    MobilityHelper gnbMobility;
    gnbMobility.SetPositionAllocator(gnbPos);
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);

    // UE randomness with SteadyStateRandomWaypoint: start uniformly in the same bounding box
    Ptr<RandomBoxPositionAllocator> uePos = CreateObject<RandomBoxPositionAllocator>();
    Ptr<UniformRandomVariable> xPos = CreateObject<UniformRandomVariable>();
    xPos->SetAttribute("Min", DoubleValue(params.boundingBoxMinX));
    xPos->SetAttribute("Max", DoubleValue(params.boundingBoxMaxX));
    uePos->SetAttribute("X", PointerValue(xPos));
    Ptr<UniformRandomVariable> yPos = CreateObject<UniformRandomVariable>();
    yPos->SetAttribute("Min", DoubleValue(params.boundingBoxMinY));
    yPos->SetAttribute("Max", DoubleValue(params.boundingBoxMaxY));
    uePos->SetAttribute("Y", PointerValue(yPos));
    Ptr<ConstantRandomVariable> zPos = CreateObject<ConstantRandomVariable>();
    zPos->SetAttribute("Constant", DoubleValue(1.5));
    uePos->SetAttribute("Z", PointerValue(zPos));
    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator(uePos);

    // Configure SteadyStateRandomWaypoint defaults (bounding box)
    Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MinX", DoubleValue(params.boundingBoxMinX));
    Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MaxX", DoubleValue(params.boundingBoxMaxX));
    Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MinY", DoubleValue(params.boundingBoxMinY));
    Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MaxY", DoubleValue(params.boundingBoxMaxY));
    Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::Z", DoubleValue(1.5));

    // Install the model; adjust Min/MaxSpeed as desired
    ueMobility.SetMobilityModel("ns3::SteadyStateRandomWaypointMobilityModel",
                                "MinSpeed", DoubleValue(params.ueMinSpeed),
                                "MaxSpeed", DoubleValue(params.ueMaxSpeed));
    ueMobility.Install(ueNodes);
    
    /***********************************************
    *             5G NR RAN settings
    **********************************************/
    
    // Set some defaults
    Config::SetDefault ("ns3::NrGnbMac::NumberOfRaPreambles", UintegerValue (params.NumberOfRaPreambles));
    Config::SetDefault ("ns3::NrHelper::UseIdealRrc", BooleanValue (params.UseIdealRrc)); // To prevent errors in control channel

     /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - nrEpcHelper, which will setup the core network
     * - RealisticBeamformingHelper, which takes care of the beamforming part
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
     * - NrChannelHelper, which takes care of the spectrum channel
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<RealisticBeamformingHelper> beamformingHelper = CreateObject<RealisticBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetGnbBeamManagerTypeId(RealisticBfManager::GetTypeId());
    nrHelper->SetGnbBeamManagerAttribute("TriggerEvent",
                                         EnumValue(RealisticBfManager::SRS_COUNT));
    nrHelper->SetUePhyAttribute("EnableUplinkPowerControl",
                                BooleanValue(params.enableUlPc));
    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaPF::GetTypeId());
    nrHelper->SetEpcHelper(nrEpcHelper);

    /*
     * Spectrum division. We create one operational band that contains a single component
     * carrier and a single bandwidth part centered at the frequency specified by the
     * input parameters.
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    // Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates a single BWP per CC
    CcBwpCreator::SimpleOperationBandConf bandConf1(params.centralFrequencyBand,
                                                    params.bandwidthHz,
                                                    numCcPerBand);

    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf1);

    /**
     * The channel is configured by this helper using a combination of the scenario, the channel
     * condition model, and the fading model.
     */

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("InH-OfficeOpen", "Default", "ThreeGpp"); // factory-like indoor scenario
    /**
     * Use channelHelper API to define the attributes for the channel model (condition, pathloss and
     * spectrum)
     */
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

    /*
     * allBwps contains all the spectrum configuration needed for the nrHelper.
     *
     * Now, we can setup the attributes. We can have three kind of attributes:
     * (i) parameters that are valid for all the bandwidth parts and applies to
     * all nodes, (ii) parameters that are valid for all the bandwidth parts
     * and applies to some node only, and (iii) parameters that are different for
     * every bandwidth parts. The approach is:
     *
     * - for (i): Configure the attribute through the helper, and then install;
     * - for (ii): Configure the attribute through the helper, and then install
     * for the first set of nodes. Then, change the attribute through the helper,
     * and install again;
     * - for (iii): Install, and then configure the attributes by retrieving
     * the pointer needed, and calling "SetAttribute" on top of such pointer.
     *
     */

    /*
     *  Case (i): Attributes valid for all the nodes
     */
    // Beamforming method for industrial scenario: channel-measurement-driven realistic algorithm
    beamformingHelper->SetBeamformingMethod(RealisticBeamformingAlgorithm::GetTypeId());

    // Core latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    const uint32_t bwpIdForLowLat = 0;
    const uint32_t bwpIdForRegular = 0;

    // gNb routing between Bearer and bandwidh part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",
                                                 UintegerValue(bwpIdForLowLat));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForRegular));

    // Ue routing between Bearer and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForLowLat));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpIdForRegular));

    /*
     * We miss many other parameters. By default, not configuring them is equivalent
     * to use the default values. Please, have a look at the documentation to see
     * what are the default values for all the attributes you are not seeing here.
     */

    /*
     * Case (ii): Attributes valid for a subset of the nodes
     */

    // NOT PRESENT IN THIS SIMPLE EXAMPLE

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    const uint16_t echoPortNum = 9;
    const uint16_t ulDelayPortNum = 17000;
    const uint16_t dlDelayPortNum = 18000;
    const uint16_t vrPortNum = 16000;
    const uint32_t echoPacketCount = 0xFFFFFFFF;
    const uint32_t delayPacketCount = 0xFFFFFFFF;
    NrEpsBearer vrBearer(static_cast<NrEpsBearer::Qci>(params.vrBearerQci));
    Ptr<NrEpcTft> vrTft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter vrPf;
    vrPf.direction = NrEpcTft::UPLINK;
    vrPf.localPortStart = vrPortNum;
    vrPf.localPortEnd = vrPortNum;
    vrTft->Add(vrPf);

    NrEpsBearer ctrlBearer(static_cast<NrEpsBearer::Qci>(params.controlBearerQci));
    Ptr<NrEpcTft> ctrlTft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlDelayPf;
    dlDelayPf.direction = NrEpcTft::DOWNLINK;
    dlDelayPf.localPortStart = dlDelayPortNum;
    dlDelayPf.localPortEnd = dlDelayPortNum;
    ctrlTft->Add(dlDelayPf);
    NrEpcTft::PacketFilter ulDelayPf;
    ulDelayPf.direction = NrEpcTft::UPLINK;
    ulDelayPf.remotePortStart = ulDelayPortNum;
    ulDelayPf.remotePortEnd = ulDelayPortNum;
    ctrlTft->Add(ulDelayPf);
    NrEpcTft::PacketFilter rttPf;
    rttPf.direction = NrEpcTft::BIDIRECTIONAL;
    rttPf.remotePortStart = echoPortNum;
    rttPf.remotePortEnd = echoPortNum;
    ctrlTft->Add(rttPf);
    Ptr<NrRadioEnvironmentMapHelper> remHelper;

    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDevs, randomStream);
    /*
     * Case (iii): Go node for node and change the attributes we have to setup
     * per-node.
     */

    // Get the first netdevice (gnbNetDev.Get (0)) and the first bandwidth part (0)
    Ptr<NrGnbPhy> gnbPhy0 = NrHelper::GetGnbPhy(gnbNetDev.Get(0), 0);
    gnbPhy0->SetAttribute("Pattern", StringValue(params.tddPattern));
    gnbPhy0->SetAttribute("Numerology", UintegerValue(params.numerologyBwp1));
    gnbPhy0->SetAttribute("TxPower", DoubleValue(params.BsTxPower));

    // From here, it is standard NS3. In the future, we will create helpers
    // for this part as well.

    /****************************************************
    * Install Internet for all Nodes
    *****************************************************/
    auto [remoteHost, pgwAddress] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.000));
    Ptr<Ipv4> remoteHostIpv4 = remoteHost->GetObject<Ipv4>();
    Ipv4Address remoteHostAddr = remoteHostIpv4->GetAddress(1, 0).GetLocal();

    InternetStackHelper internet;
    internet.Install(ueNodes);

    ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(ueNetDevs);

    /****************************************************
    * Attach UEs to gNBs
    *****************************************************/      
    
    nrHelper->AttachToClosestGnb(ueNetDevs, gnbNetDev);

    /***********************************************
    * Traffic generation applications
    **********************************************/

    // Server Config 
    ApplicationContainer serverApps;

    // Declaration of Helpers for Sinks and Servers 
    UdpServerHelper ulDelayPacketSink (ulDelayPortNum);
    UdpServerHelper dlDelayPacketSink (dlDelayPortNum);
    UdpEchoServerHelper echoServer (echoPortNum);
    //vr  
    Ptr<UniformRandomVariable> vrStart = CreateObject<UniformRandomVariable> ();
    vrStart->SetAttribute ("Min", DoubleValue (params.vrStartTimeMin));
    vrStart->SetAttribute ("Max", DoubleValue (params.vrStartTimeMax));
    
    // Server Creation 
    if(params.traceDelay)
    {
        serverApps.Add (ulDelayPacketSink.Install (remoteHost)); // appId updated on remoteHost
    }
    if(params.traceRtt)
    {
        serverApps.Add (echoServer.Install (remoteHost)); // appId updated on remoteHost
    }

    //========================================================
    // Client Config 
    ApplicationContainer clientApps;

    // Declarations of Helpers for Clients
    UdpClientHelper ulDelayClient;
    UdpClientHelper dlDelayClient;
    UdpEchoClientHelper echoClient (remoteHostAddr, echoPortNum);
    //vr
    BurstSinkHelper burstSinkHelper ("ns3::UdpSocketFactory",
                                   InetSocketAddress (Ipv4Address::GetAny (), vrPortNum));
    
    // Client Config
    if(params.traceDelay)
    {
        // Configure UL and DL delay client applications 
        ulDelayClient.SetAttribute ("MaxPackets", UintegerValue (delayPacketCount));
        ulDelayClient.SetAttribute ("PacketSize", UintegerValue (params.delayPacketSize));
        ulDelayClient.SetAttribute ("Interval", TimeValue (params.delayInterval));

        dlDelayClient.SetAttribute ("MaxPackets", UintegerValue (delayPacketCount));
        dlDelayClient.SetAttribute ("PacketSize", UintegerValue (params.delayPacketSize));
        dlDelayClient.SetAttribute ("Interval", TimeValue (params.delayInterval));
    }
    if(params.traceRtt)
    {
        // Configure echo client application
        echoClient.SetAttribute ("MaxPackets", UintegerValue (echoPacketCount));
        echoClient.SetAttribute ("Interval", TimeValue (params.echoInterPacketInterval));
        echoClient.SetAttribute ("PacketSize", UintegerValue (params.echoPacketSize));
    }
    if(params.traceVr)
    {
        // Nothing to configure
    }

    // Client Creation on the desired devices
    Ptr<UniformRandomVariable> startRng = CreateObject<UniformRandomVariable> ();
    startRng->SetStream (RngSeedManager::GetRun ());

    
    const uint32_t totalUes = ueNodes.GetN();
    const uint32_t totalVrUes = params.traceVr
                                    ? std::min<uint32_t>(params.numUesWithVrApp, totalUes)
                                    : 0;
    const uint32_t vrStartIndex = (totalVrUes == 0) ? totalUes : (totalUes - totalVrUes);

    /***********************************************
    * Iterate through UEs and install apps 
    **********************************************/    
    
    for (uint32_t ueId = 0; ueId < totalUes; ++ueId)
    {
        Ptr<Node> node = ueNodes.Get (ueId);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
        Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0); 
        Ipv4Address addr = iaddr.GetLocal ();
        Ptr<NetDevice> ueDevice = ueNetDevs.Get(ueId);

        nrHelper->ActivateDedicatedEpsBearer(ueDevice, ctrlBearer, ctrlTft);
        const bool isVrUe = (totalVrUes > 0 && ueId >= vrStartIndex);
        if (isVrUe)
        {
            nrHelper->ActivateDedicatedEpsBearer(ueDevice, vrBearer, vrTft);
        }

        // Client apps
        // These are the apps that are on all devices 
        if (params.traceDelay)
        {
            serverApps.Add (dlDelayPacketSink.Install (node));  
            auto appType2 = InstallUlDelayTrafficApps (node,
                                  &ulDelayClient,
                                  remoteHost, remoteHostAddr, ulDelayPortNum, params.appStartTime,
                                  startRng, params.appGenerationTime);
            clientApps.Add (appType2.first);
            auto appType3 = InstallDlDelayTrafficApps (node, addr,
                              &dlDelayClient,
                                  remoteHost, dlDelayPortNum, params.appStartTime,
                                  startRng, params.appGenerationTime);
            clientApps.Add (appType3.first);
        }
        if(params.traceRtt)
        {
            auto appType1 = InstallUdpEchoApps (node,
                              &echoClient,
                              params.appStartTime,
                              startRng, params.appGenerationTime);
            clientApps.Add (appType1.first);
        } 
        if (isVrUe) 
        {
            // Random sample for the start time fo the VR session for each UE  
            double vrStartTime = vrStart->GetValue();
            // The sender of VR traffic to be installed on remoteHost
            std::string vrTraceFile = params.vrTraceFiles[vrTraceFileIndex];
            BurstyHelper burstyHelper ("ns3::UdpSocketFactory", 
                                       InetSocketAddress (remoteHostAddr, vrPortNum)); 
            burstyHelper.SetAttribute ("FragmentSize", UintegerValue (1200));
            burstyHelper.SetBurstGenerator ("ns3::TraceFileBurstGenerator", 
                                            "TraceFile", StringValue (params.traceFolder + vrTraceFile), 
                                            "StartTime", DoubleValue (vrStartTime));
            vrTraceFileIndex = (vrTraceFileIndex + 1)%8;
            serverApps.Add (burstyHelper.Install (node));
            std::cout << " VR trace file " << vrTraceFile << " scheduled at t="
                      << vrStartTime << "s for UE IMSI " << GetImsi_from_node(node) << std::endl;
            // The receiver of the VR traffic to be installed on remote host
            clientApps.Add (burstSinkHelper.Install (remoteHost));
            // Print the IMSI of the ues that are doing this	
            std::cout << " IMSI: " << GetImsi_from_node(node) 
                << " Ip_addr: " << addr 
                << " has VR app installed " << std::endl;
            continue;
        }
    } // end of for over UEs


    
    // Server Start  
    serverApps.Start (params.appStartTime);
    // client apps are started individually using the Install function 

    // enable the RAN traces provided by the NR module
    if (params.traces == true) 
    {
          nrHelper->EnableTraces ();
    }

    // enable packet tracing from the application layer 
    if (params.traces == true)
    {
        // appId is being used here BE CAREFUL about changing the order 
        // in which the apps get added to the server container
        if (params.traceDelay)
        {
            Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::UdpServer/RxWithAddresses", 
            MakeBoundCallback (&udpServerTrace, 
                               std::make_pair(ulDelayPortNum, dlDelayPortNum),
                               remoteHost));
        }

        // connect custom trace sinks for RRC connection establishment and handover notification
        
        Config::Connect ("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionEstablished",
                       MakeCallback (&NotifyConnectionEstablishedUe));
        Config::Connect ("/NodeList/*/DeviceList/*/NrGnbRrc/ConnectionEstablished",
                       MakeCallback (&NotifyConnectionEstablishedEnb));
        

        if(params.traceRtt)
        {
            Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::UdpEchoClient/RxWithAddresses", 
                             MakeBoundCallback (&rttTrace, rttStream));
            Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::UdpEchoClient/TxWithAddresses",
                             MakeCallback (&RttTxTrace));
        }
        if(params.traceVr)
        {
            Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::BurstSink/BurstRx", 
                             MakeBoundCallback (&BurstRx, burstRxStream));
            Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::BurstSink/FragmentRx", 
                             MakeBoundCallback (&FragmentRx, fragmentRxStream));
        }
    }

    if (params.createRemMap)
    {
        NS_ABORT_MSG_IF(gnbNetDev.GetN() == 0 || ueNetDevs.GetN() == 0,
                        "Cannot create REM without gNB and UE devices");
        remHelper = CreateObject<NrRadioEnvironmentMapHelper>();
        remHelper->SetRemMode(NrRadioEnvironmentMapHelper::COVERAGE_AREA);
        remHelper->SetMinX(params.boundingBoxMinX);
        remHelper->SetMaxX(params.boundingBoxMaxX);
        remHelper->SetMinY(params.boundingBoxMinY);
        remHelper->SetMaxY(params.boundingBoxMaxY);
        remHelper->SetResX(200);
        remHelper->SetResY(200);
        remHelper->SetZ(params.ueHeight);
        remHelper->CreateRem(gnbNetDev, ueNetDevs.Get(0), 0);
    }

    // Add some extra time for the last generated packets to be received
    PrintSimInfoToFile ();
    const Time appStopWindow = MilliSeconds (50);
    Time stopTime = params.appStartTime + appStartWindow + params.appGenerationTime + appStopWindow;
    std::cout << "\n------------------------------------------------------\n";
    std::cout << "Start Simulation ! Runtime: " << stopTime.GetSeconds() << " seconds\n";
    Simulator::Stop (stopTime);
    // schedule the periodic logging of UE positions
    Simulator::Schedule (MilliSeconds(500), &LogPosition, mobStream);
    Simulator::Run ();
    std::cout << "\n------------------------------------------------------\n"
            << "End simulation"
            << std::endl;
    Simulator::Destroy ();
}// end of Cellural-network
    

} // end of namespace ns3
