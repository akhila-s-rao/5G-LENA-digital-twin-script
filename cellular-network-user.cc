/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include <ns3/command-line.h>
#include <ns3/show-progress.h>
#include "cellular-network.h"
/*
 * QCI lookup (NrEpsBearer::Qci):
 *  1  -> GBR_CONV_VOICE              67 -> GBR_MC_VIDEO
 *  2  -> GBR_CONV_VIDEO              69 -> NGBR_MC_DELAY_SIGNAL
 *  3  -> GBR_GAMING                  70 -> NGBR_MC_DATA
 *  4  -> GBR_NON_CONV_VIDEO          71 -> GBR_LIVE_UL_71
 *  5  -> NGBR_IMS                    72 -> GBR_LIVE_UL_72
 *  6  -> NGBR_VIDEO_TCP_OPERATOR     73 -> GBR_LIVE_UL_73
 *  7  -> NGBR_VOICE_VIDEO_GAMING     74 -> GBR_LIVE_UL_74
 *  8  -> NGBR_VIDEO_TCP_PREMIUM      75 -> GBR_V2X
 *  9  -> NGBR_VIDEO_TCP_DEFAULT      76 -> GBR_LIVE_UL_76
 * 65  -> GBR_MC_PUSH_TO_TALK         79 -> NGBR_V2X
 * 66  -> GBR_NMC_PUSH_TO_TALK        80 -> NGBR_LOW_LAT_EMBB
 * 82  -> DGBR_DISCRETE_AUT_SMALL     83 -> DGBR_DISCRETE_AUT_LARGE
 * 84  -> DGBR_ITS                    85 -> DGBR_ELECTRICITY
 * 86  -> DGBR_V2X                    87 -> DGBR_INTER_SERV_87
 * 88  -> DGBR_INTER_SERV_88          89 -> DGBR_VISUAL_CONTENT_89
 * 90  -> DGBR_VISUAL_CONTENT_90
 */
using namespace ns3;

/**
 * \ingroup examples
 * \file lena-lte-comparison-user.cc
 * \brief A multi-cell network deployment with site sectorization
 *
 * This example describes how to setup a simulation using the 3GPP channel model
 * from TR 38.900. This example consists of an hexagonal grid deployment
 * consisting on a central site and a number of outer rings of sites around this
 * central site. Each site is sectorized, meaning that a number of three antenna
 * arrays or panels are deployed per gNB. These three antennas are pointing to
 * 30º, 150º and 270º w.r.t. the horizontal axis. We allocate a band to each
 * sector of a site, and the bands are contiguous in frequency.
 *
 * We provide a number of simulation parameters that can be configured in the
 * command line, such as the number of UEs per cell or the number of outer rings.
 * Please have a look at the possible parameters to know what you can configure
 * through the command line.
 *
 * With the default configuration, the example will install uplink and downlink
* delay probes on each UE. Additional application mixes (VR, RTT,
 * throughput probes) can be enabled through the command-line arguments.
 *
 * \code{.unparsed}
$ ./waf --run "lena-lte-comparison-user --Help"
    \endcode
 *
 */
int
main (int argc, char *argv[])
{
    Parameters params;
    /*
    * From here, we instruct the ns3::CommandLine class of all the input parameters
    * that we may accept as input, as well as their description, and the storage
    * variable.
    */
    CommandLine cmd;

 
    cmd.AddValue("numUes", 
                "The number of UEs per cell", params.numUes);
    cmd.AddValue("numUesWithVrApp",
                 "The number of UEs that should run the VR/XR application in uplink",
                 params.numUesWithVrApp);
    cmd.AddValue ("appGenerationTime",
                "Duration applications will generate traffic.",
                params.appGenerationTime);
    cmd.AddValue("bandwidthHz",
                 "BWP bandwidth in Hz (applies to the single operational band)",
                 params.bandwidthHz);
    cmd.AddValue ("progressInterval",
                "Progress reporting interval",
                params.progressInterval);
    cmd.AddValue ("randomSeed",
                "Random seed to create repeatable or different runs",
                params.randSeed);
    cmd.AddValue("vrBearerQci",
                 "QCI value (as per NrEpsBearer::Qci) to use for VR traffic bearers",
                 params.vrBearerQci);
    cmd.AddValue("controlBearerQci",
                 "QCI value to use for delay/RTT control bearers",
                 params.controlBearerQci);
    cmd.AddValue("createRemMap",
                 "If true, generate an NR REM coverage map before the simulation starts",
                 params.createRemMap);
    // Parse the command line
    cmd.Parse (argc, argv);
    params.Validate ();

    std::cout << params;

    ShowProgress spinner (params.progressInterval);

    CellularNetwork (params);

    return 0;
}
