#include "ns3/mmwave-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/command-line.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"


using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE("HEVC-UDP-Transit");

uint32_t g_rxPackets; // total number of received packets
uint32_t g_txPackets; // total number of transmitted packets
static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  g_rxPackets++;
  SeqTsHeader header;
  p->PeekHeader(header);
  *stream->GetStream () << header.GetSeq() << "," << "udp" << "," << p->GetSize() << "," << Simulator::Now().GetMilliSeconds() << std::endl;
}

static void Tx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  g_txPackets++;
  SeqTsHeader header;
  p->PeekHeader(header);
  *stream->GetStream () << header.GetSeq() <<"," << "udp" << "," << p->GetSize() << "," << Simulator::Now().GetMilliSeconds() << std::endl;
}



int main(int argc, char *argv[])
{
    uint16_t numEnb = 1;
    uint16_t numUe = 1;
    double simTime = 4.7;
    double interPacketInterval = 0;
    double DistancetoCellTower = 200.0; // eNB-UE distance in meters
    bool harqEnabled = true;
    bool rlcAmEnabled = false;


    CommandLine cmd;
    cmd.AddValue ("numEnb", "Number of eNBs", numEnb);
    cmd.AddValue ("numUe", "Number of UEs per eNB", numUe);
    cmd.AddValue ("simTime", "Total duration of the simulation [s])", simTime);
    cmd.AddValue ("interPacketInterval", "Inter-packet interval [us])", interPacketInterval);
    cmd.AddValue ("harq", "Enable Hybrid ARQ", harqEnabled);
    cmd.AddValue ("rlcAm", "Enable RLC-AM", rlcAmEnabled);
    cmd.Parse (argc, argv);
    LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
    LogComponentEnable("HEVC-UDP-Transit", LOG_LEVEL_INFO);

    Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
    Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
    Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
    Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
    Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));

    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
    mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");
    Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
    mmwaveHelper->SetEpcHelper (epcHelper);
    mmwaveHelper->SetHarqEnabled (harqEnabled);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1300));
    p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0)));

    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
    // interface 0 is localhost, 1 is the p2p device
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create (numEnb);
    ueNodes.Create (numUe);

    // Install Mobility Model
    Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
    enbPositionAlloc->Add (Vector (0.0, 0.0, 0.0));
    MobilityHelper enbmobility;
    enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    enbmobility.SetPositionAllocator (enbPositionAlloc);
    enbmobility.Install (enbNodes);

    MobilityHelper uemobility;
    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
    Ptr<UniformRandomVariable> distRv = CreateObject<UniformRandomVariable> ();
    uePositionAlloc->Add (Vector (DistancetoCellTower, 0.0, 0.0));

    uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    uemobility.SetPositionAllocator (uePositionAlloc);
    uemobility.Install (ueNodes);

    // Install mmWave Devices to the nodes
    NetDeviceContainer enbmmWaveDevs = mmwaveHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer uemmWaveDevs = mmwaveHelper->InstallUeDevice (ueNodes);
    // Install the IP stack on the UEs
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (uemmWaveDevs));
    
    
    
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get (u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
    
    
    mmwaveHelper->AttachToClosestEnb (uemmWaveDevs, enbmmWaveDevs);
    uint16_t ulPort = 4000;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    mmwaveHelper->EnableTraces ();

    UdpTraceClientHelper client_transmitter (remoteHostAddr, ulPort, "trace.txt");
    AsciiTraceHelper asciiTraceHelper_tx;
    Ptr<OutputStreamWrapper> stream_tx = asciiTraceHelper_tx.CreateFileStream ("TX-mmwave.txt");
    client_transmitter.SetAttribute ("MaxPacketSize", UintegerValue (1300));
    client_transmitter.SetAttribute ("TraceLoop",ns3::BooleanValue (false));
    clientApps.Add (client_transmitter.Install (ueNodes.Get(0)));
    clientApps.Get(0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx, stream_tx));


    UdpServerHelper server (ulPort);
    serverApps = server.Install (remoteHost);


    AsciiTraceHelper asciiTraceHelper;
    Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("RX-mmwave.txt");
    serverApps.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream));

    p2ph.EnablePcapAll("mmwave-streaming.pcap");

    serverApps.Start (Seconds (0.0));
    clientApps.Start (Seconds (0.0));
    Simulator::Stop (Seconds (simTime));
    Simulator::Run ();
    Simulator::Destroy ();

    NS_LOG_INFO("Total packets transmitted: " << g_txPackets);  
    NS_LOG_INFO("Total packets received: " << g_rxPackets);
    NS_LOG_INFO("Reception ratio: " << (g_rxPackets*1.0) /(g_txPackets*1.0));
    return 0;
}