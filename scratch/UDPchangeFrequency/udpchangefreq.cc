#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/node-list.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/buildings-helper.h"
#include "ns3/buildings-module.h"
#include "ns3/global-value.h"
#include "ns3/command-line.h"
#include <ns3/random-variable-stream.h>
#include <ns3/lte-ue-net-device.h>
#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <list>

using namespace ns3;
using namespace mmwave;

NS_LOG_COMPONENT_DEFINE ("UDP-changefreq");

static ns3::GlobalValue g_mmw1DistFromMainStreet ("mmw1Dist", "Distance from the main street of the first MmWaveEnb",
                                                  ns3::UintegerValue (50), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_mmw2DistFromMainStreet ("mmw2Dist", "Distance from the main street of the second MmWaveEnb",
                                                  ns3::UintegerValue (50), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_mmw3DistFromMainStreet ("mmw3Dist", "Distance from the main street of the third MmWaveEnb",
                                                  ns3::UintegerValue (110), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_mmWaveDistance ("mmWaveDist", "Distance between MmWave eNB 1 and 2",
                                          ns3::UintegerValue (200), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_numBuildingsBetweenMmWaveEnb ("numBlocks", "Number of buildings between MmWave eNB 1 and 2",
                                                        ns3::UintegerValue (8), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_interPckInterval ("interPckInterval", "Interarrival time of UDP packets (us)",
                                            ns3::UintegerValue (20), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_bufferSize ("bufferSize", "RLC tx buffer size (MB)",
                                      ns3::UintegerValue (20), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_x2Latency ("x2Latency", "Latency on X2 interface (us)",
                                     ns3::DoubleValue (500), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_mmeLatency ("mmeLatency", "Latency on MME interface (us)",
                                      ns3::DoubleValue (10000), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_mobileUeSpeed ("mobileSpeed", "The speed of the UE (m/s)",
                                         ns3::DoubleValue (13.8), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_rlcAmEnabled ("rlcAmEnabled", "If true, use RLC AM, else use RLC UM",
                                        ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_maxXAxis ("maxXAxis", "The maximum X coordinate for the area in which to deploy the buildings",
                                    ns3::DoubleValue (150), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_maxYAxis ("maxYAxis", "The maximum Y coordinate for the area in which to deploy the buildings",
                                    ns3::DoubleValue (40), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_outPath ("outPath",
                                   "The path of output log files",
                                   ns3::StringValue ("./"), ns3::MakeStringChecker ());
static ns3::GlobalValue g_noiseAndFilter ("noiseAndFilter", "If true, use noisy SINR samples, filtered. If false, just use the SINR measure",
                                          ns3::BooleanValue (false), ns3::MakeBooleanChecker ());
static ns3::GlobalValue g_handoverMode ("handoverMode",
                                        "Handover mode",
                                        ns3::UintegerValue (3), ns3::MakeUintegerChecker<uint8_t> ());
static ns3::GlobalValue g_reportTablePeriodicity ("reportTablePeriodicity", "Periodicity of RTs",
                                                  ns3::UintegerValue (1600), ns3::MakeUintegerChecker<uint32_t> ());
static ns3::GlobalValue g_outageThreshold ("outageTh", "Outage threshold",
                                           ns3::DoubleValue (-5), ns3::MakeDoubleChecker<double> ());
static ns3::GlobalValue g_lteUplink ("lteUplink", "If true, always use LTE for uplink signalling",
                                     ns3::BooleanValue (false), ns3::MakeBooleanChecker ());

void ChangeSpeed (Ptr<Node> n, Vector speed)
{
  n->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (speed);
  NS_LOG_UNCOND ("************************--------------------Change Speed-------------------------------*****************");
}
uint32_t g_rx1Packets; // total number of received packets
uint32_t g_tx1Packets; // total number of transmitted packets
uint32_t g_rx2Packets; // total number of received packets
uint32_t g_tx2Packets; // total number of transmitted packets
static void Rx1 (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  g_rx1Packets++;
  SeqTsHeader header;
  p->PeekHeader (header);
  *stream->GetStream () << header.GetSeq () << ","
                        << "udp"
                        << "," << p->GetSize () << "," << Simulator::Now ().GetMilliSeconds ()
                        << std::endl;
}

static void Tx1 (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  g_tx1Packets++;
  SeqTsHeader header;
  p->PeekHeader (header);
  *stream->GetStream () << header.GetSeq () << ","
                        << "udp"
                        << "," << p->GetSize () << "," << Simulator::Now ().GetMilliSeconds ()
                        << std::endl;
}
static void Rx2 (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  g_rx2Packets++;
  SeqTsHeader header;
  p->PeekHeader (header);
  *stream->GetStream () << header.GetSeq () << ","
                        << "udp"
                        << "," << p->GetSize () << "," << Simulator::Now ().GetMilliSeconds ()
                        << std::endl;
}

static void Tx2 (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  g_tx2Packets++;
  SeqTsHeader header;
  p->PeekHeader (header);
  *stream->GetStream () << header.GetSeq () << ","
                        << "udp"
                        << "," << p->GetSize () << "," << Simulator::Now ().GetMilliSeconds ()
                        << std::endl;
}
int
main (int argc, char *argv[])
{
  bool harqEnabled = false;
  bool fixedTti = false;
  CommandLine cmd;
  cmd.Parse (argc, argv);

  UintegerValue uintegerValue;
  BooleanValue booleanValue;
  StringValue stringValue;
  DoubleValue doubleValue;
  LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
  LogComponentEnable("MmWaveHelper", LOG_LEVEL_INFO);

  //EnumValue enumValue;
  //   GlobalValue::GetValueByName ("numBlocks", uintegerValue);
  //   uint32_t numBlocks = uintegerValue.Get ();
  //   GlobalValue::GetValueByName ("maxXAxis", doubleValue);
  //   double maxXAxis = doubleValue.Get ();
  //   GlobalValue::GetValueByName ("maxYAxis", doubleValue);
  //   double maxYAxis = doubleValue.Get ();

  double ueInitialPosition = 0;
  double ueFinalPosition = 370;

  // Variables for the RT
  int windowForTransient = 150; // number of samples for the vector to use in the filter

  GlobalValue::GetValueByName ("reportTablePeriodicity", uintegerValue);
  int ReportTablePeriodicity = (int) uintegerValue.Get (); // in microseconds
  if (ReportTablePeriodicity == 1600)
    {
      windowForTransient = 150;
    }
  else if (ReportTablePeriodicity == 25600)
    {
      windowForTransient = 50;
    }
  else if (ReportTablePeriodicity == 12800)
    {
      windowForTransient = 100;
    }
  else
    {
      NS_ASSERT_MSG (false, "Unrecognized");
    }
  int vectorTransient = windowForTransient * ReportTablePeriodicity;
  // params for RT, filter, HO mode
  GlobalValue::GetValueByName ("noiseAndFilter", booleanValue);
  bool noiseAndFilter = booleanValue.Get ();
  GlobalValue::GetValueByName ("handoverMode", uintegerValue);
  uint8_t hoMode = uintegerValue.Get ();
  GlobalValue::GetValueByName ("outageTh", doubleValue);
  double outageTh = doubleValue.Get ();
  GlobalValue::GetValueByName ("rlcAmEnabled", booleanValue);
  bool rlcAmEnabled = booleanValue.Get ();
  GlobalValue::GetValueByName ("bufferSize", uintegerValue);
  uint32_t bufferSize = uintegerValue.Get ();
  GlobalValue::GetValueByName ("interPckInterval", uintegerValue);
  uint32_t interPacketInterval = uintegerValue.Get ();
  GlobalValue::GetValueByName ("x2Latency", doubleValue);
  double x2Latency = doubleValue.Get ();
  GlobalValue::GetValueByName ("mmeLatency", doubleValue);
  double mmeLatency = doubleValue.Get ();
  GlobalValue::GetValueByName ("mobileSpeed", doubleValue);
  double ueSpeed = doubleValue.Get ();

  double transientDuration = double (vectorTransient) / 1000000;
  double simTime =
      transientDuration + ((double) ueFinalPosition - (double) ueInitialPosition) / ueSpeed + 1;
  NS_LOG_UNCOND ("rlcAmEnabled " << rlcAmEnabled << " bufferSize " << bufferSize
                                 << " interPacketInterval " << interPacketInterval << " x2Latency "
                                 << x2Latency << " mmeLatency " << mmeLatency << " mobileSpeed "
                                 << ueSpeed);
  NS_LOG_UNCOND ("simTime " << simTime << " transientDuration " << transientDuration);
  // RLC AM (Aknowledge mode): Retransmission in case of loss of DL PDU
  // RLC UM (Unacknowledge mode): No feedback
  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  // HARQ(Hybrid Automated Repeat Request) : Retransmission in case of loss of DL PDU
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled",BooleanValue (harqEnabled));

    // Fixed slot size in the transmission TDMA
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::FixedTti", BooleanValue (fixedTti));
   // Symbole per slot in the transmission TDMA
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::SymPerSlot", UintegerValue (6));

   // Time required by the PHY layer to decode a transport block in us
  Config::SetDefault ("ns3::MmWavePhyMacCommon::TbDecodeLatency", UintegerValue (200.0));
  // Number of concurrent stop-and-wait Hybrid ARQ processes per user
  Config::SetDefault ("ns3::MmWavePhyMacCommon::NumHarqProcess", UintegerValue (100));
  // Specify the channel coherence time
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (100.0)));
  //The interval for sending system information.
  Config::SetDefault ("ns3::LteEnbRrc::SystemInformationPeriodicity",
                      TimeValue (MilliSeconds (5.0)));
  // How much to wait to issue a new Report Buffer Status since the last time a new SDU was received                    
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer",
                      TimeValue (MicroSeconds (100.0)));
    // Surrounding Reference Signal periodicity in microseconds
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
  //Time in ms of initial SIB (System Information Blocks) message
  //SIBs carry relevant information for the UE, which helps UE to access a cell,
  //perform cell re-selection, information related to INTRA-frequency, 
  //INTER-frequency, and INTER-RAT cell selections
  Config::SetDefault ("ns3::LteEnbRrc::FirstSibTime", UintegerValue (2));
  // Delay in microsecond for packet EPC network
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDelay",
                      TimeValue (MicroSeconds (x2Latency)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkDataRate",
                      DataRateValue (DataRate ("1000Gb/s")));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::X2LinkMtu", UintegerValue (10000));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1uLinkDelay",
                      TimeValue (MicroSeconds (1000)));
  Config::SetDefault ("ns3::MmWavePointToPointEpcHelper::S1apLinkDelay",
                      TimeValue (MicroSeconds (mmeLatency)));
  // Maximum Size of the Transmission Buffer (in Bytes)
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize",
                      UintegerValue (bufferSize * 1024 * 1024));
    // Value of the t-StatusProhibit timer (See section 7.3 of 3GPP TS 36.322)
  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (MilliSeconds (10.0)));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (bufferSize * 1024 * 1024));
  switch (hoMode)
    {
    case 1:
      Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode",
                          EnumValue (LteEnbRrc::THRESHOLD));
      break;
    case 2:
      Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode",
                          EnumValue (LteEnbRrc::FIXED_TTT));
      break;
    case 3:
      Config::SetDefault ("ns3::LteEnbRrc::SecondaryCellHandoverMode",
                          EnumValue (LteEnbRrc::DYNAMIC_TTT));
      break;
    }
    // Time to trigger Handover
  Config::SetDefault ("ns3::LteEnbRrc::FixedTttValue", UintegerValue (150));
  // Config Radio Ressources controls
  Config::SetDefault ("ns3::LteEnbRrc::CrtPeriod", IntegerValue (ReportTablePeriodicity));
  // Outage threshold express in SINR (dB)
  Config::SetDefault ("ns3::LteEnbRrc::OutageThreshold", DoubleValue (outageTh));
  // Update the SINR for each UE after a certain period
  Config::SetDefault ("ns3::MmWaveEnbPhy::UpdateSinrEstimatePeriod",
                      IntegerValue (ReportTablePeriodicity));
  Config::SetDefault ("ns3::MmWaveEnbPhy::Transient", IntegerValue (vectorTransient));
  // Filter the noisy SINR packets otherwise just print out the value of sinr.
  Config::SetDefault ("ns3::MmWaveEnbPhy::NoiseAndFilter", BooleanValue (noiseAndFilter));
  // set the type of RRC to use, i.e., ideal or real
  // by setting the following two attributes to true, the simulation will use
  // the ideal paradigm, meaning no packets are sent. in fact, only the callbacks are triggered
  Config::SetDefault ("ns3::MmWaveHelper::UseIdealRrc", BooleanValue (true));
  GlobalValue::GetValueByName ("lteUplink", booleanValue);
  bool lteUplink = booleanValue.Get ();
  Config::SetDefault ("ns3::McUePdcp::LteUplink", BooleanValue (lteUplink));
  NS_LOG_UNCOND ("Lte uplink " << lteUplink << "\n");

  // settings for the 3GPP the channel
  Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod",
                      TimeValue (MilliSeconds (
                          100))); // interval after which the channel for a moving user is updated,
  Config::SetDefault ("ns3::ThreeGppChannelModel::Blockage",
                      BooleanValue (true)); // use blockage or not
  Config::SetDefault ("ns3::ThreeGppChannelModel::PortraitMode",
                      BooleanValue (true)); // use blockage model with UT in portrait mode
  Config::SetDefault ("ns3::ThreeGppChannelModel::NumNonselfBlocking",
                      IntegerValue (4)); // number of non-self blocking obstacles

  // set the number of antennas in the devices
  Config::SetDefault ("ns3::McUeNetDevice::AntennaNum", UintegerValue (16));
  // set the number of antennas in the eNBs
  Config::SetDefault ("ns3::MmWaveNetDevice::AntennaNum", UintegerValue (64));

  // set to false to use the 3GPP radiation pattern (proper configuration of the bearing and downtilt angles is needed)
  Config::SetDefault ("ns3::ThreeGppAntennaArrayModel::IsotropicElements", BooleanValue (true));

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwaveHelper->SetPathlossModelType ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel");
  mmwaveHelper->SetChannelConditionModelType ("ns3::BuildingsChannelConditionModel");

  Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  mmwaveHelper->SetHarqEnabled (harqEnabled);
  mmwaveHelper->Initialize ();

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

  // parse again so you can override default values from the command line
  cmd.Parse (argc, argv);

  // Get SGW/PGW and create a single RemoteHost
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);
  // Create the Internet by connecting remoteHost to pgw. Setup routing too
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (2500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
  ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);
  
  
  // create LTE, mmWave eNB nodes and UE node
  NodeContainer ueNodes;
  NodeContainer mmWaveEnbNodes;
  NodeContainer lteEnbNodes;
  NodeContainer allEnbNodes;

  mmWaveEnbNodes.Create (2);
  lteEnbNodes.Create (1);
  ueNodes.Create (2);
  allEnbNodes.Add (lteEnbNodes);
  allEnbNodes.Add (mmWaveEnbNodes);
  // Positions
  Vector mmw1Position = Vector (50, 70, 3);
  Vector mmw2Position = Vector (150, 70, 3);

  // Install Mobility Model
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  //enbPositionAlloc->Add (Vector ((double)mmWaveDist/2 + streetWidth, mmw1Dist + 2*streetWidth, mmWaveZ));
  enbPositionAlloc->Add (mmw1Position); // LTE BS, out of area where buildings are deployed
  enbPositionAlloc->Add (mmw1Position);
  enbPositionAlloc->Add (mmw2Position);
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (allEnbNodes);
  BuildingsHelper::Install (allEnbNodes);

  MobilityHelper uemobility;
  Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
  uePositionAlloc->Add (Vector (ueInitialPosition, -5, 1.6));
  uemobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes);
  BuildingsHelper::Install (ueNodes);

  // Antena 1 mobility for MDC Description 1
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (ueInitialPosition, -5, 1.6));
  ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, 0, 0));
  // Antena 2 mobility for MDC Description 2
  ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (ueInitialPosition, 5, 1.6));
  ueNodes.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, 0, 0));
  // Install mmWave, lte, mc Devices to the nodes
  NetDeviceContainer lteEnbDevs = mmwaveHelper->InstallLteEnbDevice (lteEnbNodes);
  NetDeviceContainer mmWaveEnbDevs = mmwaveHelper->InstallEnbDevice (mmWaveEnbNodes);
  NetDeviceContainer mcUeDevs;
  mcUeDevs = mmwaveHelper->InstallMcUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (mcUeDevs));
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  // Add X2 interfaces
  mmwaveHelper->AddX2Interface (lteEnbNodes, mmWaveEnbNodes);

  // Manual attachment
  mmwaveHelper->AttachToClosestEnb (mcUeDevs, mmWaveEnbDevs, lteEnbDevs);
  uint16_t ulPort = 4000;
  ApplicationContainer serverApps;
  ApplicationContainer clientApps;
  mmwaveHelper->EnableTraces ();

  UdpTraceClientHelper client_transmitter_D1 (remoteHostAddr, ulPort, "D1trace.txt");
  UdpTraceClientHelper client_transmitter_D2 (remoteHostAddr, ulPort+1, "D2trace.txt");
  client_transmitter_D1.SetAttribute ("MaxPacketSize", UintegerValue (1450));
  client_transmitter_D1.SetAttribute ("TraceLoop", ns3::BooleanValue (false));
  client_transmitter_D2.SetAttribute ("MaxPacketSize", UintegerValue (1450));
  client_transmitter_D2.SetAttribute ("TraceLoop", ns3::BooleanValue (false));

  AsciiTraceHelper asciiTraceHelper_tx1;
  Ptr<OutputStreamWrapper> stream_tx1 = asciiTraceHelper_tx1.CreateFileStream ("TX-mmwave-1.txt");
  AsciiTraceHelper asciiTraceHelper_tx2;
  Ptr<OutputStreamWrapper> stream_tx2 = asciiTraceHelper_tx2.CreateFileStream ("TX-mmwave-2.txt");
  // Two Antena on the car
  clientApps.Add (client_transmitter_D1.Install (ueNodes.Get (0)));
  clientApps.Add (client_transmitter_D2.Install (ueNodes.Get (1)));
  clientApps.Get (0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx1, stream_tx1));
  clientApps.Get (1)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx2, stream_tx2));

  //Server D1 and D2 of MDC
  UdpServerHelper serverD1 (ulPort);
  serverApps.Add (serverD1.Install (remoteHost));
  UdpServerHelper serverD2 (ulPort + 1);
  serverApps.Add (serverD2.Install (remoteHost));


  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream_rx1 = asciiTraceHelper.CreateFileStream ("RX-mmwave-1.txt");
  serverApps.Get (0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx1, stream_rx1));
  Ptr<OutputStreamWrapper> stream_rx2 = asciiTraceHelper.CreateFileStream ("RX-mmwave-2.txt");
  serverApps.Get (0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx2, stream_rx2));

  //p2ph.EnablePcapAll ("mmwave-streaming.pcap");
  // Transient Duration, time to etablish the connection between ue and station
  serverApps.Start (Seconds (transientDuration));
  clientApps.Start (Seconds (transientDuration));
  // Simulator::Schedule (Seconds (transientDuration), &ChangeSpeed, ueNodes.Get (0), Vector (ueSpeed, 0, 0)); // start UE movement after Seconds(0.5)
  // Simulator::Schedule (Seconds (transientDuration), &ChangeSpeed, ueNodes.Get (1), Vector (ueSpeed, 0, 0)); // start UE movement after Seconds(0.5)
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_UNCOND("Total packets dropped (D1 Server): " << serverD1.GetServer()->GetLost()<<std::endl);
  NS_LOG_UNCOND("Total packets dropped (D2 Server): " << serverD2.GetServer()->GetLost()<<std::endl);
  NS_LOG_UNCOND("Total packets received (D1 Server): " << serverD1.GetServer()->GetReceived()<<std::endl);
  NS_LOG_UNCOND("Total packets received (D2 Server): " << serverD2.GetServer()->GetReceived()<<std::endl);
  NS_LOG_UNCOND("Total packets sent (D1): " << g_tx1Packets<<std::endl);
  NS_LOG_UNCOND("Total packets received (D1): " << g_rx1Packets<<std::endl);
  NS_LOG_UNCOND("Total packets sent (D2): " << g_tx2Packets<<std::endl);
  NS_LOG_UNCOND("Total packets received (D2): " << g_rx2Packets<<std::endl);
}