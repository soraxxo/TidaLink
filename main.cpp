/* Copyright 2016, ableton AG, Berlin. All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  If you would like to incorporate Link into a proprietary software application,
 *  please contact <link-devs@ableton.com>.
 */

#include <ableton/Link.hpp>
#include <ableton/link/HostTimeFilter.hpp>
#include "oscpack/osc/OscOutboundPacketStream.h"
#include "oscpack/osc/OscReceivedElements.h"
#include "oscpack/osc/OscPrintReceivedElements.h"
#include "dirtyudp.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#if defined(LINK_PLATFORM_UNIX)
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#endif

// #define NTP_UT_EPOCH_DIFF ((70 * 365 + 17) * 24 * 60 * 60)
int OUTPUT_BUFFER_SIZE = 1024;  // change style of variable name?

// referencing this to make sure everything is working properly
osc::OutboundPacketStream* stream;

/*
class UdpBroadcastSocket : public UdpSocket{
public:
	UdpBroadcastSocket( const IpEndpointName& remoteEndpoint ) {
	  SetEnableBroadcast(true);
	  Connect( remoteEndpoint );
	}
};
*/
UdpSender* sender;
UdpReceiver* receiver;

void printHelp()
{
  std::cout << std::endl << " < T I D A L I N K >" << std::endl << std::endl;
  std::cout << "usage:" << std::endl;
  // Spacebar does not appear to be implemented
  // std::cout << "  start / stop: space" << std::endl;
  std::cout << "  decrease / increase tempo: w / e" << std::endl;
  std::cout << "  decrease / increase quantum: r / t" << std::endl;
  std::cout << "  quit: q" << std::endl << std::endl;
}

void clear() {
    // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
    //std::cout << "\x1B[2J\x1B[H";
    // \033c cleans the screen -- seems to be working better
    std::cout << "\033c";
}

void clearLine()
{
  std::cout << "   \r" << std::flush;
  std::cout.fill(' ');
}

void sendTempo(ableton::Link& link, double quantum, double latency, long int beatOffset) {
  auto timeline = link.captureAppTimeline();
  const auto time = link.clock().micros();
  const auto tempo = timeline.tempo();
  const auto beats = (timeline.beatAtTime(time, quantum) - beatOffset);
  const auto phase = timeline.phaseAtTime(time, quantum);
  const auto cycle = beats / quantum;
  const double cps = (timeline.tempo() / quantum) / 60;
  const auto t     = std::chrono::microseconds(time).count();
  static long diff = 0;
  static double last_cps = -1;
  char buffer[OUTPUT_BUFFER_SIZE];
  if (diff == 0) {
    unsigned long milliseconds_since_epoch = 
      std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
    // POSIX is millis and Link is micros.. Not sure if that `+500` helps
    diff = ((milliseconds_since_epoch*1000 + 500) - t);
  }
  double timetag_ut = ((double) (t + diff)) / ((double) 1000000);
  timetag_ut -= latency;
  int sec = floor(timetag_ut);
  int usec = floor(1000000 * (timetag_ut - sec));
  osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
  clear();
  printHelp();
  std::cout << "current cps: " << cps << " | last cps: " << last_cps << "\n";
  last_cps = cps;
  p << osc::BeginMessage( "/tempo" )
    << sec << usec
    << (float) cycle << (float) cps << "True" << osc::EndMessage;
  //s.Send( p.Data(), p.Size() );
  sender->Send((char *)p.Data(), p.Size());
}


struct State
{
  std::atomic<bool> running;
  ableton::Link link;
  double quantum;
  double latency = 0.145;  //originally 0.4
  long int beatOffset = 0;
  State()
    : running(true)
      , link(120.)
  {
    link.enable(true);
    link.setTempoCallback([this](const double bpm) {
	sendTempo(std::ref(link), quantum, latency, beatOffset);
    });

    quantum=4;
  }
};

void disableBufferedInput()
{
#if defined(LINK_PLATFORM_UNIX)
  termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
#endif
}

void enableBufferedInput()
{
#if defined(LINK_PLATFORM_UNIX)
  termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag |= ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
#endif
}

void printState(const std::chrono::microseconds time,
    const ableton::Link::Timeline timeline,
    const std::size_t numPeers,
    const double quantum,
    State& state)
{
  const auto beats = timeline.beatAtTime(time, quantum);
  const auto phase = timeline.phaseAtTime(time, quantum);
  const auto cycle = (beats - state.beatOffset) / quantum;
  const double cps = (timeline.tempo() / quantum) / 60;
  const auto t     = std::chrono::microseconds(time).count();
  static long diff = 0;
  static double last_cps = -1;
  //const auto time = state.link.clock().micros();
  
  if (diff == 0) {
    unsigned long milliseconds_since_epoch = 
      std::chrono::duration_cast<std::chrono::milliseconds>
      (std::chrono::system_clock::now().time_since_epoch()).count();
    // POSIX is millis and Link is micros.. Not sure if that `+500` helps
    diff = ((milliseconds_since_epoch*1000 + 500) - t);
  }
  double timetag_ut = ((double) (t + diff)) / ((double) 1000000);
  timetag_ut -= state.latency;
  int sec = floor(timetag_ut);
  int usec = floor(1000000 * (timetag_ut - sec));

  clearLine();
  std::cout << std::defaultfloat << "peers: " << numPeers 
            << " | " << "quantum: " << quantum 
            << " | " << "tempo: " << timeline.tempo()
    //<< " | " << std::fixed << "beats: " << (beats - state.beatOffset)
    //<< " | sec: " << sec
    //      << " | usec: " << usec
            << " | lat: " << state.latency << " | ";
  for (int i = 0; i < ceil(quantum); ++i)
  {
    if (i < phase)
    {
      std::cout << 'X';
    }
    else
    {
      std::cout << 'O';
    }
  }
}

void input(State& state)
{
  char in;

#if defined(LINK_PLATFORM_WINDOWS)
  HANDLE stdinHandle = GetStdHandle(STD_INPUT_HANDLE);
  DWORD numCharsRead;
  INPUT_RECORD inputRecord;
  do
  {
    ReadConsoleInput(stdinHandle, &inputRecord, 1, &numCharsRead);
  } while ((inputRecord.EventType != KEY_EVENT) || inputRecord.Event.KeyEvent.bKeyDown);
  in = inputRecord.Event.KeyEvent.uChar.AsciiChar;
#elif defined(LINK_PLATFORM_UNIX)
  in = std::cin.get();
#endif
  auto timeLine = state.link.captureAppTimeline();
  const auto tempo = timeLine.tempo();

  std::chrono::microseconds updateAt = state.link.clock().micros();

  switch (in)
  {
    case 'q':
      state.running = false;
      //clearLine();
      std::cout << "\n";
      std::cout << "\n";  //remove?
      return;
    case 'w':
      //clearLine();
      if (tempo > 20) {
        timeLine.setTempo(tempo-1,updateAt);
      }
      break;
    case 'e':
      //clearLine();
      if (tempo < 999) {
        timeLine.setTempo(tempo+1,updateAt);
      }
      break;
    case 'r':
      //clearLine();
      if (state.quantum > 1) {
        state.quantum -= 1;
      }
      break;
    case 't':
      //clearLine();
      state.quantum += 1;
      break;
    // default:
  }
  state.link.commitAppTimeline(timeLine);
  input(state);
}

void oscRecvHandler(char* packet, int packetSize, void* data) {
  State* state = (State*)data;
  const auto time = state->link.clock().micros();
  auto timeline = state->link.captureAppTimeline();
  osc::ReceivedPacket* oscPacket = new osc::ReceivedPacket(packet, packetSize);
  if(oscPacket->IsMessage()) {
    osc::ReceivedMessage* message = new osc::ReceivedMessage(*oscPacket);
    if(std::strcmp(message->AddressPattern(), "/cps") == 0) {
      auto timeLine = state->link.captureAppTimeline();
      const auto tempo = timeLine.tempo();
      // --- //
      osc::ReceivedMessage::const_iterator arg = message->ArgumentsBegin();
      float cps = (arg++)->AsFloat();
      if(arg != message->ArgumentsEnd())
        throw osc::ExcessArgumentException();
      // --- //
      std::chrono::microseconds updateAt = state->link.clock().micros();
      timeLine.setTempo((double)(cps*state->quantum*60), updateAt);
      state->link.commitAppTimeline(timeLine);
    }
    if(std::strcmp(message->AddressPattern(), "/latency") == 0) {
      auto timeLine = state->link.captureAppTimeline();
      const auto tempo = timeLine.tempo();
      // --- //
      osc::ReceivedMessage::const_iterator arg = message->ArgumentsBegin();
      state->latency = (arg++)->AsFloat();
      if(arg != message->ArgumentsEnd())
        throw osc::ExcessArgumentException();
      sendTempo(std::ref(state->link), state->quantum, state->latency, state->beatOffset);
    }
    if(std::strcmp(message->AddressPattern(), "/nudge") == 0) {
      auto timeLine = state->link.captureAppTimeline();
      const auto tempo = timeLine.tempo();
      // --- //
      osc::ReceivedMessage::const_iterator arg = message->ArgumentsBegin();
      state->latency += (arg++)->AsFloat();
      if(arg != message->ArgumentsEnd())
        throw osc::ExcessArgumentException();
      sendTempo(std::ref(state->link), state->quantum, state->latency, state->beatOffset);
      std::cout << "\n\n" << "got nudge, latency now set to " << state->latency << "\n\n";
    }
    if(std::strcmp(message->AddressPattern(), "/resetbeat") == 0) {
      auto timeLine = state->link.captureAppTimeline();
      const auto tempo = timeLine.tempo();
      auto beats = timeLine.beatAtTime(time, state->quantum);
      
      osc::ReceivedMessage::const_iterator arg = message->ArgumentsBegin();
      state->beatOffset = floor(beats+0.5);
      state->beatOffset -= (arg++)->AsInt32();
      if(arg != message->ArgumentsEnd())
        throw osc::ExcessArgumentException();
      sendTempo(std::ref(state->link), state->quantum, state->latency, state->beatOffset);
      std::cout << "\n\ngot resetbeat\n\n";
    }
    if(std::strcmp(message->AddressPattern(), "/ping") == 0) {
      auto timeLine = state->link.captureAppTimeline();
      const auto tempo = timeLine.tempo();
      // --- //
      sendTempo(std::ref(state->link), state->quantum, state->latency, state->beatOffset);
    }
  }
}

void oscRecvThreadFunc(State& state) {
  receiver->Loop(oscRecvHandler, (void*)(&state));
  oscRecvThreadFunc(state);
}

int main(int argc, char **argv)
{
  // #TODO add check for flags
  // #TODO add case for output buffer size flag
  // #TODO add case for latency flag
  sender = new UdpSender("127.0.0.1", 6043, OUTPUT_BUFFER_SIZE);
  receiver = new UdpReceiver(6042, OUTPUT_BUFFER_SIZE);
  State state;
  clear();
  printHelp();
  std::thread thread(input, std::ref(state));
  std::thread oscRecvThread(oscRecvThreadFunc, std::ref(state));
  disableBufferedInput();

  while (state.running)
  {
    const auto time = state.link.clock().micros();
    auto timeline = state.link.captureAppTimeline();
    printState(time, timeline, state.link.numPeers(), state.quantum, state);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  enableBufferedInput();
  thread.join();
  return 0;
}

// ------------------------------------------ //
// --- DIRTYUDP OSCPACK INTEGRATION TESTS --- //
// ------------------------------------------ //
#define BUFFERSIZE 4096
void udpHandler(char* packet, int packetSize, void* data) {
  std::cout << osc::ReceivedPacket(packet, packetSize);
}
int main_udprcv(int argc, char** argv) {
  UdpReceiver* receiver = new UdpReceiver(7000, BUFFERSIZE);
  while(1) receiver->Loop(udpHandler, NULL);
}
char buffer [BUFFERSIZE];
int main_udptx(int argc, char** argv) {
  UdpSender* sender = new UdpSender("127.0.0.1", 7000, BUFFERSIZE);
  osc::OutboundPacketStream p(buffer, BUFFERSIZE);
  p << osc::BeginBundleImmediate
    << osc::BeginMessage( "/test1" )
    << true << 23 << (float)3.1415 << "hello" << osc::EndMessage
    << osc::BeginMessage( "/test2" )
    << true << 24 << (float)10.8 << "world" << osc::EndMessage
    << osc::EndBundle;
  sender->Send((char *)p.Data(), p.Size());
}
