// Copyright 2026 Zheng Qu
// SPDX-License-Identifier: Apache-2.0

#include <cstdio>
#include <cstdlib>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/protocol2_packet_handler.h"

// Linker wrapping applies only to the copy of protocol2_packet_handler.cpp
// compiled into this test, never to the installed SDK or unrelated libraries.
static bool fail_receive_allocation = false;
extern "C" void * __real_malloc(size_t size);
extern "C" void * __wrap_malloc(size_t size)
{
  if (fail_receive_allocation && size == 1024) {
    fail_receive_allocation = false;
    return nullptr;
  }
  return __real_malloc(size);
}

class EmptyPort : public dynamixel::PortHandler
{
public:
  EmptyPort() {is_using_ = false;}
  bool openPort() override {return true;}
  void closePort() override {}
  void clearPort() override {}
  void setPortName(const char *) override {}
  char * getPortName() override {return nullptr;}
  bool setBaudRate(int) override {return true;}
  int getBaudRate() override {return 1000000;}
  int getBytesAvailable() override {return 0;}
  int readPort(uint8_t *, int) override {return 0;}
  int writePort(uint8_t *, int length) override {return length;}
  void setPacketTimeout(uint16_t) override {}
  void setPacketTimeout(double) override {}
  bool isPacketTimeout() override {return true;}
};

int main()
{
  EmptyPort port;
  auto * handler = dynamixel::Protocol2PacketHandler::getInstance();
  for (bool combined : {false, true}) {
    uint8_t data = 77, error = 88;
    if (!combined && handler->readTx(&port, 1, 132, 1) != COMM_SUCCESS)
      return 1;
    fail_receive_allocation = true;
    int result = combined ? handler->readTxRx(&port, 1, 132, 1, &data, &error) :
      handler->readRx(&port, 1, 1, &data, &error);
    if (fail_receive_allocation || result != COMM_TX_FAIL || port.is_using_ ||
        data != 77 || error != 88) {
      std::fprintf(stderr, "receive allocation failure must release the port without copying\n");
      return 1;
    }
    // No manual port reset: the very next transmission must not be PORT_BUSY.
    if (handler->readTx(&port, 1, 132, 1) != COMM_SUCCESS)
      return 1;
    if (handler->readRx(&port, 1, 1, &data, &error) != COMM_RX_TIMEOUT)
      return 1;
  }
}
