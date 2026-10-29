// Copyright 2026 Zheng Qu
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

using Bytes = std::vector<uint8_t>;

void require(bool condition)
{
  if (!condition) throw std::runtime_error("read reply validation failed");
}

// In-memory transport: no device or serial port is opened.
class ReplyPort : public dynamixel::PortHandler
{
public:
  Bytes reply;
  size_t offset = 0;
  ReplyPort() {is_using_ = false;}
  bool openPort() override {return true;}
  void closePort() override {}
  void clearPort() override {}
  void setPortName(const char *) override {}
  char * getPortName() override {return nullptr;}
  bool setBaudRate(int) override {return true;}
  int getBaudRate() override {return 1000000;}
  int getBytesAvailable() override {return reply.size() - offset;}
  int readPort(uint8_t * data, int length) override
  {
    int count = std::min(length, getBytesAvailable());
    std::copy_n(reply.data() + offset, count, data);
    offset += count;
    return count;
  }
  int writePort(uint8_t *, int length) override {return length;}
  void setPacketTimeout(uint16_t) override {}
  void setPacketTimeout(double) override {}
  bool isPacketTimeout() override {return offset == reply.size();}
  void load(Bytes bytes) {reply = bytes; offset = 0; is_using_ = false;}
};

void appendCRC(Bytes & bytes)
{
  uint16_t crc = 0;
  for (uint8_t byte : bytes) {
    crc ^= uint16_t(byte) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc << 1) ^ ((crc & 0x8000) ? 0x8005 : 0);
  }
  bytes.push_back(crc & 0xff);
  bytes.push_back(crc >> 8);
}

Bytes status(uint8_t id, uint8_t error, Bytes data)
{
  uint16_t length = data.size() + 4;
  Bytes bytes = {0xff, 0xff, 0xfd, 0, id, uint8_t(length), uint8_t(length >> 8), 0x55, error};
  bytes.insert(bytes.end(), data.begin(), data.end());
  appendCRC(bytes);
  return bytes;
}

Bytes aggregate(uint8_t error, bool wrong_id, bool short_data)
{
  uint16_t length = short_data ? 11 : 13;
  Bytes bytes = {0xff, 0xff, 0xfd, 0, 0xfe, uint8_t(length), 0, 0x55, 0, 1, 11, 12};
  appendCRC(bytes);
  bytes.insert(bytes.end(), {error, uint8_t(wrong_id ? 3 : 2)});
  if (!short_data) bytes.insert(bytes.end(), {21, 22});
  appendCRC(bytes);
  return bytes;
}

template<class Group>
class InspectGroup : public Group
{
public:
  using Group::Group;
  uint8_t stored(uint8_t id) {return this->data_list_[id][0];}
};

template<class Group>
void checkGroup(Group & group, ReplyPort & port, bool fast)
{
  for (int fault = 0; fault < 5; ++fault) {
    Bytes reply;
    if (fast) {
      reply = aggregate(fault == 2 ? 4 : fault == 4 ? 0x80 : 0, fault == 3, fault == 1);
    } else {
      reply = status(1, 0, {11, 12});
      Bytes second = status(fault == 3 ? 3 : 2, fault == 2 ? 4 : fault == 4 ? 0x80 : 0,
        fault == 1 ? Bytes{} : Bytes{21, 22});
      reply.insert(reply.end(), second.begin(), second.end());
    }
    port.load(reply);
    int result = group.rxPacket();
    bool valid = fault == 0 || fault == 4;
    require((result == COMM_SUCCESS) == valid);
    require(group.isAvailable(2, 132, 2) == valid);
    // Failure never overwrites data from the previous successful reply.
    require(group.stored(2) == 21);
    if (valid) {
      uint8_t error = 0;
      require(group.getError(2, &error));
      require(error == (fault == 4 ? 0x80 : 0));
    }
  }
}

int main()
{
  ReplyPort port;
  auto * handler = dynamixel::PacketHandler::getPacketHandler(2.0);
  for (bool txrx : {false, true}) {
    for (int fault = 0; fault < 6; ++fault) {
      uint8_t error = 0xff;
      uint8_t expected_error = fault == 3 ? 4 : fault == 5 ? 0x80 : 0;
      Bytes data = fault == 1 ? Bytes{} : fault == 2 ? Bytes{1, 2, 3} : Bytes{11, 12};
      port.load(status(fault == 4 ? 2 : 1, expected_error, data));
      uint8_t output[2] = {77, 88};
      int result = txrx ? handler->readTxRx(&port, 1, 132, 2, output, &error) :
        handler->readRx(&port, 1, 2, output, &error);
      bool valid = fault == 0 || fault == 5;
      require((result == COMM_SUCCESS) == valid);
      require(error == expected_error);
      require(output[0] == (valid ? 11 : 77) && output[1] == (valid ? 12 : 88));
    }
  }
  InspectGroup<dynamixel::GroupSyncRead> sync(&port, handler, 132, 2);
  InspectGroup<dynamixel::GroupBulkRead> bulk(&port, handler);
  InspectGroup<dynamixel::GroupFastSyncRead> fast_sync(&port, handler, 132, 2);
  InspectGroup<dynamixel::GroupFastBulkRead> fast_bulk(&port, handler);
  for (uint8_t id : {1, 2}) {
    require(sync.addParam(id));
    require(bulk.addParam(id, 132, 2));
    require(fast_sync.addParam(id));
    require(fast_bulk.addParam(id, 132, 2));
  }
  checkGroup(sync, port, false);
  checkGroup(bulk, port, false);
  checkGroup(fast_sync, port, true);
  checkGroup(fast_bulk, port, true);
}
