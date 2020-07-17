#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <stdexcept>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include "rtos_common.hpp"
#include "openradio.hpp"
#include <algorithm>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <vector>
#include <memory>

#define u32_as_bytes(n) (uint8_t)((n >> 0) & 0xFF), (uint8_t)((n >> 8) & 0xFF), (uint8_t)((n >> 16) & 0xFF), (uint8_t)((n >> 24) & 0xFF)
#define u16_as_bytes(n) (uint8_t)((n >> 0) & 0xFF), (uint8_t)((n >> 8) & 0xFF)
#define bytes_as_u32(buf, off) ((uint32_t)(((buf)[off] << 0) | ((buf)[off+1] << 8) | ((buf)[off+2] << 16) | ((buf)[off+3] << 24)))
#define bytes_as_u16(buf, off) ((uint32_t)(((buf)[off] << 0) | ((buf)[off+1] << 8)))

//Max number of connections that should be managed.
//1 is recommended unless you need multiple connections (and
//have code that can deal with such an arrangement)
static const int maxConnections = 5;

//Keep in mind that LWIP send() on ESP32 can buffer a max of 5744 bytes.
//Keeping this lowish means that 2 big packets or several small packets
//can be buffered for a weak wireless connection to not hold up the
//lower-level robot-to-ESP32 communication.
static const int maxPacketDataSize = 2048;

//Physical receive and transmit ports
static uart_port_t rx;
static uart_port_t tx;

//Serial number of last acknowledged remote packet.
int oppSerial = 0;
//Serial number of current packet to be sent.
int mySerial = 1;

using bytes = std::vector<uint8_t>;
using std::vector;

template <typename T>
struct Queue {
  QueueHandle_t queue;
  Queue(int size): queue(xQueueCreate(size, sizeof(T))) {
    printf("queue is %08X\n", (int)queue);
  }
  Queue(const Queue&) = delete;
  Queue& operator=(const Queue&) = delete;
  ~Queue() {
    vQueueDelete(queue);
  }
  T recv(uint32_t timeout = 0) {
    T ret;
    if(timeout == 0) {
      xQueueReceive(queue, &ret, portMAX_DELAY);
    } else {
      if(!xQueueReceive(queue, &ret, timeout)) {
        throw std::runtime_error("No bytes within timeout.");
      }
    }
    return std::move(ret);
  }
  vector<T> bulk_recv(uint32_t amt) {
    vector<T> ret;
    if(!amt) return ret;
    ret.resize(amt);
    printf("Ret data & size: %08X %d\n", (int)ret.data(), ret.size());
    for(int i = 0; i < amt; i++) xQueueReceive(queue, &ret[i], portMAX_DELAY);
    return ret;
  }
  void push(T b, uint32_t timeout = 0) {
    if(timeout == 0) {
      xQueueSend(queue, &b, portMAX_DELAY);
    } else {
      if(!xQueueSend(queue, &b, timeout)) {
        throw std::runtime_error("No more room!");
      }
    }
  }
  size_t waiting() {
    return uxQueueMessagesWaiting(queue);
  }
  void reset() {
    xQueueReset(queue);
  }
};

struct Message {
  int channel;
  bytes data;
};

struct ORPacket {
  int opp;
  int cur;
  vector<Message> messages;
  ORPacket(): opp{oppSerial}, cur{mySerial} {}
};

uint16_t crc16(const unsigned char *buf, int len);

void dumpBytes(const bytes& data) {
  for(uint8_t b: data) {
    printf("%02X ", b);
  }
  printf("\n");
}

ORPacket deserializePacket(const bytes& rawData) {
  ORPacket ret;
  //Skipping over 4 sync bytes
  ret.opp = bytes_as_u32(rawData, 4 + 0);
  ret.cur = bytes_as_u32(rawData, 4 + 4);
  //Only necessary for creation of the buffer
  //int msgSize = bytes_as_u32(rawData, 4 + 8);
  uint16_t providedCRC = 0;
  providedCRC = bytes_as_u16(rawData, rawData.size() - 2);
  uint16_t expectedCRC = crc16(rawData.data(), rawData.size());
  if(providedCRC != expectedCRC) {
    dumpBytes(rawData);
    throw std::runtime_error("Bad checksum (expected " + std::to_string(expectedCRC) + " but got " + std::to_string(providedCRC) + ")");
  }

  //TODO: This assumes the packet is well-formed.
  //(It *probably* is, but just for that 1/65536 chance of missing a failure, this should be fixed.)
  int offset = 20;
  for(int i = 0; i < bytes_as_u32(rawData, 16); i++) {
    Message msg;
    msg.channel = bytes_as_u32(rawData, offset);
    offset += 4;
    uint32_t length = bytes_as_u32(rawData, offset);
    offset += 4;
    msg.data.insert(msg.data.end(), rawData.begin() + offset, rawData.begin() + offset + length);
    offset += length;
    ret.messages.push_back(msg);
  }

  return ret;
}

bytes bytesForMessages(const vector<Message>& messages) {
  bytes ret;
  ret.insert(ret.end(), { u32_as_bytes(messages.size()) });
  for(const auto& msg: messages) {
    ret.insert(ret.end(), { u32_as_bytes(msg.channel), u32_as_bytes(msg.data.size()) });
    ret.insert(ret.end(), msg.data.begin(), msg.data.end());
  }
  return ret;
}

bytes serializePacket(const ORPacket& toSend) {
  bytes content = bytesForMessages(toSend.messages);
  bytes serialized = {
    0x77, 0xBB, 0x77, 0xBB, u32_as_bytes(toSend.opp), u32_as_bytes(toSend.cur), u32_as_bytes(content.size())
  };
  serialized.insert(serialized.end(), content.begin(), content.end());
  auto checksum = crc16(serialized.data(), serialized.size());
  serialized.insert(serialized.end(), { u16_as_bytes(checksum) });
  return serialized;
}

struct Connection {
  int fd;
  bytes receivedData;
  //Tries to get a valid packet from receivedData.
  std::unique_ptr<ORPacket> tryRecvCached() {
    //14: Header length + CRC
    if(receivedData.size() < 14) return std::unique_ptr<ORPacket>(nullptr);
    int length = bytes_as_u32(receivedData, 8);
    if(length > maxPacketDataSize || length < 4) {
      throw std::runtime_error("This connection is sending malformed packets (invalid size for maxPacketDataSize)");
    }
    if(receivedData.size() >= 14 + length) {
      return std::unique_ptr<ORPacket>(new ORPacket(deserializePacket(receivedData)));
    }
    return std::unique_ptr<ORPacket>(nullptr);
  }
  //Gets as much data as possible into receivedData.
  //If receivedData is a valid packet, this will return that
  //and clear the packet from receivedData. Otherwise it will
  //return nullptr.
  std::unique_ptr<ORPacket> tryRecv() {
    auto ret = tryRecvCached();
    if(ret) return ret;
    //Read as much as we can into the cache
    int curIdx = receivedData.size();
    receivedData.resize(receivedData.size() + maxPacketDataSize + 14);
    int reallyRead = recv(fd, receivedData.data() + curIdx, maxPacketDataSize + 14, 0);
    if(reallyRead == -1) {
      receivedData.resize(curIdx);
      if(errno == EWOULDBLOCK) return nullptr;
      perror("recv");
      throw std::runtime_error("Fail while receiving from connection");
    }
    receivedData.resize(curIdx + reallyRead);
    //And try to form a packet from the cache again.
    return tryRecvCached();
  }
  bytes packetToSend;
  //Returns false if packetToSend is still populated, otherwise fills packetToSend with packet data.
  //Then attempts to send as much of packetToSend as possible to the connection.
  bool trySend(ORPacket& toSend) {
    bool ret = packetToSend.empty();
    if(ret) {
      packetToSend = serializePacket(toSend);
    }
    int reallySent = send(fd, packetToSend.data(), packetToSend.size(), 0);
    if(reallySent == -1) {
      if(errno == EWOULDBLOCK) {
        printf("A connection is blocked. Not sending data to any connection until this one thaws. Consider lowering maxConnections to 1.\n");
        return ret;
      }
      perror("send");
      throw std::runtime_error("Fail while sending to connection");
    }
    packetToSend.erase(packetToSend.begin(), packetToSend.begin() + reallySent);
    return ret;
  }
  //Flag used to check if this connection has accepted the latest packet.
  //Set to true after trySend returns true
  //Once the above has happened for all connections, all connections have this flag reset to false
  //and opp is incremented to properly acknowledge the robot's packet.
  bool hasHadPacketSent;
  Connection(int fd): fd(fd) {}
  Connection(): fd(-1) {}
  //Don't copy this.
  Connection(const Connection&) = delete;
  Connection& operator=(const Connection&) = delete;
  Connection(Connection&& other) {
    fd = other.fd;
    other.fd = -1;
  }
  Connection& operator=(Connection&& other) {
    if(this == &other) return *this;
    if(fd != -1) {
      close(fd);
    }
    fd = other.fd;
    other.fd = -1;
    return *this;
  };
  ~Connection() {
    if(fd != -1) close(fd);
  }
};

vector<Connection> connections;
SemaphoreHandle_t connectionsLock;

//Incoming Connection Thread
void handleIncomingConnections() {
  //Prepare socket
  int listeningSock = socket(AF_INET6, SOCK_STREAM, IPPROTO_IPV6);
  if(listeningSock < 0) {
    perror("socket");
    esp_restart();
  }

  struct sockaddr_in6 destAddr;
  memset((void*)&destAddr.sin6_addr.un, 0, sizeof(destAddr.sin6_addr.un));
  destAddr.sin6_family = AF_INET6;
  destAddr.sin6_port = htons(8301);
  if(bind(listeningSock, (struct sockaddr*)&destAddr, sizeof(destAddr))) {
    perror("bind");
    esp_restart();
  }

  if(listen(listeningSock, 1)) {
    perror("listen");
    esp_restart();
  }

  printf("Listening on port %d.\n", destAddr.sin6_port);

  //Watch for connections
  while(true) {
    struct sockaddr_in6 sourceAddr;
    uint32_t addrLen = sizeof(sourceAddr);
    int sock = accept(listeningSock, (struct sockaddr*)&sourceAddr, &addrLen);
    if (sock < 0) {
      perror("accept");
      continue;
    }
    printf("Got a connection!\n");

    char* addrStr = (char*)malloc(128);
    strcpy(addrStr, "(Unknown host)");
    if (sourceAddr.sin6_family == PF_INET) {
      inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addrStr, 128 - 1);
    } else if (sourceAddr.sin6_family == PF_INET6) {
      inet6_ntoa_r(sourceAddr.sin6_addr, addrStr, 128 - 1);
    }

    printf("Accepted connection from %s on port %d, handling on socket %d.\n", addrStr, listeningSock, sock);
    free(addrStr);

    int flags = fcntl(sock, F_GETFL);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    xSemaphoreTake(connectionsLock, 100);
    connections.emplace_back(sock);
    if(connections.size() > maxConnections) {
      connections.erase(connections.begin());
      printf("Disconnected one connection because the total number of connections was > maxConnections.\n");
    }
    xSemaphoreGive(connectionsLock);
  }
}

void delayMS(int time) {
  vTaskDelay(time);
}

uint8_t ser_getchar(bool first = false) {
  uint8_t ret;
  //Only wait past the 40ms rescan rate if waiting for the first byte of a packet.
  //Otherwise, if we get a corrupted size, we will keep reading indefinitely.
  auto timeout = first ? 90 : 30;
  if(uart_read_bytes(rx, &ret, 1, timeout) <= 0) {
    throw std::runtime_error("No serial bytes within timeout.");
  }
  return ret;
}

void writeDataOut(const bytes& data) {
  int remaining = data.size();
  while(remaining) {
    int written = uart_write_bytes(tx, (const char*)data.data() + (data.size() - remaining), data.size());
    if(written != remaining) printf("Only wrote %d bytes\n", written);
    remaining -= written;
  }
}

void clearInputBuffer() {
  size_t sizeToClear;
  uart_get_buffered_data_len(rx, &sizeToClear);
  uint8_t* ignored = new uint8_t[sizeToClear];
  uart_read_bytes(rx, ignored, sizeToClear, 1);
  delete ignored;
}

void sendPacket(const ORPacket& toSend) {
  writeDataOut(serializePacket(toSend));
}

const uint8_t syncBytes[] = { 0x55, 0xAA, 0x55, 0xAA };
ORPacket kokoroyohou() {
  int syncIndex = 0;
  bytes entireMessage;
  while(true) {
    auto syncChar = ser_getchar(syncIndex == 0);
    if(syncBytes[syncIndex] == syncChar) syncIndex++;
    else syncIndex = 0;
    if(syncIndex == 4) break;
  }
  entireMessage.insert(entireMessage.begin(), syncBytes, syncBytes + 4);
  for(int i = 0; i < 12; i++) {
    entireMessage.push_back(ser_getchar());
  }
  int msgSize = bytes_as_u32(entireMessage, 4 + 8);
  if(msgSize > maxPacketDataSize || msgSize < 4) {
    dumpBytes(entireMessage);
    try {
      while(true) ser_getchar();
    } catch(...) {
      throw std::runtime_error("Refused to allocate " + std::to_string(msgSize) + " bytes of memory for message");
    }
  }
  for(int i = 0; i < msgSize + 2; i++) {
    entireMessage.push_back(ser_getchar());
  }
  return deserializePacket(entireMessage);
}

//Messages with a channel namespace of 1 are considered meta-messages for the ESP32.
//This function must return immediately.
void handleMetaMessage(Message& msg) {
  printf("Got meta-message.\n");
  dumpBytes(msg.data);
}

void handleMetaMessages(ORPacket& packet) {
  for(int i = 0; i < packet.messages.size(); i++) {
    auto& msg = packet.messages[i];
    if(msg.channel >> 8 == 1) {
      try {
        handleMetaMessage(msg);
      } catch(const std::runtime_error& ex) {
        printf("Exception %s while handling meta-message to channel %d\n", ex.what(), msg.channel);
      } catch(...) {
        printf("Unknown exception while handling meta-message to channel %d\n", msg.channel);
      }
      packet.messages.erase(packet.messages.begin() + i);
      i--;
      continue;
    }
  }
}

ORPacket makeNextPacket() {
  xSemaphoreTake(connectionsLock, 100);
  for(int i = 0; i < connections.size(); i++) {
    auto& con = connections[i];
    std::unique_ptr<ORPacket> packet;
    try {
      packet = con.tryRecv();
    } catch(const std::runtime_error& ex) {
      printf("Disconnecting connection because: %s\n", ex.what());
      connections.erase(connections.begin() + i);
      i--;
      continue;
    } catch(...) {
      printf("Bruh moment detected while trying to receive packet from connection.\n");
      connections.erase(connections.begin() + i);
      i--;
      continue;
    }
    if(packet) {
      handleMetaMessages(*packet);
      if(packet->messages.size()) {
        //This is the one we need to send.
        //(Note: It's up to the user submitting packets to send to keep the packet's data < maxPacketDataSize.)
        xSemaphoreGive(connectionsLock);
        return *packet;
      }
    }
  }
  xSemaphoreGive(connectionsLock);
  return ORPacket();
}

void radioLoop() {
  printf("Yo, it's radioLoop!\n");
  ORPacket next = makeNextPacket();
  while(true) {
    try {
      ORPacket received = kokoroyohou();
      //If the V5 brain has just started, but this has been going,
      //then disconnect all clients, reset mySerial, and reset "next".
      if(received.cur == 1 && mySerial != 1) {
        //This will call the destructors of the connections,
        //causing them to close.
        connections.clear();
        mySerial = 1;
        next = ORPacket();
      }
      if(received.cur % 1000 == 0) {
        printf("Serials %d, %d with desync of %d\n", received.cur, received.opp, received.cur - received.opp);
      }
      if(received.cur != oppSerial) { // "You need to handle my packet"
        handleMetaMessages(received);
        if(received.messages.size()) {
          bool allConnectionsGotPacket = true;
          xSemaphoreTake(connectionsLock, 100);
          for(int i = 0; i < connections.size(); i++) {
            auto& con = connections[i];
            if(!con.hasHadPacketSent) {
              try {
                if(!(con.hasHadPacketSent = con.trySend(received))) {
                  allConnectionsGotPacket = false;
                }
              } catch(const std::runtime_error& ex) {
                printf("Disconnecting connection because: %s\n", ex.what());
                connections.erase(connections.begin() + i);
                i--;
                continue;
              } catch(...) {
                printf("Bruh moment detected while trying to send packet to connection.\n");
                connections.erase(connections.begin() + i);
                i--;
                continue;
              }
            }
          }
          if(allConnectionsGotPacket) {
            for(auto& con: connections) {
              con.hasHadPacketSent = false;
            }
            next.opp = oppSerial = received.cur;
          }
          xSemaphoreGive(connectionsLock);
        } else {
          next.opp = oppSerial = received.cur;
        }
        //If not all connections could get the packet; just ignore the robot and try again next time.
      } else {
        //We handled the robot's packet, but the robot still wants to send it?
        printf("Remote is unsure whether data was received.\n");
      }
      if(received.opp == mySerial) { // "I handled your packet"
        mySerial++;
        if(mySerial <= 0) mySerial = 1;
        next = makeNextPacket();
      } else if(mySerial != 1) {
        printf("Remote did not receive message (or is overloaded).\n");
      }
      sendPacket(next);
      clearInputBuffer();
    } catch(const std::exception& owo) {
      if(strlen(owo.what()) < 23 || strncmp(owo.what(), "No bytes within timeout", 23) != 0) {
        printf("%s\n", owo.what());
      }
    } catch(...) {
      delayMS(100);
    }
  }
}

void init_openradio(uart_port_t rxPort, uart_port_t txPort) {
  rx = rxPort; tx = txPort;
  connectionsLock = xSemaphoreCreateMutex();
  launchCPPTask(radioLoop, "Radio Loop", 8192);
  launchCPPTask(handleIncomingConnections, "Network", 4096);
}