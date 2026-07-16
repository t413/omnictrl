#pragma once
#include <stdint.h>
#include "rfprotocol.h"


struct Peer {
  uint8_t mac[6] = {0};
  MotionControl lastCmd_;
  uint32_t lastPing_ = 0;
  bool isRecent(uint32_t now, uint32_t threshold=500) const;
  bool isArmed() const;
  uint32_t lastHeard() const;
  bool isValid() const;
};
constexpr uint8_t PEER_CRSF = 0;
constexpr uint8_t PEERS_MAX = 8;

bool validMac(const uint8_t* mac);


// ------------- //
//    Manager    //
// ------------- //

struct PeerMgr {
  Peer peers_[PEERS_MAX];
  uint8_t activePeer_ = PEERS_MAX; //who's in control

  uint8_t getActiveIdx() const { return activePeer_; }
  uint8_t findPeerIdx(const uint8_t* mac) const;
  uint8_t oldestPeerSlot() const;

  Peer const* findPeer(uint8_t idx) const { return idx < PEERS_MAX? &peers_[idx] : nullptr; }
  Peer*       findPeer(uint8_t idx)       { return idx < PEERS_MAX? &peers_[idx] : nullptr; }
  uint8_t newPeer(const uint8_t* mac);

  void activate(uint8_t idx) { activePeer_ = (idx < PEERS_MAX)? idx : PEERS_MAX; }
};
