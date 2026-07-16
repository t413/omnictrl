#pragma once
#include <stdint.h>
#include "rfprotocol.h"

bool validMac(const uint8_t* mac);
bool espnowRegisterMac(const uint8_t* mac);

template<typename CmdT>
struct Peer {
  uint8_t mac[6] = {0};
  CmdT lastCmd_;
  uint32_t lastPing_ = 0;
  bool isRecent(uint32_t now, uint32_t threshold=500) const { return (now - lastHeard()) < threshold; }
  bool isArmed() const;
  uint32_t lastHeard() const;
  bool isValid() const { return validMac(mac); }
  bool registerMac() { return espnowRegisterMac(mac); }
};
constexpr uint8_t PEERS_MAX = 8;


// ------------- //
//    Manager    //
// ------------- //

template<typename CmdT>
struct PeerMgr {
  Peer<CmdT> peers_[PEERS_MAX];
  uint8_t activePeer_ = PEERS_MAX; //who's in control

  uint8_t getActiveIdx() const { return activePeer_; }
  uint8_t findOrMakePeer(const uint8_t* mac, bool allownew, bool registernewmac, bool allowActivate);
  uint8_t findPeerIdx(const uint8_t* mac) const;
  uint8_t oldestPeerSlot() const;
  bool isActive(uint8_t idx) const { return idx < PEERS_MAX && idx == activePeer_; }

  Peer<CmdT> const* findPeer(uint8_t idx) const { return idx < PEERS_MAX? &peers_[idx] : nullptr; }
  Peer<CmdT>*       findPeer(uint8_t idx)       { return idx < PEERS_MAX? &peers_[idx] : nullptr; }
  uint8_t newPeer(const uint8_t* mac);
  uint8_t getRecentCount(uint32_t now, uint32_t threshold) const;

  void activate(uint8_t idx) { activePeer_ = (idx < PEERS_MAX)? idx : PEERS_MAX; }
};
