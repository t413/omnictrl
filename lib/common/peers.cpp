#include "peers.h"
#include <cstring>
#include <algorithm>
#include "log.h"
#ifdef ESP32
#include <esp_now.h>
#endif

using namespace std;

const uint8_t BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool validMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++)
    if (mac[i] != 0) return true;
  return false;
}

bool espnowRegisterMac(const uint8_t* mac) {
  if (!mac) { D_LOG("peers: ERROR no mac"); return false; }
  #ifdef ESP32
  esp_now_peer_info_t newpeer = {0};
  memcpy(newpeer.peer_addr, mac, 6);
  newpeer.channel = 0;
  newpeer.encrypt = false;
  auto res = esp_now_add_peer(&newpeer);
  D_LOG("peers: ESP-NOW add peer: %d", res);
  return (res == ESP_OK);
  #endif
  return false;
}

// ------------- //
//    Peer       //
// ------------- //

template<>bool     Peer<MotionControl>::isArmed() const { return (lastCmd_.state & 1); }
template<>uint32_t Peer<MotionControl>::lastHeard() const { return max(lastCmd_.timestamp, lastPing_); }

template<>uint32_t Peer<Telem>::lastHeard() const { return max(lastCmd_.timestamp, lastPing_); }


// ------------- //
//    Manager    //
// ------------- //

template<typename CmdT>
uint8_t PeerMgr<CmdT>::findOrMakePeer(const uint8_t* mac, bool allownew, bool registernewmac, bool allowActivate) {
  auto peeridx = findPeerIdx(mac);
  if ((peeridx >= PEERS_MAX) && allownew) { //new peer!
    peeridx = newPeer(mac);
    if (peeridx >= PEERS_MAX) return peeridx; //no open slot
    if (registernewmac)
      espnowRegisterMac(mac); // Register peer's mac
    if (allowActivate && activePeer_ == PEERS_MAX) {
      activate(peeridx);
    }
    D_LOG("peers: new peer :%02x:%02x -> [%d] act%d", mac[4], mac[5], peeridx, isActive(peeridx));
  }
  return peeridx;
}

template<typename CmdT>
uint8_t PeerMgr<CmdT>::findPeerIdx(const uint8_t* mac) const {
  if (mac)
    for (int i = 0; i < PEERS_MAX; i++)
      if (memcmp(peers_[i].mac, mac, 6) == 0)
        return i;
  return UINT8_MAX;
}

template<typename CmdT>
uint8_t PeerMgr<CmdT>::openSlot() const {
  for (uint8_t i = 0; i < PEERS_MAX; i++)
    if (peers_[i].lastHeard() == 0 || !peers_[i].isValid())
      return i;
  return PEERS_MAX;
}

template<typename CmdT>
uint8_t PeerMgr<CmdT>::newPeer(const uint8_t* mac) {
  if (!mac) return UINT8_MAX;
  uint8_t newslot = openSlot();
  if (newslot >= PEERS_MAX) {
    D_LOG("peers can't find slot");
    return UINT8_MAX;
  }
  memcpy(peers_[newslot].mac, mac, 6);
  D_LOG("peers: new slot :%02x:%02x -> %d", mac[4], mac[5], newslot);
  return newslot;
}

template<typename CmdT>
uint8_t PeerMgr<CmdT>::getRecentCount(uint32_t now, uint32_t threshold) const {
  uint8_t ret = 0;
  for (uint8_t i = 0; i < PEERS_MAX; i++) {
    auto& peer = peers_[i];
    if (peer.isValid() && peer.isRecent(now, threshold))
      ret++;
  }
  return ret;
}

// Explicit template instantiations
template class PeerMgr<MotionControl>;
template class PeerMgr<Telem>;
