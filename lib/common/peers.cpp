#include "peers.h"
#include <string.h>
#include <algorithm>

using namespace std;

bool Peer::isRecent(uint32_t now, uint32_t threshold) const {
  return (now - lastCmd_.timestamp) < threshold || (now - lastPing_) < threshold;
}
bool Peer::isArmed() const {
  return (lastCmd_.state & 1);
}
uint32_t Peer::lastHeard() const {
  return max(lastCmd_.timestamp, lastPing_);
}
bool Peer::isValid() const { return validMac(mac); }


bool validMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++)
    if (mac[i] != 0) return true;
  return false;
}



// ------------- //
//    Manager    //
// ------------- //

uint8_t PeerMgr::findPeerIdx(const uint8_t* mac) const {
  for (int i = 0; i < PEERS_MAX; i++)
    if (memcmp(peers_[i].mac, mac, 6) == 0)
      return i;
  return UINT8_MAX;
}

uint8_t PeerMgr::oldestPeerSlot() const {
  uint8_t ret = PEER_CRSF + 1;
  for (uint8_t i = ret; i < PEERS_MAX; i++) {
    if (!peers_[i].lastPing_) return i; //easy mode
    if (i > ret && peers_[i].lastHeard() < peers_[i].lastHeard())
      ret = i;
  }
  return ret;
}

uint8_t PeerMgr::newPeer(const uint8_t* mac) {
  if (!mac) return UINT8_MAX;
  uint8_t newslot = oldestPeerSlot();
  if (newslot >= PEERS_MAX) return UINT8_MAX;
  memcpy(peers_[newslot].mac, mac, 6);
  return newslot;
}
