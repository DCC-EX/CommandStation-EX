// EXIO_TCPClientProvider.h
#pragma once

#include <Arduino.h>
#include <Client.h>

namespace EXIO_TCPClientProvider {

  // Returns true if the platform/network stack is plausibly usable.
  // (A failed connect() will still be handled by caller.)
  bool networkReady();

  // Returns a freshly-allocated Client suitable for outbound TCP connections.
  // Caller owns it and must delete it.
  // Returns nullptr if this build has no Arduino Client-based network stack.
  Client* createClient();

}
