#pragma once

inline bool isTransportTimestampFresh(unsigned long now,
                                      unsigned long lastUpdateMs,
                                      unsigned long timeoutMs) {
  return lastUpdateMs > 0 && now - lastUpdateMs < timeoutMs;
}
