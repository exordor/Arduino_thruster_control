#pragma once

struct ThrusterCommandDecision {
  bool shouldApply;
  unsigned long receivedAtMs;
};

class ThrusterCommandGate {
 public:
  explicit ThrusterCommandGate(unsigned long minRepeatIntervalMs)
      : minRepeatIntervalMs_(minRepeatIntervalMs),
        haveAppliedCommand_(false),
        lastLeftUs_(0),
        lastRightUs_(0),
        lastAppliedAtMs_(0) {}

  ThrusterCommandDecision observe(int leftUs,
                                  int rightUs,
                                  unsigned long now) {
    const bool changed = !haveAppliedCommand_ || leftUs != lastLeftUs_ ||
                         rightUs != lastRightUs_;
    const bool repeatIntervalElapsed =
        haveAppliedCommand_ &&
        now - lastAppliedAtMs_ >= minRepeatIntervalMs_;
    const bool shouldApply = changed || repeatIntervalElapsed;

    if (shouldApply) {
      haveAppliedCommand_ = true;
      lastLeftUs_ = leftUs;
      lastRightUs_ = rightUs;
      lastAppliedAtMs_ = now;
    }

    return {shouldApply, now};
  }

 private:
  unsigned long minRepeatIntervalMs_;
  bool haveAppliedCommand_;
  int lastLeftUs_;
  int lastRightUs_;
  unsigned long lastAppliedAtMs_;
};
