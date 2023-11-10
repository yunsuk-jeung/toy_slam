#pragma once

class Frame;
class FeatureModule {
public:
  FeatureModule() {}
  virtual ~FeatureModule() {}

protected:
  virtual bool process(Frame*) = 0;
};