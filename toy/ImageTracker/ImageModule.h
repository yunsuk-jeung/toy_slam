#pragma once

class Frame;
class ImageModule {
public:

  ImageModule() {}
  virtual ~ImageModule() {}

protected:

  bool process(Frame*) = 0;
};