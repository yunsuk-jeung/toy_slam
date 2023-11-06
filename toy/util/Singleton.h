#pragma once

#include <memory>

namespace toy {

template <typename T>
class Singleton {
public:

  static T* getInstance() {
    if (!instance) {
      instance.reset(new T());
    }
    return instance.get();
  }

  static void deleteInstance() { instance.reset(); }

protected:

  Singleton() {}  
  virtual ~Singleton() {}

  Singleton(const Singleton&)            = delete;
  Singleton& operator=(const Singleton&) = delete;
  Singleton(Singleton&&)                 = delete;
  Singleton& operator=(Singleton&&)      = delete;

private:

  static std::unique_ptr<T, void (*)(T*)> instance;
};

template <typename T>
std::unique_ptr<T, void (*)(T*)> Singleton<T>::instance(nullptr, [](T* obj) { delete obj; });

}  //namespace ts