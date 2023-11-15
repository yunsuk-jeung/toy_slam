#pragma once

namespace toy {

template <typename T>
class Singleton {
public:
  static T* getInstance() {
    if (!instance) {
      instance = new T();
    }
    return instance;
  }

  static void deleteInstance() { 
    delete instance; 
  }

protected:
  Singleton() {}
  virtual ~Singleton() {}

  Singleton(const Singleton&)            = delete;
  Singleton& operator=(const Singleton&) = delete;
  Singleton(Singleton&&)                 = delete;
  Singleton& operator=(Singleton&&)      = delete;

private:
  static T* instance;
};

template <typename T>
T* Singleton<T>::instance(nullptr);

}  //namespace toy