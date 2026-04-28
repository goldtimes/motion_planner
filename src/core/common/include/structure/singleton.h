#ifndef RMP_COMMON_STRUCTURE_SINGLETON_H_
#define RMP_COMMON_STRUCTURE_SINGLETON_H_
#include <memory>
namespace rmp::common::structure
{
template <typename TSingleton>
class Singleton
{
public:
  using TSingletonPtr = std::unique_ptr<TSingleton>;

private:
  Singleton() = default;
  virtual ~Singleton() = default;
  Singleton(const Singleton&) = delete;
  Singleton(Singleton&&) = delete;
  Singleton& operator=(const Singleton&) = delete;

public:
  static TSingletonPtr& Instance()
  {
    static TSingletonPtr instance = std::make_unique<TSingleton>();
    return instance;
  }
};
}  // namespace rmp
#endif