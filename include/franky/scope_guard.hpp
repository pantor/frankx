#include <functional>

namespace franky {
  class scope_guard {
  public:
    explicit scope_guard(std::function<void()> f) : f_(f) {}

    ~scope_guard() {
      f_();
    }

  private:
    std::function<void()> f_;
  };
}