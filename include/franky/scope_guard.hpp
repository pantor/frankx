#include <functional>
#include <utility>

namespace franky {
  class scope_guard {
  public:
    explicit scope_guard(std::function<void()> f) : f_(std::move(f)) {}

    ~scope_guard() {
      f_();
    }

  private:
    std::function<void()> f_;
  };
}  // namespace franky