// lo_supervisor: drives the lifecycle of the STRIDE node (docs/DESIGN.md "Lifecycle discipline",
// "Process crash -> Restart, new chain_id, INIT").
//   unconfigured -> configure; inactive -> activate (refused until INIT completed: retried);
//   active -> monitored. If the node disappears (crash), launch respawns it (respawn=True) and
//   it comes back unconfigured: the supervisor configures and activates it again. The new
//   process cold-starts (INIT) and its chain_id is the stamp of its first frame, i.e. new.
#include <chrono>
#include <memory>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class Supervisor : public rclcpp::Node
{
public:
  Supervisor()
  : Node("lo_supervisor")
  {
    target_ = declare_parameter<std::string>("node_name", "stride_lo");
    const double period = declare_parameter<double>("retry_period", 0.5);
    get_state_ = create_client<lifecycle_msgs::srv::GetState>(target_ + "/get_state");
    change_state_ = create_client<lifecycle_msgs::srv::ChangeState>(target_ + "/change_state");
    timer_ = create_wall_timer(std::chrono::duration<double>(period), [this] {tick();});
    RCLCPP_INFO(get_logger(), "supervising '%s'", target_.c_str());
  }

private:
  void tick()
  {
    if (busy_) {return;}
    if (!get_state_->service_is_ready()) {
      if (last_state_ != State::PRIMARY_STATE_UNKNOWN) {
        RCLCPP_WARN(get_logger(), "%s is gone (crash?): waiting for its respawn", target_.c_str());
        last_state_ = State::PRIMARY_STATE_UNKNOWN;
      }
      return;
    }
    busy_ = true;
    get_state_->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture f) {
        const uint8_t id = f.get()->current_state.id;
        if (id != last_state_) {
          RCLCPP_INFO(get_logger(), "%s: %s", target_.c_str(), f.get()->current_state.label.c_str());
          last_state_ = id;
        }
        if (id == State::PRIMARY_STATE_UNCONFIGURED) {
          request(Transition::TRANSITION_CONFIGURE);
        } else if (id == State::PRIMARY_STATE_INACTIVE) {
          request(Transition::TRANSITION_ACTIVATE);   // refused until INIT completed
        } else {
          busy_ = false;
        }
      });
  }

  void request(uint8_t transition)
  {
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = transition;
    change_state_->async_send_request(req,
      [this, transition](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture f) {
        if (!f.get()->success && transition == Transition::TRANSITION_CONFIGURE) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
            "configure of %s failed (see its log); retrying", target_.c_str());
        }
        busy_ = false;
      });
  }

  std::string target_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint8_t last_state_{State::PRIMARY_STATE_UNKNOWN};
  bool busy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Supervisor>());
  rclcpp::shutdown();
  return 0;
}
