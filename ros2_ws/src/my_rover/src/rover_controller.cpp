#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class RoverController : public rclcpp::Node {
  public:
  RoverController() : Node("rover_controller"), front_dist_(2.0), rear_dist_(2.0) {
    // Publisher verso il plugin DiffDrive
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscriber per i due sensori di prossimità (LaserScan)
    // Si usa SensorDataQoS() per compatibilità con i bridge di Gazebo
    front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/front_sensor/scan", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          front_dist_ = get_min_range(msg);
        });

    rear_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/rear_sensor/scan", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          rear_dist_ = get_min_range(msg);
        });

    // Ciclo di controllo a 20 Hz (ogni 50 ms)
    timer_ = this->create_wall_timer(
        50ms, std::bind(&RoverController::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Controller Braitenberg attivo!");
  }

  private:
  // Estrae la distanza minima valida dai 5 raggi del cono del sensore
  double get_min_range(const sensor_msgs::msg::LaserScan::SharedPtr &msg) {
    double min_val = msg->range_max;
    for (float r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r < min_val) {
        min_val = r;
      }
    }
    return min_val;
  }

  // Converte la distanza (in metri) in uno stimolo normalizzato [0.0, 1.0]
  // 0.0 = nessun ostacolo vicino | 1.0 = ostacolo a contatto
  double compute_stimulus(double distance, double threshold) {
    if (distance >= threshold) {
      return 0.0;
    }
    return std::clamp(1.0 - (distance / threshold), 0.0, 1.0);
  }

  void set_velocity(double linear_x, double angular_z) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_pub_->publish(msg);
  }

  void control_loop() {
    // Parametri di taratura Braitenberg
    const double v_base = 0.35;      // Velocità di crociera in campo libero (m/s)
    const double threshold = 0.80;   // Distanza di attivazione sensori (m)
    const double k_front = 0.75;     // Guadagno repulsione anteriore (> v_base per poter retromarciare)
    const double k_rear = 0.60;      // Guadagno repulsione posteriore
    const double k_turn = 1.40;      // Guadagno rotazione per evitare stalli frontali/posteriori

    // 1. Calcolo degli stimoli dai sensori [0.0 .. 1.0]
    double s_front = compute_stimulus(front_dist_, threshold);
    double s_rear  = compute_stimulus(rear_dist_, threshold);

    // 2. Legge di Braitenberg:
    // - Ostacolo davanti (s_front) -> frena e inverte la marcia
    // - Ostacolo dietro (s_rear)   -> spinge in avanti
    double linear_x = v_base - (k_front * s_front) + (k_rear * s_rear);

    // 3. Rotazione proporzionale allo stimolo per sterzare via dall'ostacolo
    double angular_z = k_turn * (s_front + s_rear);

    // Limiti di sicurezza sulle velocità
    linear_x  = std::clamp(linear_x, -0.5, 0.5);
    angular_z = std::clamp(angular_z, -1.5, 1.5);

    set_velocity(linear_x, angular_z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double front_dist_;
  double rear_dist_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverController>());
  rclcpp::shutdown();
  return 0;
}