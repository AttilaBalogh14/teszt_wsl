#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>  // A szög és a fordulás miatt

class DiamondDraw : public rclcpp::Node
{
public:
    DiamondDraw() : Node("diamond_draw"), count_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        /*pose_publisher_ = this->create_publisher<turtlesim::msg::Pose>("/turtle1/pose", 10);

        // Kezdő pozíció a képernyő közepén
        auto pose_message = turtlesim::msg::Pose();
        pose_message.x = 5.0;
        pose_message.y = 5.0;
        pose_message.theta = 0.0;
        pose_publisher_->publish(pose_message);
        */
        // Indítjuk a ciklust
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&DiamondDraw::loop, this));
        RCLCPP_INFO_STREAM(this->get_logger(), "Drawing a diamond.");
        loop();
    }

private:
    void publish_message(double fwd, double turn)
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = fwd;
        message.angular.z = turn;
        count_++;
        RCLCPP_INFO(this->get_logger(), "Step %ld. speed: '%.1f' turn: '%.1f'", count_, message.linear.x, message.angular.z);
        publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Csökkentett idő
    }

    void loop()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Loop started.");
        
        // Rajzolás kezdete
        std::this_thread::sleep_for(std::chrono::seconds(1)); // delay for 2 second
        
        // Gyémánt rajzolása
        draw_diamond();

        // A program vége
        RCLCPP_INFO_STREAM(this->get_logger(), "Diamond drawing finished.");
        rclcpp::shutdown();
    }

    void draw_diamond()
    {      
        publish_message(0.0, 5*M_PI / 4);  // Fordulás 45 fokot jobbra
        publish_message(1, 0.0);  // Előrehaladás
        publish_message(0.0, 136*M_PI / 180);  // Fordulás 45 fokot jobbra
        publish_message(4, 0.0);  // Előrehaladás
        publish_message(0.0, 3*M_PI / 4);  // Fordulás 45 fokot jobbra
        publish_message(1, 0.0);  // Előrehaladás
        publish_message(0.0, 44.8*M_PI / 180);  // Fordulás 45 fokot jobbra
        publish_message(2.55, 0.0);  // Előrehaladás
        publish_message(0.0, 3*M_PI / 4);  // Fordulás 45 fokot jobbra

        for (int i = 0; i < 2; ++i)  // 2 oldal felfelé és lefelé
        {
            publish_message(0.9, 0.0);  // Előrehaladás
            publish_message(0.0, M_PI / 2);  // Fordulás 90 fokot balra (180 - 45*2)
            publish_message(0.9, 0.0);  // Előrehaladás
            publish_message(0.0, -M_PI / 2);  // Fordulás 90 fokot balra (180 - 45*2)
        }

        publish_message(1, 0.0);  // Előrehaladás
        publish_message(0.0, -M_PI / 2);  // Fordulás 90 fokot balra (180 - 45*2)
        publish_message(3.3, 0.0);  // Előrehaladás
        publish_message(0.0, -157*M_PI / 180);  // Fordulás 90 fokot balra (180 - 45*2)
        publish_message(2.55, 0.0);  // Előrehaladás
        publish_message(0.0, 111.6*M_PI / 180);  // Fordulás 90 fokot balra (180 - 45*2)
        publish_message(1.265, 0.0);  // Előrehaladás
        publish_message(0.0, 96.5*M_PI / 180);  // Fordulás 90 fokot balra (180 - 45*2)
        publish_message(2.4, 0.0);  // Előrehaladás
        publish_message(0.0, -5*M_PI / 6);  // Fordulás 90 fokot balra (180 - 45*2)
        publish_message(2.8, 0.0);  // Előrehaladás
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr pose_publisher_; // Új publisher a pozícióhoz
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiamondDraw>());
    rclcpp::shutdown();
    return 0;
}
