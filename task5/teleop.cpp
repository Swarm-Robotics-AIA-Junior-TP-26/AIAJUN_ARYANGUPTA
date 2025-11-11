#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/set_pen.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <map>
#include <string>

const double NORMAL_SPEED = 2.0;
const double BOOST_SPEED = 4.0;
const double BOOST_DURATION_SEC = 1.0;

const int DEFAULT_PEN_R = 0;
const int DEFAULT_PEN_G = 0;
const int DEFAULT_PEN_B = 0;
const int DEFAULT_PEN_WIDTH = 3;

std::map<std::string, std::tuple<double, double>> MOVE_BINDINGS = {
    {"w", {1.0, 0.0}},
    {"s", {-1.0, 0.0}},
    {"a", {0.0, 1.0}},
    {"d", {0.0, -1.0}},
    {"\x1b[A", {1.0, 0.0}},
    {"\x1b[B", {-1.0, 0.0}},
    {"\x1b[C", {0.0, -1.0}},
    {"\x1b[D", {0.0, 1.0}}
};

std::map<std::string, std::tuple<int, int, int>> COLOR_BINDINGS = {
    {"r", {255, 0, 0}},
    {"g", {0, 255, 0}},
    {"b", {0, 0, 255}}
};

struct termios g_original_termios;

void save_terminal_settings()
{
    tcgetattr(STDIN_FILENO, &g_original_termios);
}

void restore_terminal_settings()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &g_original_termios);
}

void set_terminal_raw()
{
    struct termios raw = g_original_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

class ExtendedTeleop : public rclcpp::Node
{
public:
    ExtendedTeleop() : Node("extended_teleop")
    {
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        cli_set_pen_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        while (!cli_set_pen_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Service /turtle1/set_pen not available, waiting...");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to /turtle1/set_pen service.");

        boost_active_ = false;
        target_linear_x_ = 0.0;
        target_angular_z_ = 0.0;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ExtendedTeleop::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Extended Teleop Node Started.");
        print_instructions();
    }

private:
    void print_instructions()
    {
        RCLCPP_INFO(this->get_logger(), "\n"
            "ROS2 Teleop Task - Turtle Control\n"
            "-----------------------------------\n"
            "* Arrow Keys / WASD: Move the turtle (hold to move)\n"
            "* Spacebar: Apply a 1-second velocity boost\n"
            "* R: Change pen to red\n"
            "* G: Change pen to green\n"
            "* B: Change pen to blue\n"
            "* Any other key: Reset pen to default (black)\n"
            "\n"
            "Press Ctrl+C to quit."
        );
    }

    std::string get_key()
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        int ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &timeout);
        if (ret > 0)
        {
            char buffer[4];
            int n = read(STDIN_FILENO, buffer, 4);
            if (n < 0) return "";

            if (buffer[0] == '\x1b' && n > 1)
            {
                return std::string(buffer, n);
            }
            return std::string(1, buffer[0]);
        }
        return "";
    }

    void call_set_pen(int r, int g, int b, int width = DEFAULT_PEN_WIDTH, int off = 0)
    {
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->r = r;
        request->g = g;
        request->b = b;
        request->width = width;
        request->off = off;

        cli_set_pen_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Setting pen color to (%d, %d, %d)", r, g, b);
    }

    void timer_callback()
    {
        std::string key = get_key();
        bool key_pressed = !key.empty();

        if (key == "\x03")
        {
            RCLCPP_INFO(this->get_logger(), "Ctrl+C detected, shutting down.");
            rclcpp::shutdown();
            return;
        }

        double current_speed = NORMAL_SPEED;
        if (boost_active_)
        {
            if (this->get_clock()->now() > boost_end_time_)
            {
                boost_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Boost expired.");
            }
            else
            {
                current_speed = BOOST_SPEED;
            }
        }

        if (MOVE_BINDINGS.count(key))
        {
            target_linear_x_ = std::get<0>(MOVE_BINDINGS[key]);
            target_angular_z_ = std::get<1>(MOVE_BINDINGS[key]);
        }
        else if (key == " ")
        {
            if (!boost_active_)
            {
                boost_active_ = true;
                boost_end_time_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(BOOST_DURATION_SEC);
                current_speed = BOOST_SPEED;
                RCLCPP_INFO(this->get_logger(), "Boost activated! Speed: %.1f for %.1fs", current_speed, BOOST_DURATION_SEC);
            }
        }
        else if (COLOR_BINDINGS.count(key))
        {
            auto [r, g, b] = COLOR_BINDINGS[key];
            call_set_pen(r, g, b);
            target_linear_x_ = 0.0;
            target_angular_z_ = 0.0;
        }
        else if (key_pressed)
        {
            call_set_pen(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B);
            target_linear_x_ = 0.0;
            target_angular_z_ = 0.0;
        }
        else if (!key_pressed)
        {
            target_linear_x_ = 0.0;
            target_angular_z_ = 0.0;
        }

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = target_linear_x_ * current_speed;
        twist_msg->angular.z = target_angular_z_ * current_speed;
        
        pub_cmd_vel_->publish(std::move(twist_msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr cli_set_pen_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool boost_active_;
    rclcpp::Time boost_end_time_;
    double target_linear_x_;
    double target_angular_z_;
};

int main(int argc, char *argv[])
{
    save_terminal_settings();
    
    set_terminal_raw();

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ExtendedTeleop>();
    rclcpp::spin(node);
    
    restore_terminal_settings();
    rclcpp::shutdown();
    
    return 0;
}