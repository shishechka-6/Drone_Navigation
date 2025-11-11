#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("octomap_publisher");
    auto pub = node->create_publisher<octomap_msgs::msg::Octomap>("/octomap_full", 1);

    // Загрузка OctoMap из файла
    std::string map_path = "/home/ros2_ws/src/fr_079.ot"; // Укажите путь к файлу карты
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(map_path));
    if (!octree) {
        RCLCPP_ERROR(node->get_logger(), "Не удалось загрузить карту из файла");
        return -1;
    }

    // Преобразование OctoMap в ROS-сообщение
    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = "map"; // Укажите правильный фрейм
    msg.header.stamp = node->now();
    if (!octomap_msgs::fullMapToMsg(*octree, msg)) {
        RCLCPP_ERROR(node->get_logger(), "Ошибка преобразования OctoMap в сообщение");
        return -1;
    }

    // Публикация сообщения
    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "OctoMap опубликована в топике /octomap");

//    rclcpp::spin(node);
    rclcpp::WallRate loop_rate(1); // Публикуем с частотой 1 Гц (раз в секунду)
    while (rclcpp::ok()) {
    	msg.header.stamp = node->now(); // Обновляем временную метку
    	pub->publish(msg);
    	RCLCPP_INFO(node->get_logger(), "OctoMap опубликована (периодически)");
    	rclcpp::spin_some(node);
    	loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
