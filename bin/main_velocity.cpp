#include "ref_model_velocity.hpp"

int main (int argc, char* argv[]) {
    // force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf (stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init (argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    std::shared_ptr<ReferenceModelVelocity> lc_node =
    std::make_shared<ReferenceModelVelocity> ("reference_model_velocity");
    exe.add_node (lc_node->get_node_base_interface ());
    exe.spin ();
    rclcpp::shutdown ();
    return 0;
}
