#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cr_interfaces/action/pick.hpp>
#include <cr_interfaces/action/place.hpp>

namespace cr {
namespace task_orchestration {

    class TaskOrchestrator : public rclcpp::Node {
    
    public:
        
        explicit TaskOrchestrator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        void send_pick_goal();
        void send_place_goal();

    private:

        using Pick = cr_interfaces::action::Pick;
        using GoalHandlePick = rclcpp_action::ClientGoalHandle<Pick>;
        
        using Place = cr_interfaces::action::Place;
        using GoalHandlePlace = rclcpp_action::ClientGoalHandle<Place>;

        rclcpp_action::Client<Pick>::SharedPtr pick_client_ptr_;
        rclcpp_action::Client<Place>::SharedPtr place_client_ptr_;

        // Callback per gestione della risposta dal server
        void pick_goal_response_callback(const GoalHandlePick::SharedPtr & goal_handle);
        void place_goal_response_callback(const GoalHandlePlace::SharedPtr & goal_handle);

        // Callback per la gestione dei feedback
        void pick_feedback_callback(GoalHandlePick::SharedPtr, const std::shared_ptr<const Pick::Feedback> feedback);
        void place_feedback_callback(GoalHandlePlace::SharedPtr, const std::shared_ptr<const Place::Feedback> feedback);

        // Callback per la gestione dei result
        void pick_result_callback(const GoalHandlePick::WrappedResult & result);
        void place_result_callback(const GoalHandlePlace::WrappedResult & result);

    };

} // namespace task_orchestrator
} // namespace cr