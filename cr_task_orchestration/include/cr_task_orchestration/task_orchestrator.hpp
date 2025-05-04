#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cr_interfaces/action/pick.hpp>
#include <cr_interfaces/action/place.hpp>
#include <cr_interfaces/action/execute_workflow.hpp>
#include <cr_interfaces/srv/get_object_info.hpp>
#include <cr_interfaces/msg/object_info.hpp>
#include <cr_interfaces/msg/freeze_scene.hpp>

namespace cr {
namespace task_orchestration {

    class TaskOrchestrator : public rclcpp::Node {
    
    public:

        using ExecuteWorkflow = cr_interfaces::action::ExecuteWorkflow;
        using GoalHandleExecuteWorkflow = rclcpp_action::ServerGoalHandle<ExecuteWorkflow>;

        using Pick = cr_interfaces::action::Pick;
        using GoalHandlePick = rclcpp_action::ClientGoalHandle<Pick>;
        
        using Place = cr_interfaces::action::Place;
        using GoalHandlePlace = rclcpp_action::ClientGoalHandle<Place>;
        
        explicit TaskOrchestrator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:

        rclcpp_action::Server<ExecuteWorkflow>::SharedPtr execute_workflow_server_ptr_;
        rclcpp_action::Client<Pick>::SharedPtr pick_client_ptr_;
        rclcpp_action::Client<Place>::SharedPtr place_client_ptr_;

        rclcpp::Client<cr_interfaces::srv::GetObjectInfo>::SharedPtr get_object_info_client_;

        rclcpp::Publisher<cr_interfaces::msg::FreezeScene>::SharedPtr freeze_scene_pub_;

        bool is_busy_ = false;
        cr_interfaces::msg::ObjectInfo target_object_;
        std::shared_ptr<GoalHandleExecuteWorkflow> current_goal_handle_;

        // Metodo per richiedere le informazioni di uno specifico oggetto
        void get_object_info(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

        void send_pick_goal();
        void send_place_goal();

        // Callbacks lato server
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteWorkflow::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleExecuteWorkflow> goal_handle);

        // Callbacks lato client
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