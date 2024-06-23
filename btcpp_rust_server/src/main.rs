use std::default;

use futures::stream::Stream;
use futures::StreamExt;
use futures::future::{self, Either};
use r2r::btcpp_ros2_interfaces::msg::NodeStatus;
use r2r::{ActionServerGoal, ParameterValue, QosProfile, Node};
use r2r::ur_controller_msgs::action::UniversalRobotControl;
use r2r::btcpp_ros2_interfaces::action::ExecuteAction;
use rand::Rng;

pub static NODE_ID: &'static str = "btcpp_rust_server";

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let action = node.create_action_server::<ExecuteAction::Action>("bt_action_service")?;

    let ur_client = node.create_action_client::<UniversalRobotControl::Action>("ur_control")?;
    let waiting_for_ur_server = Node::is_available(&ur_client)?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_warn!(NODE_ID, "Waiting for UR Control Service...");
    waiting_for_ur_server.await?;
    r2r::log_info!(NODE_ID, "UR Control Service available.");

    tokio::task::spawn(async move {
        let result =
        server(action, &ur_client)
                .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "BT action service call succeeded."),
            Err(e) => {
                r2r::log_error!(NODE_ID, "BT action service call failed with: {}.", e)
            }
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())

}

async fn server(
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<ExecuteAction::Action>> + Unpin,
    ur_client: &r2r::ActionClient<UniversalRobotControl::Action>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match requests.next().await {
            Some(request) => {
                let (mut g, mut _cancel) =
                    request.accept().expect("Could not accept goal request.");
                let g_clone = g.clone();

                // match g_clone.goal.

                match execute_ur_control(g_clone, &ur_client).await {
                    Ok(result) => g.succeed(ExecuteAction::Result { success: result, return_message: "adsf".to_string() }).expect("Could not send result."),
                    Err(_) => g.abort(ExecuteAction::Result { success: false, return_message: "asdf".to_string() }).expect("Could not abort."),
                }
            }
            None => (),
        }
    }
}


async fn execute_ur_control(
    g:ActionServerGoal<ExecuteAction::Action>,
    client: &r2r::ActionClient<UniversalRobotControl::Action>
) -> Result<bool, Box<dyn std::error::Error>> {

    let mut robot_goal = UniversalRobotControl::Goal::default();
    robot_goal.goal_feature_id = g.goal.command.to_string();

    // robot_goal.json = g.goal.command.to_string();
    robot_goal.tcp_id = "tool0".to_string();
    robot_goal.command = "move_j".to_string();
    robot_goal.velocity = 0.2;
    robot_goal.acceleration = 0.2;

    // r2r::log_info!(NODE_ID, "Sending request to the Universal Robot Controller.");
    // let _ = g.publish_feedback(ExecuteAction::Feedback {
    //     node_status: NodeStatus {
    //         status: 1
    //     }, // try to sort this out
    //     message: "Sending request to Univrsal Robot Controller.".into(),
    // });

    let (_goal, result, mut feedback) = match client.send_goal_request(robot_goal) {
        Ok(x) => match x.await {
            Ok(y) => y,
            Err(e) => {
                r2r::log_info!(NODE_ID, "Could not send goal request.");
                return Err(Box::new(e));
            }
        },
        Err(e) => {
            r2r::log_info!(NODE_ID, "Did not get goal.");
            return Err(Box::new(e));
        }
    };

    // spawn task for propagating the feedback
    // let g_clone = g.clone();
    // tokio::spawn(async move {
    //     loop {
    //         if let Some(_fb) = feedback.next().await {
    //             // let passed_on = ExecuteAction::Feedback {
    //             //     node_status: NodeStatus {
    //             //         status: 1
    //             //     }, // try to sort this out
    //             //     message: "Sending request to Universal Robot Controller.".into(),
    //             // };
    //             if let Err(_) = g_clone.publish_feedback(passed_on) {
    //                 // could not publish, probably done...
    //                 break;
    //             }
    //         } else {
    //             // sender dropped, we are done.
    //             break;
    //         }
    //     }
    // });

    match result.await {
        Ok((status, msg)) => match status {
            r2r::GoalStatus::Aborted => {
                r2r::log_info!(NODE_ID, "Goal succesfully aborted with: {:?}", msg);
                Ok(msg.success)
            }
            _ => {
                r2r::log_info!(
                    NODE_ID,
                    "Executing the Simple Robot Simulator action succeeded."
                );
                Ok(msg.success)
            }
        },
        Err(e) => {
            r2r::log_error!(
                NODE_ID,
                "Simple Robot Simulator action failed with: {:?}",
                e,
            );
            return Err(Box::new(e));
        }
    }
}
