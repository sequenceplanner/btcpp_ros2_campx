use std::default;

use futures::stream::Stream;
use futures::StreamExt;
use futures::future::{self, Either};
use r2r::{ActionServerGoal, ParameterValue, QosProfile, Node};
use r2r::ur_controller_msgs::action::UniversalRobotControl;
use rand::Rng;

pub static NODE_ID: &'static str = "btcpp_rust_server";

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    let action = node.create_action_server::<UniversalRobotControl::Action>("sleep_service")?;

    let ur_client = node.create_action_client::<UniversalRobotControl::Action>("ur_control")?;
    let waiting_for_ur_server = Node::is_available(&ur_client)?;

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    // THIS MAKES THE BT EXEC FAIL! WHY?
    r2r::log_warn!(NODE_ID, "Waiting for extra service...");
    waiting_for_ur_server.await?;
    r2r::log_info!(NODE_ID, "extra Service available.");

    tokio::task::spawn(async move {
        let result =
        server(action, &ur_client)
                .await;
        match result {
            Ok(()) => r2r::log_info!(NODE_ID, "BT plugin trigger call succeeded."),
            Err(e) => {
                r2r::log_error!(NODE_ID, "BT plugin trigger call failed with: {}.", e)
            }
        };
    });

    handle.join().unwrap();

    r2r::log_warn!(NODE_ID, "Node started.");

    Ok(())

}

async fn server(
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<UniversalRobotControl::Action>> + Unpin,
    client: &r2r::ActionClient<UniversalRobotControl::Action>,
    // prefix: &str
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match requests.next().await {
            Some(request) => {
                let (mut g, mut _cancel) =
                    request.accept().expect("Could not accept goal request.");
                let g_clone = g.clone();

                let delay: u64 = {
                    let mut rng = rand::thread_rng();
                    rng.gen_range(0..3000)
                };

                // simulate random task execution time
                tokio::time::sleep(std::time::Duration::from_millis(delay)).await;

                // g.succeed(UniversalRobotControl::Result { success: true }).expect("Could not send result.");

                match execute_ur_control(g_clone, &client).await {
                    Ok(result) => g.succeed(UniversalRobotControl::Result { success: result }).expect("Could not send result."),
                    Err(_) => g.abort(UniversalRobotControl::Result { success: false }).expect("Could not abort."),
                }
            }
            None => (),
        }
    }
}


async fn execute_ur_control(
    g: ActionServerGoal<UniversalRobotControl::Action>,
    client: &r2r::ActionClient<UniversalRobotControl::Action>
) -> Result<bool, Box<dyn std::error::Error>> {
    let mut goal = g.clone();
    goal.goal.goal_feature_id = g.goal.command.to_string();
    goal.goal.tcp_id = "tool0".to_string();
    goal.goal.command = "move_j".to_string();
    goal.goal.velocity = 0.2;
    goal.goal.acceleration = 0.2;

    r2r::log_info!(NODE_ID, "Sending request to the Robot Controller.");
    let _ = g.publish_feedback(UniversalRobotControl::Feedback {
        current_state: "Sending request to Robot Controller.".into(),
    });

    let (_goal, result, mut feedback) = match client.send_goal_request(goal.goal) {
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
    let g_clone = g.clone();
    tokio::spawn(async move {
        loop {
            if let Some(fb) = feedback.next().await {
                let passed_on = UniversalRobotControl::Feedback {
                    current_state: fb.current_state,
                };
                if let Err(_) = g_clone.publish_feedback(passed_on) {
                    // could not publish, probably done...
                    break;
                }
            } else {
                // sender dropped, we are done.
                break;
            }
        }
    });

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
