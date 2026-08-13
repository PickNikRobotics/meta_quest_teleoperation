# meta_quest_teleoperation

A Meta Quest Unity app that publishes controller poses and button state to ROS 2 over [Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Endpoint). This can be used to provide smooth expert demonstrations for robotics manipulation models.

Pairs with the host-side `ros_tcp_endpoint` node and [MoveIt Pro clutch-based teleoperation](https://docs.picknik.ai/hardware_guides/input_devices/setting_up_the_meta_quest_for_teleop/) functionality.

## Connection

On launch the app reads the host IP from `PlayerPrefs` (saved from the previous successful session) and connects to `<host>:10000`. To change the IP, press **Left A** to open the on-screen keyboard, type the new address, and press **Done**. The new IP is saved and the connection is re-established.

The IP must point at the machine running the ROS 2 host (`ros2 run ros_tcp_endpoint default_server_endpoint`) — typically your MoveIt Pro host. Find it via `ip a` on Linux.

## Topics published

### Per-button event topics — `std_msgs/Empty`, fires once per press
For each button, an Empty message is published the moment it transitions from released to pressed using Quest's internal debounce filtering.

| Button | Right topic | Left topic |
|---|---|---|
| Grip | `/right_grip_button_event` | `/left_grip_button_event` |
| Trigger | `/right_trigger_button_event` | `/left_trigger_button_event` |
| A (primary) | `/right_a_button_event` | `/left_a_button_event` |
| B (secondary) | `/right_b_button_event` | `/left_b_button_event` |
| Menu | `/right_menu_button_event` | `/left_menu_button_event` |

### Per-button state topics — `std_msgs/Bool`, polled at 60 Hz
Continuous polling of `IsPressed()` for each button. Use these when you need to query the live state.

Same five buttons × two controllers, with `_button_state` instead of `_button_event` (e.g. `/right_grip_button_state`).

### Controller pose — `nav_msgs/Odometry`, polled at 60 Hz
- `/right_controller_odom` — right controller pose in the `headset` frame, child `right_controller_odom`
- `/left_controller_odom` — left controller pose in the `headset` frame, child `left_controller_odom`

Pose is relative to the current headset transform (`RosPublishers.headTransform` in the
Inspector, normally the XR rig's Main Camera) rather than Quest's boot/recenter-time tracking
origin, so it stays valid regardless of where the operator physically stands or whether they've
recentered. The twist field is zero (only pose is reported).

### TF — `tf2_msgs/TFMessage`, polled at 60 Hz
- `/tf` — both controllers as `headset → {left,right}_controller_odom` transforms

### Joystick — `geometry_msgs/Vector3`, polled at 60 Hz
- `/left_joystick` — left thumbstick, `x=horizontal`, `y=vertical`, `z=0`
- `/right_joystick` — right thumbstick, `x=horizontal`, `y=vertical`, `z=0`

Thumbstick button/touch topics:

- `/left_joystick_click_event`, `/right_joystick_click_event` — `std_msgs/Empty`, fires on click
- `/left_joystick_click_state`, `/right_joystick_click_state` — `std_msgs/Bool`
- `/left_joystick_touch_state`, `/right_joystick_touch_state` — `std_msgs/Bool`

## Special-purpose buttons

| Button | Behavior |
|---|---|
| **Left A** | Open the on-screen keyboard to change the host IP |
| **Right A** | Plays a "starting demonstration" audio cue locally (in addition to `/right_a_button_event`) |
| **Right B** | Plays a "stopping demonstration" audio cue locally (in addition to `/right_b_button_event`) |

All other buttons only fire their event/state topics.

The audio cues exist to support expert-demonstration recording workflows for VLA / imitation-learning pipelines, where a wearer needs an in-headset confirmation that a take started or ended. The cues are purely local — the app does not publish `/demonstration_indicator` itself. If your host-side workflow wants that topic, drive it from the behavior tree off `/right_a_button_event` and `/right_b_button_event` (the v11 teleop tree in `lab_sim` does this with a `PublishString` action — copy that pattern if you need a different topic name or different cue text).

## How teleoperation works

This app **does not implement clutch logic itself** — it just publishes raw pose and button state. Clutch behavior (snap initial poses, apply controller delta to the EE) lives on the ROS 2 host in the MoveIt Pro Objective tree (`Teleop With Meta via Pose v11`). The grip button event/state combination is what that Objective uses to enter and exit a teleop session.

For host-side setup and end-to-end usage, see the MoveIt Pro docs.

## Known limitations

### Spurious "Not registered to publish topic" errors after a `ros_tcp_endpoint` restart

If the host-side `ros_tcp_endpoint` restarts (for example, restarting the MoveIt Pro drivers container) while the Quest app stays running, the host log will briefly emit a burst of

```
[ros_tcp_endpoint] [ERROR] Not registered to publish topic '/<topic>'! Valid publish topics are: dict_keys([])
```

one per published topic, lasting on the order of 100–300 ms.

**The errors are cosmetic.** Publishes that arrive before the new registration are dropped host-side; once registrations land (a few hundred ms later) every subsequent publish goes through cleanly. No teleop control loop is affected.

The first connect after the Quest app launches is unaffected — `RosPublishers.cs` holds publishes in `Update()` for 2 s after `Connect()` so the endpoint has time to process the registrations we send up front. Reconnects are not gated because they happen on the Connector's background `ConnectionThread`.

## After launching

On first launch you should see a screen that looks like the following image. The IP field shows the "Enter ROS PC IP" placeholder until you press **Left A** to open the on-screen keyboard and type your host's IP — once entered, the value is saved and reused on subsequent launches.

![ROS Teleop startup screen showing the IP entry placeholder text](images/startup.png)
