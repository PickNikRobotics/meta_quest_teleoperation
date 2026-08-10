using System;
using System.Collections;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using TMPro;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine.InputSystem;

public class RosPublishers : MonoBehaviour
{
    private const string kHeadsetFrameId = "headset";

    public ROSConnection ros;
    // Event updates: Publish "WasPressedThisFrame()" button updates the frame they are detected.
    // /demonstration_indicator is published from the host-side behavior tree off the Right A / Right B
    // button events — this app only emits the raw button events and plays the local audio cues.
    public string leftGripButtonEventTopicName = "/left_grip_button_event";
    public string leftTriggerButtonEventTopicName = "/left_trigger_button_event";
    // x
    public string leftAButtonEventTopicName = "/left_a_button_event";
    // y
    public string leftBButtonEventTopicName = "/left_b_button_event";
    public string leftMenuButtonEventTopicName = "/left_menu_button_event";
    public string rightGripButtonEventTopicName = "/right_grip_button_event";
    public string rightTriggerButtonEventTopicName = "/right_trigger_button_event";
    public string rightAButtonEventTopicName = "/right_a_button_event";
    public string rightBButtonEventTopicName = "/right_b_button_event";
    public string rightMenuButtonEventTopicName = "/right_menu_button_event";
    // State updates: Publish odom and "isPressed()" updates at the below rate
    public float odomPublishFrequency = 1.0f / 60.0f;
    public string tfTopicName = "/tf";
    public string leftOdomTopicName = "/left_controller_odom";
    public string leftChildFrame = "left_controller_odom";
    public string leftGripButtonStateTopicName = "/left_grip_button_state";
    public string leftTriggerButtonStateTopicName = "/left_trigger_button_state";
    public string leftAButtonStateTopicName = "/left_a_button_state";
    public string leftBButtonStateTopicName = "/left_b_button_state";
    public string leftMenuButtonStateTopicName = "/left_menu_button_state";
    public string leftJoystickTopicName = "/left_joystick";
    public string leftJoystickClickEventTopicName = "/left_joystick_click_event";
    public string leftJoystickClickStateTopicName = "/left_joystick_click_state";
    public string leftJoystickTouchStateTopicName = "/left_joystick_touch_state";
    public string rightOdomTopicName = "/right_controller_odom";
    public string rightChildFrame = "right_controller_odom";
    public string rightGripButtonStateTopicName = "/right_grip_button_state";
    public string rightTriggerButtonStateTopicName = "/right_trigger_button_state";
    public string rightAButtonStateTopicName = "/right_a_button_state";
    public string rightBButtonStateTopicName = "/right_b_button_state";
    public string rightMenuButtonStateTopicName = "/right_menu_button_state";
    public string rightJoystickTopicName = "/right_joystick";
    public string rightJoystickClickEventTopicName = "/right_joystick_click_event";
    public string rightJoystickClickStateTopicName = "/right_joystick_click_state";
    public string rightJoystickTouchStateTopicName = "/right_joystick_touch_state";
    public InputActionAsset inputActions;

    public GameObject leftController;
    public GameObject rightController;
    // Reference frame for the published controller poses. Reporting pose relative to the
    // current headset transform (instead of Quest's boot/recenter-time tracking origin) makes
    // the reference frame the operator's own head -- stable regardless of where they physically
    // stand or whether they've recentered. Assign the XR rig's "Main Camera" in the Inspector.
    public Transform headTransform;

    private float _timeElapsed;
    private InputAction _startDemoAction;
    private InputAction _stopDemoAction;
    private InputAction _keyboardAction;
    private InputAction _leftGripAction;
    private InputAction _leftTriggerAction;
    private InputAction _leftAAction;
    private InputAction _leftBAction;
    private InputAction _leftMenuAction;
    private InputAction _leftJoystickAction;
    private InputAction _leftJoystickClickAction;
    private InputAction _leftJoystickTouchAction;
    private InputAction _rightGripAction;
    private InputAction _rightTriggerAction;
    private InputAction _rightAAction;
    private InputAction _rightBAction;
    private InputAction _rightMenuAction;
    private InputAction _rightJoystickAction;
    private InputAction _rightJoystickClickAction;
    private InputAction _rightJoystickTouchAction;
    // Registration gate: ros_tcp_endpoint has no registration ACK, so RegisterPublisher
    // commands race against subsequent Publish commands over separate TCP connections.
    // We block all Publish calls in Update() until this flag flips, giving the endpoint
    // time to process every RegisterPublisher. Reset on every (re)connect.
    private bool _registered;
    private Coroutine _registrationGateCoroutine;
    private const float kRegistrationDelaySeconds = 2.0f;

    // Reusable message instances for the 60 Hz publish path. Allocating fresh OdometryMsg /
    // PoseMsg / TFMessageMsg trees on every frame creates ~7,200 GC events/sec across both
    // controllers. We instantiate once per controller and mutate in place.
    //
    // ROSConnection.Publish() does NOT serialize synchronously: it only enqueues a reference
    // to the message object, and the actual serialization happens later on ROSConnection's
    // background ConnectionThread. Sharing one set of message objects between the left and
    // right publish calls is therefore unsafe -- mutating them for the right controller before
    // the background thread serializes the left controller's queued message overwrites the
    // left message with the right controller's data. Each controller gets its own instance.
    private class OdomTfState
    {
        public readonly HeaderMsg header;
        public readonly PointMsg posePoint;
        public readonly QuaternionMsg poseQuat;
        public readonly Vector3Msg twistLinear;
        public readonly Vector3Msg twistAngular;
        public readonly Vector3Msg joystick;
        public readonly OdometryMsg odom;
        public readonly Vector3Msg tfTranslation;
        public readonly QuaternionMsg tfRotation;
        public readonly TransformStampedMsg tfStamped;
        public readonly TFMessageMsg tfMessage;

        public OdomTfState()
        {
            header = new HeaderMsg { frame_id = kHeadsetFrameId };
            posePoint = new PointMsg();
            poseQuat = new QuaternionMsg();
            PoseMsg pose = new PoseMsg { position = posePoint, orientation = poseQuat };
            PoseWithCovarianceMsg poseWithCov = new PoseWithCovarianceMsg { pose = pose };
            twistLinear = new Vector3Msg(0, 0, 0);
            twistAngular = new Vector3Msg(0, 0, 0);
            joystick = new Vector3Msg(0, 0, 0);
            TwistMsg twist = new TwistMsg { linear = twistLinear, angular = twistAngular };
            TwistWithCovarianceMsg twistWithCov = new TwistWithCovarianceMsg { twist = twist };
            odom = new OdometryMsg { header = header, pose = poseWithCov, twist = twistWithCov };

            tfTranslation = new Vector3Msg();
            tfRotation = new QuaternionMsg();
            TransformMsg tfTransform = new TransformMsg { translation = tfTranslation, rotation = tfRotation };
            tfStamped = new TransformStampedMsg { header = header, transform = tfTransform };
            tfMessage = new TFMessageMsg(new TransformStampedMsg[1] { tfStamped });
        }
    }

    private OdomTfState _leftOdomTf;
    private OdomTfState _rightOdomTf;

    private AudioSource _startDemoAudioData;
    private AudioSource _stopDemoAudioData;
    private TouchScreenKeyboard _keyboard;
    public TextMeshProUGUI textInput;

    private void InitializeReusableMessages()
    {
        _leftOdomTf = new OdomTfState();
        _rightOdomTf = new OdomTfState();
    }

    private void RegisterAllPublishers()
    {
        // QoS policy: publishers are RELIABLE (the ROS-TCP-Connector default). Subscribers
        // are free to opt into BEST_EFFORT for low-latency consumption — ROS 2 QoS
        // compatibility rules allow a BEST_EFFORT subscriber to connect to a RELIABLE
        // publisher (the connection downgrades), so this default leaves the choice to the
        // host side without losing reliable delivery for subscribers that want it.

        // Event updates
        ros.RegisterPublisher<EmptyMsg>(leftGripButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(leftTriggerButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(leftAButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(leftBButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(leftMenuButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(rightGripButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(rightTriggerButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(rightAButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(rightBButtonEventTopicName);
        ros.RegisterPublisher<EmptyMsg>(rightMenuButtonEventTopicName);

        // State updates
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName);
        ros.RegisterPublisher<OdometryMsg>(leftOdomTopicName);
        ros.RegisterPublisher<BoolMsg>(leftGripButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftTriggerButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftAButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftBButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftMenuButtonStateTopicName);
        ros.RegisterPublisher<Vector3Msg>(leftJoystickTopicName);
        ros.RegisterPublisher<EmptyMsg>(leftJoystickClickEventTopicName);
        ros.RegisterPublisher<BoolMsg>(leftJoystickClickStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftJoystickTouchStateTopicName);

        ros.RegisterPublisher<OdometryMsg>(rightOdomTopicName);
        ros.RegisterPublisher<BoolMsg>(rightGripButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightTriggerButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightAButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightBButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightMenuButtonStateTopicName);
        ros.RegisterPublisher<Vector3Msg>(rightJoystickTopicName);
        ros.RegisterPublisher<EmptyMsg>(rightJoystickClickEventTopicName);
        ros.RegisterPublisher<BoolMsg>(rightJoystickClickStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightJoystickTouchStateTopicName);

    }

    private void ConnectAndRegister(string ipAddress)
    {
        ros.Disconnect();
        ros.Connect(ipAddress, 10000);
        RegisterAllPublishers();

        // Block publishes until the endpoint has had time to process every
        // RegisterPublisher. Coroutine flips _registered to true after the delay.
        _registered = false;
        if (_registrationGateCoroutine != null)
        {
            StopCoroutine(_registrationGateCoroutine);
        }
        _registrationGateCoroutine = StartCoroutine(MarkRegisteredAfterDelay());
    }

    private IEnumerator MarkRegisteredAfterDelay()
    {
        yield return new WaitForSeconds(kRegistrationDelaySeconds);
        _registered = true;
    }

    private InputAction FindAndEnableAction(string actionName)
    {
        InputAction action = inputActions.FindAction(actionName);
        if (action == null)
        {
            Debug.LogError($"RosPublishers: InputAction '{actionName}' not found in {nameof(inputActions)}.");
            return null;
        }
        action.Enable();
        return action;
    }

    public void Start()
    {
        InitializeReusableMessages();

        if (headTransform == null)
        {
            Debug.LogError($"RosPublishers: {nameof(headTransform)} is not assigned. Odom/TF publishing will throw a NullReferenceException.");
        }

        ros = ROSConnection.GetOrCreateInstance();
        // Only attempt to connect when the user has saved an IP from a prior session.
        // Skipping ConnectAndRegister on first launch avoids burning the registration
        // gate against an unreachable host and keeps the scene's "Enter ROS PC IP"
        // placeholder text visible until the user opens the keyboard.
        if (PlayerPrefs.HasKey("RosIPAddress"))
        {
            ConnectAndRegister(PlayerPrefs.GetString("RosIPAddress"));
        }

        // If not using this for MoveIt Pro's diffusion training pipeline, do not enable the StartDemo and StopDemo actions.
        _startDemoAction = FindAndEnableAction("StartDemo");
        _stopDemoAction = FindAndEnableAction("StopDemo");

        _keyboardAction = FindAndEnableAction("OpenKeyboard");
        _leftGripAction = FindAndEnableAction("LeftGrip");
        _leftTriggerAction = FindAndEnableAction("LeftTrigger");
        _leftAAction = FindAndEnableAction("LeftA");
        _leftBAction = FindAndEnableAction("LeftB");
        _leftMenuAction = FindAndEnableAction("LeftMenu");
        _leftJoystickAction = FindAndEnableAction("LeftJoystick");
        _leftJoystickClickAction = FindAndEnableAction("LeftJoystickClick");
        _leftJoystickTouchAction = FindAndEnableAction("LeftJoystickTouch");
        _rightGripAction = FindAndEnableAction("RightGrip");
        _rightTriggerAction = FindAndEnableAction("RightTrigger");
        _rightAAction = FindAndEnableAction("RightA");
        _rightBAction = FindAndEnableAction("RightB");
        _rightMenuAction = FindAndEnableAction("RightMenu");
        _rightJoystickAction = FindAndEnableAction("RightJoystick");
        _rightJoystickClickAction = FindAndEnableAction("RightJoystickClick");
        _rightJoystickTouchAction = FindAndEnableAction("RightJoystickTouch");

        AudioSource[] audioSources = GetComponents<AudioSource>();
        if (audioSources.Length >= 2)
        {
            _startDemoAudioData = audioSources[0];
            _stopDemoAudioData = audioSources[1];
        }
        else
        {
            Debug.LogError($"RosPublishers: expected at least 2 AudioSource components on this GameObject, found {audioSources.Length}. Demo start/stop sounds disabled.");
        }

        GameObject textGameObject = GameObject.Find("Text");
        if (textGameObject != null)
        {
            textInput = textGameObject.GetComponent<TextMeshProUGUI>();
        }
        if (textInput == null)
        {
            Debug.LogError("RosPublishers: GameObject 'Text' with TextMeshProUGUI component not found in scene. IP display disabled.");
        }
        else if (PlayerPrefs.HasKey("RosIPAddress"))
        {
            textInput.text = ros.RosIPAddress;
        }
        // else: leave the scene's placeholder text (e.g. "Enter ROS PC IP") alone.
    }


    public void Update()
    {
        //Event updates
        if (_keyboardAction != null && _keyboardAction.WasPressedThisFrame())
        {
            TouchScreenKeyboard.hideInput = false;
            _keyboard = TouchScreenKeyboard.Open("",
                TouchScreenKeyboardType.NumbersAndPunctuation, false, false, false, false);
        }

        // Only reconnect when the user finishes editing (keyboard closed via Done,
        // LostFocus, or Canceled). Reconnecting per keystroke would tear down
        // ros_tcp_endpoint registrations on every character typed.
        if (_keyboard != null && textInput != null &&
            (_keyboard.status == TouchScreenKeyboard.Status.Done ||
             _keyboard.status == TouchScreenKeyboard.Status.LostFocus ||
             _keyboard.status == TouchScreenKeyboard.Status.Canceled))
        {
            if (_keyboard.status == TouchScreenKeyboard.Status.Done &&
                !string.IsNullOrEmpty(textInput.text) &&
                !ros.RosIPAddress.Equals(textInput.text))
            {
                ConnectAndRegister(textInput.text);
                PlayerPrefs.SetString("RosIPAddress", textInput.text);
            }
            _keyboard = null;
        }

        // Block publishes until ros_tcp_endpoint has had time to process every
        // RegisterPublisher we sent in ConnectAndRegister. Without this, Publish
        // commands race against RegisterPublisher commands over separate TCP
        // connections and the endpoint logs "Not registered to publish topic 'X'".
        if (!_registered)
        {
            return;
        }

        // Audio-only cues for start/stop. The host-side BT listens to the Right A / Right B
        // button events and publishes the matching string to /demonstration_indicator.
        if (_startDemoAction != null && _startDemoAction.WasPressedThisFrame())
        {
            if (_startDemoAudioData != null) _startDemoAudioData.Play(0);
        }

        if (_stopDemoAction != null && _stopDemoAction.WasPressedThisFrame())
        {
            if (_stopDemoAudioData != null) _stopDemoAudioData.Play(0);
        }

        PublishEventIfPressed(_leftGripAction, leftGripButtonEventTopicName);
        PublishEventIfPressed(_leftTriggerAction, leftTriggerButtonEventTopicName);
        PublishEventIfPressed(_leftAAction, leftAButtonEventTopicName);
        PublishEventIfPressed(_leftBAction, leftBButtonEventTopicName);
        PublishEventIfPressed(_leftMenuAction, leftMenuButtonEventTopicName);
        PublishEventIfPressed(_leftJoystickClickAction, leftJoystickClickEventTopicName);
        PublishEventIfPressed(_rightGripAction, rightGripButtonEventTopicName);
        PublishEventIfPressed(_rightTriggerAction, rightTriggerButtonEventTopicName);
        PublishEventIfPressed(_rightAAction, rightAButtonEventTopicName);
        PublishEventIfPressed(_rightBAction, rightBButtonEventTopicName);
        PublishEventIfPressed(_rightMenuAction, rightMenuButtonEventTopicName);
        PublishEventIfPressed(_rightJoystickClickAction, rightJoystickClickEventTopicName);

        // State updates: publish odom/TF and all bool button states at odomPublishFrequency.
        // Subtract the threshold instead of resetting to 0 so the actual rate tracks the
        // configured frequency rather than the frame rate.
        _timeElapsed += Time.deltaTime;
        if (_timeElapsed >= odomPublishFrequency)
        {
            _timeElapsed -= odomPublishFrequency;

            PublishOdomAndTf(_leftOdomTf, leftController.transform, leftChildFrame, leftOdomTopicName);
            PublishOdomAndTf(_rightOdomTf, rightController.transform, rightChildFrame, rightOdomTopicName);

            PublishBoolState(_leftGripAction, leftGripButtonStateTopicName);
            PublishBoolState(_leftTriggerAction, leftTriggerButtonStateTopicName);
            PublishBoolState(_leftAAction, leftAButtonStateTopicName);
            PublishBoolState(_leftBAction, leftBButtonStateTopicName);
            PublishBoolState(_leftMenuAction, leftMenuButtonStateTopicName);
            PublishJoystick(_leftOdomTf, _leftJoystickAction, leftJoystickTopicName);
            PublishBoolState(_leftJoystickClickAction, leftJoystickClickStateTopicName);
            PublishBoolState(_leftJoystickTouchAction, leftJoystickTouchStateTopicName);

            PublishBoolState(_rightGripAction, rightGripButtonStateTopicName);
            PublishBoolState(_rightTriggerAction, rightTriggerButtonStateTopicName);
            PublishBoolState(_rightAAction, rightAButtonStateTopicName);
            PublishBoolState(_rightBAction, rightBButtonStateTopicName);
            PublishBoolState(_rightMenuAction, rightMenuButtonStateTopicName);
            PublishJoystick(_rightOdomTf, _rightJoystickAction, rightJoystickTopicName);
            PublishBoolState(_rightJoystickClickAction, rightJoystickClickStateTopicName);
            PublishBoolState(_rightJoystickTouchAction, rightJoystickTouchStateTopicName);
        }
    }

    private void PublishEventIfPressed(InputAction action, string topic)
    {
        if (action != null && action.WasPressedThisFrame())
        {
            ros.Publish(topic, new EmptyMsg());
        }
    }

    private void PublishBoolState(InputAction action, string topic)
    {
        if (action != null)
        {
            ros.Publish(topic, new BoolMsg(action.IsPressed()));
        }
    }

    private void PublishJoystick(OdomTfState state, InputAction action, string topic)
    {
        Vector2 value = action != null ? action.ReadValue<Vector2>() : Vector2.zero;
        state.joystick.x = value.x;
        state.joystick.y = value.y;
        state.joystick.z = 0.0;
        ros.Publish(topic, state.joystick);
    }
    
    void OnGUI()
    {
        if (_keyboard != null && textInput != null)
        {
            textInput.text = _keyboard.text;
        }
    }

    private static TimeMsg GetRosTime()
    {
        DateTime unixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        DateTime now = DateTime.UtcNow;

        TimeSpan timeSinceEpoch = now - unixEpoch;
        long totalTicks = timeSinceEpoch.Ticks;
        long totalNanoseconds = totalTicks * 100;
        return new TimeMsg
        {
            sec = (int)(totalNanoseconds / 1_000_000_000),
            nanosec = (uint)(totalNanoseconds % 1_000_000_000)
        };
    }

    private void PublishOdomAndTf(OdomTfState state, Transform sourceTransform, string childFrame, string odomTopicName)
    {
        sourceTransform.GetPositionAndRotation(out Vector3 worldPosition, out Quaternion worldRotation);

        // Report the controller pose relative to the headset rather than Quest's
        // boot/recenter-time tracking origin. This makes the reference frame the operator's
        // own head -- stable regardless of where they physically stand or whether they've
        // recentered -- so headset->pelvis is a fixed anthropometric offset instead of a
        // per-session recenter-dependent calibration.
        Vector3 headRelativePosition = headTransform.InverseTransformPoint(worldPosition);
        Quaternion headRelativeRotation = Quaternion.Inverse(headTransform.rotation) * worldRotation;

        Vector3<FLU> rosPosition = CoordinateSpaceExtensions.To<FLU>(headRelativePosition);
        Quaternion<FLU> rosRotation = CoordinateSpaceExtensions.To<FLU>(headRelativeRotation);

        // Mutate this controller's own reusable instances in place -- each controller has its
        // own OdomTfState (see field declaration) so the left and right publish calls never
        // touch the same objects.
        // The published pose is in standard REP-103 FLU (X=forward, Y=left, Z=up). Consumers
        // with different conventions (e.g. MoveIt Pro IMarker EE convention) are expected to
        // apply their own change-of-basis on the host side.
        state.header.stamp = GetRosTime();

        state.posePoint.x = rosPosition.x;
        state.posePoint.y = rosPosition.y;
        state.posePoint.z = rosPosition.z;
        state.poseQuat.x = rosRotation.x;
        state.poseQuat.y = rosRotation.y;
        state.poseQuat.z = rosRotation.z;
        state.poseQuat.w = rosRotation.w;

        state.odom.child_frame_id = childFrame;
        ros.Publish(odomTopicName, state.odom);

        state.tfTranslation.x = rosPosition.x;
        state.tfTranslation.y = rosPosition.y;
        state.tfTranslation.z = rosPosition.z;
        state.tfRotation.x = rosRotation.x;
        state.tfRotation.y = rosRotation.y;
        state.tfRotation.z = rosRotation.z;
        state.tfRotation.w = rosRotation.w;

        state.tfStamped.child_frame_id = childFrame;
        ros.Publish(tfTopicName, state.tfMessage);
    }
}
