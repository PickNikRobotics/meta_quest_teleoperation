using System;
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
    public ROSConnection ros;
    // Event updates: Publish "WasPressedThisFrame()" button updates the frame they are detected
    public string demonstrationIndicatorTopic = "/demonstration_indicator";
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
    public string rightOdomTopicName = "/right_controller_odom";
    public string rightChildFrame = "right_controller_odom";
    public string rightGripButtonStateTopicName = "/right_grip_button_state";
    public string rightTriggerButtonStateTopicName = "/right_trigger_button_state";
    public string rightAButtonStateTopicName = "/right_a_button_state";
    public string rightBButtonStateTopicName = "/right_b_button_state";
    public string rightMenuButtonStateTopicName = "/right_menu_button_state";
    public InputActionAsset inputActions;

    public GameObject leftController;
    public GameObject rightController;

    private float _timeElapsed;
    private InputAction _startDemoAction;
    private InputAction _stopDemoAction;
    private InputAction _keyboardAction;
    private InputAction _leftGripAction;
    private InputAction _leftTriggerAction;
    private InputAction _leftAAction;
    private InputAction _leftBAction;
    private InputAction _leftMenuAction;
    private InputAction _rightGripAction;
    private InputAction _rightTriggerAction;
    private InputAction _rightAAction;
    private InputAction _rightBAction;
    private InputAction _rightMenuAction;
    private bool _grippedState;

    private Pose _clutchTransformRight;
    private Pose _clutchTransformLeft;
    private Pose _currentTransformRight;
    private Pose _currentTransformLeft;
    private Pose _currentDiffTransformRight;
    private Pose _currentDiffTransformLeft;
    private Pose _tmpTransformInverted;
    private Pose _tmpTransform;

    private AudioSource _startDemoAudioData;
    private AudioSource _stopDemoAudioData;

    private TouchScreenKeyboard _keyboard;
    public TextMeshProUGUI textInput;

    private void DoTransform(Pose transformLhs, Pose transformRhs, out Pose newTransform)
    {
        newTransform = transformRhs.GetTransformedBy(transformLhs);
    }

    private void DoTransformDiff(Pose transformCurrent, Pose transformDiff, Pose transformClutchIn,
        ref Pose newTransform)
    {
        newTransform.rotation = (transformClutchIn.rotation * transformDiff.rotation *
                                 Quaternion.Inverse(transformClutchIn.rotation)) * transformCurrent.rotation;
        newTransform.position = transformCurrent.position +
                                transformClutchIn.rotation * transformDiff.position;
    }

    private void InvertTransform(Pose transformBase, ref Pose newTransform)
    {
        newTransform.rotation = Quaternion.Inverse(transformBase.rotation);
        newTransform.position = -(newTransform.rotation * transformBase.position);
    }

    private void SetPoseFromTransform(Transform transformValue, ref Pose pose)
    {
        transformValue.GetPositionAndRotation(out pose.position, out pose.rotation);
    }


    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Disconnect();
        ros.Connect(PlayerPrefs.GetString("RosIPAddress", "127.0.0.1"), 10000);

        // Event updates
        ros.RegisterPublisher<StringMsg>(demonstrationIndicatorTopic);
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
        ros.RegisterPublisher<TFMessageMsg>("/tf_test");
        ros.RegisterPublisher<OdometryMsg>(leftOdomTopicName);
        ros.RegisterPublisher<BoolMsg>(leftGripButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftTriggerButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftAButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftBButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(leftMenuButtonStateTopicName);

        ros.RegisterPublisher<OdometryMsg>(rightOdomTopicName);
        ros.RegisterPublisher<BoolMsg>(rightGripButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightTriggerButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightAButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightBButtonStateTopicName);
        ros.RegisterPublisher<BoolMsg>(rightMenuButtonStateTopicName);
        
        // If not using this for MoveIt Pro's diffusion training pipeline, do not enable the StartDemo and StopDemo actions
        _startDemoAction = inputActions.FindAction("StartDemo");
        _startDemoAction.Enable();
        _stopDemoAction = inputActions.FindAction("StopDemo");
        _stopDemoAction.Enable();
        //
        _keyboardAction = inputActions.FindAction("OpenKeyboard");
        _keyboardAction.Enable();
        _leftGripAction = inputActions.FindAction("LeftGrip");
        _leftGripAction.Enable();
        _leftTriggerAction = inputActions.FindAction("LeftTrigger");
        _leftTriggerAction.Enable();
        _leftAAction = inputActions.FindAction("LeftA");
        _leftAAction.Enable();
        _leftBAction = inputActions.FindAction("LeftB");
        _leftBAction.Enable();
        _leftMenuAction = inputActions.FindAction("LeftMenu");
        _leftMenuAction.Enable();
        _rightGripAction = inputActions.FindAction("RightGrip");
        _rightGripAction.Enable();
        _rightTriggerAction = inputActions.FindAction("RightTrigger");
        _rightTriggerAction.Enable();
        _rightAAction = inputActions.FindAction("RightA");
        _rightAAction.Enable();
        _rightBAction = inputActions.FindAction("RightB");
        _rightBAction.Enable();
        _rightMenuAction = inputActions.FindAction("RightMenu");
        _rightMenuAction.Enable();


        // _currentTransformRight = new Pose();
        // _currentTransformLeft = new Pose();
        // _clutchTransformRight = new Pose();
        // _clutchTransformLeft = new Pose();
        // _currentDiffTransformRight = new Pose();
        // _currentDiffTransformLeft = new Pose();
        // _tmpTransformInverted = new Pose();
        // _tmpTransform = new Pose();

        // SetPoseFromTransform(rightController.transform, ref _currentTransformRight);
        // SetPoseFromTransform(leftController.transform, ref _currentTransformLeft);

        // _currentDiffTransformRight.rotation.Set(0, 0, 0, 1.0f);
        // _currentDiffTransformLeft.rotation.Set(0, 0, 0, 1.0f);

        _startDemoAudioData = GetComponents<AudioSource>()[0];
        _stopDemoAudioData = GetComponents<AudioSource>()[1];

        textInput = GameObject.Find("Text").GetComponent<TextMeshProUGUI>();
        textInput.text = ros.RosIPAddress;
    }


    public void Update()
    {
        //Event updates
        if (_keyboardAction.WasPressedThisFrame())
        {
            TouchScreenKeyboard.hideInput = false;
            _keyboard = TouchScreenKeyboard.Open("",
                TouchScreenKeyboardType.NumbersAndPunctuation, false, false, false, false);
        }

        if (!ros.RosIPAddress.Equals(textInput.text))
        {
            ros.Disconnect();
            ros.Connect(textInput.text, 10000);
            PlayerPrefs.SetString("RosIPAddress", ros.RosIPAddress);
        }

        if (_startDemoAction.WasPressedThisFrame())
        {
            _startDemoAudioData.Play(0);
            var msg = new StringMsg()
            {
                data = "Starting demonstration"
            };
            ros.Publish(demonstrationIndicatorTopic, msg);
        }

        if (_stopDemoAction.WasPressedThisFrame())
        {
            _stopDemoAudioData.Play(0);
            var msg = new StringMsg()
            {
                data = "Stopping demonstration"
            };
            ros.Publish(demonstrationIndicatorTopic, msg);
        }

        if (_leftGripAction.WasPressedThisFrame())
        {
            ros.Publish(leftGripButtonEventTopicName, new EmptyMsg());
        }
        if (_leftTriggerAction.WasPressedThisFrame())
        {
            ros.Publish(leftTriggerButtonEventTopicName, new EmptyMsg());
        }
        if (_leftAAction.WasPressedThisFrame())
        {
            ros.Publish(leftAButtonEventTopicName, new EmptyMsg());
        }
        if (_leftBAction.WasPressedThisFrame())
        {
            ros.Publish(leftBButtonEventTopicName, new EmptyMsg());
        }
        if (_leftMenuAction.WasPressedThisFrame())
        {
            ros.Publish(leftMenuButtonEventTopicName, new EmptyMsg());
        }
        if (_rightGripAction.WasPressedThisFrame())
        {
            ros.Publish(rightGripButtonEventTopicName, new EmptyMsg());
        }
        if (_rightTriggerAction.WasPressedThisFrame())
        {
            ros.Publish(rightTriggerButtonEventTopicName, new EmptyMsg());
        }
        if (_rightAAction.WasPressedThisFrame())
        {
            ros.Publish(rightAButtonEventTopicName, new EmptyMsg());
        }
        if (_rightBAction.WasPressedThisFrame())
        {
            ros.Publish(rightBButtonEventTopicName, new EmptyMsg());
        }
        if (_rightMenuAction.WasPressedThisFrame())
        {
            ros.Publish(rightMenuButtonEventTopicName, new EmptyMsg());
        }

        // if (_clutchAction.WasPressedThisFrame())
        // {
        //     SetPoseFromTransform(rightController.transform, ref _clutchTransformRight);
        //     SetPoseFromTransform(leftController.transform, ref _clutchTransformLeft);
        // }

        // if (_clutchAction.IsPressed())
        // {
        //     // We want to know the difference between the current transform and the clutch transform in the world frame
        //     InvertTransform(_clutchTransformRight, ref _tmpTransformInverted);
        //     SetPoseFromTransform(rightController.transform, ref _tmpTransform);
        //     DoTransform(_tmpTransformInverted, _tmpTransform, out _currentDiffTransformRight);

        //     InvertTransform(_clutchTransformLeft, ref _tmpTransformInverted);
        //     SetPoseFromTransform(leftController.transform, ref _tmpTransform);
            // DoTransform(_tmpTransformInverted, _tmpTransform, out _currentDiffTransformLeft);
        // }

        // if (_clutchAction.WasReleasedThisFrame())
        // {
        //     DoTransformDiff(_currentTransformRight, _currentDiffTransformRight, _clutchTransformRight,
        //         ref _tmpTransform);
        //     _currentTransformRight.position = _tmpTransform.position;
        //     _currentTransformRight.rotation = _tmpTransform.rotation;
        //     _currentDiffTransformRight.position.Set(0, 0, 0);
        //     _currentDiffTransformRight.rotation.Set(0, 0, 0, 1.0f);
        // }


        // State updates
        _timeElapsed += Time.deltaTime;
        if (_timeElapsed > odomPublishFrequency)
        {
            _timeElapsed = 0;

            PublishOdomAndTf(leftController.transform, leftChildFrame, leftOdomTopicName);
            PublishOdomAndTf(rightController.transform, rightChildFrame, rightOdomTopicName);

            ros.Publish(leftGripButtonStateTopicName, new BoolMsg(_leftGripAction.IsPressed()));
            ros.Publish(leftTriggerButtonStateTopicName, new BoolMsg(_leftTriggerAction.IsPressed()));
            ros.Publish(leftAButtonStateTopicName, new BoolMsg(_leftAAction.IsPressed()));
            ros.Publish(leftBButtonStateTopicName, new BoolMsg(_leftBAction.IsPressed()));
            ros.Publish(leftMenuButtonStateTopicName, new BoolMsg(_leftMenuAction.IsPressed()));


            ros.Publish(rightGripButtonStateTopicName, new BoolMsg(_rightGripAction.IsPressed()));
            ros.Publish(rightTriggerButtonStateTopicName, new BoolMsg(_rightTriggerAction.IsPressed()));
            ros.Publish(rightAButtonStateTopicName, new BoolMsg(_rightAAction.IsPressed()));
            ros.Publish(rightBButtonStateTopicName, new BoolMsg(_rightBAction.IsPressed()));
            ros.Publish(rightMenuButtonStateTopicName, new BoolMsg(_rightMenuAction.IsPressed()));
        
        }
        
    }
    
    void OnGUI()
    {
        if (_keyboard != null)
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
    // private void PublishOdomAndTf(Transform transformValue, ref Pose pose)
    // {
    //     Pose pose = new Pose();
    //     transformValue.GetPositionAndRotation(pose.position, pose.rotation);
    private void PublishOdomAndTf(Transform transform, string childFrame, string odomTopicName)
    {
    // // Convert position and rotation using ROSGeometry
    // DoTransformDiff(_currentTransformRight, _currentDiffTransformRight, _clutchTransformRight, ref _tmpTransform);
        
        Pose tempPose = new Pose();
        // Transform transform = rightController.transform;
        transform.GetPositionAndRotation(out tempPose.position, out tempPose.rotation);
        
        Vector3<FLU> rosPosition = CoordinateSpaceExtensions.To<FLU>(tempPose.position);
        Quaternion<FLU> rosRotation = CoordinateSpaceExtensions.To<FLU>(tempPose.rotation);

        // Create header
        HeaderMsg header = new HeaderMsg
        {
            frame_id = "quest",
            stamp = GetRosTime()
        };

        var pose = new PoseWithCovarianceMsg
        {
            pose = new PoseMsg
            {
                position = rosPosition.To<FLU>(),
                orientation = rosRotation.To<FLU>()
            }
        };
        var twist = new TwistWithCovarianceMsg
        {
            twist = new TwistMsg
            {
                linear = new Vector3Msg(0, 0, 0), // Assuming no linear velocity for simplicity
                angular = new Vector3Msg(0, 0, 0) // Assuming no angular velocity for simplicity
            }
        };

        var odometryMsg = new OdometryMsg()
        {
            header = header,
            child_frame_id = childFrame,
            pose = pose,
            twist = twist
        };

        //Publish the message
        ros.Publish(odomTopicName, odometryMsg);


        // Create transform
        var transformMsg = new TransformMsg
        {
            translation = rosPosition.To<FLU>(),
            rotation = rosRotation.To<FLU>()
        };

        // Create transform stamped
        var transformStamped = new TransformStampedMsg
        {
            header = header,
            child_frame_id = childFrame,
            transform = transformMsg
        };

        // Wrap in TFMessage
        var transforms = new[] { transformStamped };
        var tfMessage = new TFMessageMsg(transforms);

        // Publish the message
        ros.Publish(tfTopicName, tfMessage);
        ros.Publish("/tf_test", tfMessage);
    }
}