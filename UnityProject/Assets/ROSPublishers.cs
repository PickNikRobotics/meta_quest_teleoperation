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
    public string topicName = "/right_controller_odom";
    public string childFrame = "right_controller_odom";
    public string tfTopicName = "/tf";
    public string gripperButtonTopicName = "/gripper_button";
    public string demonstrationIndicatorTopic = "/demonstration_indicator";
    public float publishFrequency = 1.0f / 60.0f;
    public InputActionAsset inputActions;

    public GameObject rightController;
    public GameObject leftController;

    private float _timeElapsed;
    private InputAction _clutchAction;
    private InputAction _gripperAction;
    private InputAction _startDemoAction;
    private InputAction _stopDemoAction;
    private InputAction _keyboardAction;
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
        ros.RegisterPublisher<OdometryMsg>(topicName);
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName);
        ros.RegisterPublisher<TFMessageMsg>("/tf_test");
        ros.RegisterPublisher<BoolMsg>(gripperButtonTopicName);
        ros.RegisterPublisher<StringMsg>(demonstrationIndicatorTopic);
        
        _clutchAction = inputActions.FindAction("Clutch");
        _clutchAction.Enable(); // Required before reading input
        _gripperAction = inputActions.FindAction("Gripper");
        _gripperAction.Enable();
        _startDemoAction = inputActions.FindAction("StartDemo");
        _startDemoAction.Enable();
        _stopDemoAction = inputActions.FindAction("StopDemo");
        _stopDemoAction.Enable();        
        _keyboardAction = inputActions.FindAction("OpenKeyboard");
        _keyboardAction.Enable();
        _currentTransformRight = new Pose();
        _currentTransformLeft = new Pose();
        _clutchTransformRight = new Pose();
        _clutchTransformLeft = new Pose();
        _currentDiffTransformRight = new Pose();
        _currentDiffTransformLeft = new Pose();
        _tmpTransformInverted = new Pose();
        _tmpTransform = new Pose();

        SetPoseFromTransform(rightController.transform, ref _currentTransformRight);
        SetPoseFromTransform(leftController.transform, ref _currentTransformLeft);

        _currentDiffTransformRight.rotation.Set(0, 0, 0, 1.0f);
        _currentDiffTransformLeft.rotation.Set(0, 0, 0, 1.0f);

        _startDemoAudioData = GetComponents<AudioSource>()[0];
        _stopDemoAudioData = GetComponents<AudioSource>()[1];

        textInput = GameObject.Find("Text").GetComponent<TextMeshProUGUI>();
        textInput.text = ros.RosIPAddress;
    }


    public void Update()
    {
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

        if (_clutchAction.WasPressedThisFrame())
        {
            SetPoseFromTransform(rightController.transform, ref _clutchTransformRight);
            SetPoseFromTransform(leftController.transform, ref _clutchTransformLeft);
        }

        if (_clutchAction.IsPressed())
        {
            // We want to know the difference between the current transform and the clutch transform in the world frame
            InvertTransform(_clutchTransformRight, ref _tmpTransformInverted);
            SetPoseFromTransform(rightController.transform, ref _tmpTransform);
            DoTransform(_tmpTransformInverted, _tmpTransform, out _currentDiffTransformRight);

            InvertTransform(_clutchTransformLeft, ref _tmpTransformInverted);
            SetPoseFromTransform(leftController.transform, ref _tmpTransform);
            DoTransform(_tmpTransformInverted, _tmpTransform, out _currentDiffTransformLeft);
        }

        if (_clutchAction.WasReleasedThisFrame())
        {
            DoTransformDiff(_currentTransformRight, _currentDiffTransformRight, _clutchTransformRight,
                ref _tmpTransform);
            _currentTransformRight.position = _tmpTransform.position;
            _currentTransformRight.rotation = _tmpTransform.rotation;
            _currentDiffTransformRight.position.Set(0, 0, 0);
            _currentDiffTransformRight.rotation.Set(0, 0, 0, 1.0f);
        }

        _timeElapsed += Time.deltaTime;
        if (_timeElapsed > publishFrequency)
        {
            PublishOdomAndTf();
            _timeElapsed = 0;
        }

        if (_gripperAction.WasPressedThisFrame())
        {
            var msg = new BoolMsg()
            {
                data = !_grippedState
            };
            _grippedState = !_grippedState;
            ros.Publish(gripperButtonTopicName, msg);
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

    private void PublishOdomAndTf()
    {
        // Convert position and rotation using ROSGeometry
        DoTransformDiff(_currentTransformRight, _currentDiffTransformRight, _clutchTransformRight, ref _tmpTransform);
        Vector3<FLU> rosPosition = CoordinateSpaceExtensions.To<FLU>(_tmpTransform.position);
        Quaternion<FLU> rosRotation = CoordinateSpaceExtensions.To<FLU>(_tmpTransform.rotation);

        // Create header
        HeaderMsg header = new HeaderMsg
        {
            frame_id = "world",
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
        ros.Publish(topicName, odometryMsg);


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