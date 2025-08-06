using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
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
    private bool _grippedState;

    private Pose _clutchTransformRight;
    private Pose _clutchTransformLeft;
    private Pose _currentTransformRight;
    private Pose _currentTransformLeft;
    private Pose _currentDiffTransformRight;
    private Pose _currentDiffTransformLeft;
    private Pose _tmpTransformInverted;
    private Pose _tmpTransform;

    private void DoTransform(Pose transformLhs, Pose transformRhs, out Pose newTransform)
    {
        // newTransform.position = transformLhs.position + transformLhs.rotation * transformRhs.position;
        // newTransform.rotation = transformLhs.rotation * transformRhs.rotation;
        newTransform = transformRhs.GetTransformedBy(transformLhs);
    }

    private void DoTransformDiff(Pose transformCurrent, Pose transformDiff, Pose transformClutchIn,
        ref Pose newTransform)
    {
        // transformDiff.rotation = transformClutchIn.rotation * transformDiff.rotation *
        //                          Quaternion.Inverse(transformClutchIn.rotation);
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
        ros.RegisterPublisher<OdometryMsg>(topicName);
        ros.RegisterPublisher<TFMessageMsg>(tfTopicName);
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
    }


    public void Update()
    {
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
            var msg = new StringMsg()
            {
                data = "Starting demonstration"
            };
            ros.Publish(demonstrationIndicatorTopic, msg);
        }

        if (_stopDemoAction.WasPressedThisFrame())
        {
            var msg = new StringMsg()
            {
                data = "Stopping demonstration"
            };
            ros.Publish(demonstrationIndicatorTopic, msg);
        }
    }

    private static TimeMsg GetRosTime()
    {
        DateTime unixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        DateTime now = DateTime.UtcNow;

        TimeSpan timeSinceEpoch = now - unixEpoch;
        return new TimeMsg
        {
            sec = (int)timeSinceEpoch.TotalSeconds,
            nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
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
    }
}