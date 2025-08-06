using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class TfPublisher : MonoBehaviour
{
    public ROSConnection ros;
    public string topicName = "/tf";
    public string parentFrame = "world";
    public string childFrame = "object_frame";
    public float publishFrequency = 0.1f;
    
    private float _timeElapsed;

    public void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>(topicName);
    }

    public void Update()
    {
        _timeElapsed += Time.deltaTime;

        if (_timeElapsed > publishFrequency)
        {
            PublishTf();
            _timeElapsed = 0;
        }
    }

    private void PublishTf()
    {
        // Convert position and rotation using ROSGeometry
        Vector3<FLU> rosPosition = CoordinateSpaceExtensions.To<FLU>(transform.position);
        Quaternion<FLU> rosRotation = CoordinateSpaceExtensions.To<FLU>(transform.rotation);

        // Create header
        HeaderMsg header = new HeaderMsg
        {
            frame_id = parentFrame,
            stamp = new TimeMsg
            {
                sec = (int)Time.time,
                nanosec = (uint)((Time.time - Mathf.Floor(Time.time)) * 1e9)
            }
        };

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
        var transforms = new [] { transformStamped };
        var tfMessage = new TFMessageMsg(transforms);

        // Publish the message
        ros.Publish(topicName, tfMessage);
    }
}