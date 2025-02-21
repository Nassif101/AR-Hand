using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;

public class RosQuatReader : MonoBehaviour
{
    public string topicName = "quaternion";
    public Transform target;

    // private Vector3 eulerAngle;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<QuaternionMsg>(topicName, QuaternionChanged);
    }

    void QuaternionChanged(QuaternionMsg msg)
    {
        
        // eulerAngle.z = msg.From<FLU>().eulerAngles.z;

        target.rotation = Quaternion.Euler(0f , 0f, msg.From<FLU>().eulerAngles.z);
    }
}