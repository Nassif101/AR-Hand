using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;

public class DIPJoint : MonoBehaviour
{
    public Transform PIP;
    public float factor = 1f;

    void Update()
    {
        Vector3 PIPOrientation = PIP.localEulerAngles;
        PIPOrientation.z *= factor;
        transform.localEulerAngles = PIPOrientation;
    }

   
}
