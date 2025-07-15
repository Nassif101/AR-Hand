using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System.Runtime.InteropServices;

/// <summary>
///
/// </summary>
public class TrackerPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject Target;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.02f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Create a new message with the cube's position and rotation
            Target.transform.rotation = Random.rotation;
            PosRotMsg targetPos = new PosRotMsg(
                Target.transform.position.x,
                Target.transform.position.y,
                Target.transform.position.z,
                Target.transform.rotation.To<FLU>().x,
                Target.transform.rotation.To<FLU>().y,
                Target.transform.rotation.To<FLU>().z,
                Target.transform.rotation.To<FLU>().w
            );

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, targetPos);

            timeElapsed = 0;
        }
    }
}