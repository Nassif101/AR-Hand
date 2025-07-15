using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.QuatidMsg;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;




public class LatencyTest : MonoBehaviour
{
    ROSConnection ros;
    public string topicName_incoming = "hinflug";
    public string topicName_outgoing = "ruckflug";
    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<Float64Msg>(topicName_incoming, Callback);
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64Msg>(topicName_outgoing);
    }
    // Update is called once per frame
    void Callback(Float64Msg msg)
    {
        Float64Msg msg_out = new Float64Msg();
        msg_out.data = msg.data;
        Debug.Log("Received: " + msg.data);
        ros.Publish(topicName_outgoing, msg_out);
    }
    
}
