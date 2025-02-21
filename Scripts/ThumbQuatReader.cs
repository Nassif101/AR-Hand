using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;


public class ThumbQuatReader : MonoBehaviour
{
    public string topicName = "thumb_first_angle";
    public float zRotationOffset = 0f; // Offset around the Z-axis.
    public float xRotationOffset = 0f; // Offset around the X-axis.
    public float yRotationOffset = 0f; // Offset around the Y-axis.
    // private int count = 0;
    // private float mainlastUpdate = 0;
    // private float updateInterval = 1.0f;

    public float minAngle = 0f;
    public float maxAngle = 0f;

    public Vector3 prevRot;

    public bool debugMode = false;

    public float Xaxisfactor = 0f;
    public float Yaxisfactor = 0f;
    public float Zaxisfactor = 1f;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<QuaternionMsg>(topicName, QuaternionChanged);
    }

    

    void QuaternionChanged(QuaternionMsg msg)
    {
        // i want substract mainlastUpdate from current time and if it is bigger than 1 second then i want to print the count and reset it to 0
        // count++;
        // if(count > 0 && Time.time - mainlastUpdate >= updateInterval){
        // mainlastUpdate = Time.time;
        // Debug.Log("FPS: " + count);
        // count = 0;
        // }
        transform.rotation = Quaternion.Euler(msg.From<FLU>().eulerAngles.x + xRotationOffset , msg.From<FLU>().eulerAngles.y + yRotationOffset ,msg.From<FLU>().eulerAngles.z + zRotationOffset);
        Vector3 tempRotation = transform.localEulerAngles;
        // tempRotation.x = Mathf.Clamp(tempRotation.x, minAngle, maxAngle);
        // if( transform.localEulerAngles.x < minAngle || transform.localEulerAngles.x > maxAngle)
        //     prevRot = transform.localEulerAngles;
        // else
        //     tempRotation.x = prevRot.x;
        tempRotation.x *= Xaxisfactor;
        tempRotation.y *= Yaxisfactor;
        tempRotation.z *= Zaxisfactor;
        Vector3 temp;
        temp.x = tempRotation.z;
        temp.y = tempRotation.y;
        temp.z = tempRotation.x;
        transform.localEulerAngles = temp;
        if(debugMode)
            Debug.Log($"{gameObject.name}: Z rotation: {tempRotation.z}");        
    }
}
