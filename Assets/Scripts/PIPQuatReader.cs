using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using System;
using UnityEngine.UIElements;

public class PIPQuatReader : MonoBehaviour
{
    public string topicName = "indexPIP";

    [Header("Rotation Offsets")]
    public float zRotationOffset = 0f; // Offset around the Z-axis.
    public float xRotationOffset = 0f; // Offset around the X-axis.
    public float yRotationOffset = 0f; // Offset around the Y-axis.
    // private int count = 0;
    // private float mainlastUpdate = 0;
    // private float updateInterval = 1.0f;

    [Space(10)]
    [Header("Rotation Limits")]

    public float minAngle = 0f;
    public float maxAngle = 0f;
    public Vector3 prevRot;

    [Space(10)]
    [Header("Rotation Factors")]

    public float Xaxisfactor = 0f;
    public float Yaxisfactor = 0f;
    public float Zaxisfactor = 1f;

    [Space(10)]
    [Header("Calibration")]
    private Quaternion quat_ref;
    public bool selfCalibrate = true;
    public Transform target;    

    [Header("Debug")]
    [Space(10)]
    [SerializeField] private bool debugMode = false;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<QuaternionMsg>(topicName, QuaternionChanged);
    }

    // Start is called before the first frame update
    void QuaternionChanged(QuaternionMsg msg)
    {
        // i want substract mainlastUpdate from current time and if it is bigger than 1 second then i want to print the count and reset it to 0
        // count++;
        // if(count > 0 && Time.time - mainlastUpdate >= updateInterval){
        // mainlastUpdate = Time.time;
        // Debug.Log("FPS: " + count);
        // count = 0;
        // }
        if(selfCalibrate){
            quat_ref = msg.From<FLU>();
            selfCalibrate = false;
        }
        Quaternion quat_curr = Quaternion.Euler(msg.From<FLU>().eulerAngles.x + xRotationOffset , msg.From<FLU>().eulerAngles.y + yRotationOffset ,msg.From<FLU>().eulerAngles.z + zRotationOffset);
        Quaternion quat_diff = Quaternion.Inverse(quat_ref) * quat_curr;
        transform.rotation =  Quaternion.Inverse(target.transform.rotation) * quat_diff;
        // transform.rotation = Quaternion.Euler(msg.From<FLU>().eulerAngles.x + xRotationOffset , msg.From<FLU>().eulerAngles.y + yRotationOffset ,msg.From<FLU>().eulerAngles.z + zRotationOffset);
        Vector3 tempRotation = transform.localEulerAngles;
        // tempRotation.x = Mathf.Clamp(tempRotation.x, minAngle, maxAngle);
        // if( transform.localEulerAngles.x < minAngle || transform.localEulerAngles.x > maxAngle)
        //     prevRot = transform.localEulerAngles;
        // else
        //     tempRotation.x = prevRot.x;
        tempRotation.x *= Xaxisfactor;
        tempRotation.y *= Yaxisfactor;
        tempRotation.z *= Zaxisfactor;
        transform.localEulerAngles = tempRotation;
        if(debugMode)
            Debug.Log(DateTime.Now.Ticks);
            Debug.Log($"{gameObject.name}: Z rotation: {tempRotation.z}");       
        
        
    }


}
