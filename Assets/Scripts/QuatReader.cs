using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using System;
using UnityEngine.InputSystem.XR;

/// <summary>
/// Reads quaternion data and applies rotations to a game object.
/// </summary>
public class QuatReader : MonoBehaviour
{
    /// <summary>
    /// The name of the topic to subscribe to for quaternion data.
    /// </summary>
    public string topicName = "finger1_first_angle";

    [Header("Rotation Offsets")]
    public float zRotationOffset = 0f;
    public float xRotationOffset = 0f;
    public float yRotationOffset = 0f;

    [Space(10)]
    [Header("Rotation Limits")]
    public bool limitAngle = false;
    public float minAngle = 0f;
    public float maxAngle = 0f;
    private Vector3 prevRot;

    [Space(10)]
    [Header("Rotation Factors")]
    public float Xaxisfactor = 0f;
    public float Yaxisfactor = 0f;
    public float Zaxisfactor = 1f;

    [Space(10)]
    [Header("Calibration")]
    private Quaternion quat_ref;
    private bool selfCalibrate = true;
    
    TrackedPoseDriver trackedPoseDriver;

    /// <summary>
    /// Gets a value indicating whether the object is currently self-calibrating.
    /// </summary>
    public bool SelfCalibrate => selfCalibrate;

    /// <summary>
    /// Gets or sets a value indicating whether the object should be calibrated.
    /// </summary>
    public bool Calibrate
    {
        get { return GlobalConfig.Instance.calibrate; }
        set { GlobalConfig.Instance.calibrate = value; }
    }

    public bool inverse = false;
    public bool isFinger = true;

    [Header("Debug")]
    [SerializeField] private bool debugMode = false;

    /// <summary>
    /// Subscribes to the quaternion topic and registers the calibration event.
    /// </summary>
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<QuaternionMsg>(topicName, QuaternionChanged);
        GlobalConfig.calibrateEvent += CalibrateChanged;
    }

    /// <summary>
    /// Handles the quaternion changed event and applies the rotation to the game object.
    /// </summary>
    /// <param name="msg">The quaternion message.</param>
    void QuaternionChanged(QuaternionMsg msg)
    {   
        transform.rotation = Quaternion.Euler(msg.From<FLU>().eulerAngles.x + xRotationOffset, msg.From<FLU>().eulerAngles.y + yRotationOffset, msg.From<FLU>().eulerAngles.z + zRotationOffset);
        if(isFinger)
            transform.localEulerAngles = LimitLocalRotation(limitAngle);
    }
    
    /// <summary>
    /// Handles the reset calibration event and updates the reference quaternion for calibration.
    /// </summary>
    /// <param name="msg">The quaternion message.</param>
    void HandleResetCalibration(QuaternionMsg msg)
    {
        if (Calibrate && selfCalibrate)
        {
            Debug.Log($"Resetting calibration {gameObject.name}");
            quat_ref = msg.From<FLU>();
            selfCalibrate = false;
        }
    }

    /// <summary>
    /// Calculates the difference between the current quaternion and the reference quaternion.
    /// </summary>
    /// <param name="msg">The quaternion message.</param>
    /// <returns>The adjusted quaternion based on rotation offsets.</returns>
    Quaternion GetAngleDiff(QuaternionMsg msg)
    {
        HandleResetCalibration(msg);
        Quaternion quat_curr = msg.From<FLU>();
        Quaternion quat_diff = Quaternion.Inverse(quat_ref) * quat_curr;
        return Quaternion.Euler(quat_diff.eulerAngles.x + xRotationOffset, quat_diff.eulerAngles.y + yRotationOffset, quat_diff.eulerAngles.z + zRotationOffset);
    }
    /// <summary>
    /// Limits the local rotation of the game object based on the specified angle limits.
    /// </summary>
    /// <param name="clampAngle">Indicates whether to clamp the angle within the limits.</param>
    /// <returns>The limited local rotation.</returns>
    Vector3 LimitLocalRotation(bool clampAngle = false)
    {
        Vector3 tempRotation = transform.localEulerAngles;
        if (clampAngle)
        {
            tempRotation.x = Mathf.Clamp(tempRotation.x, minAngle, maxAngle);
            if (transform.localEulerAngles.x < minAngle || transform.localEulerAngles.x > maxAngle)
                prevRot = transform.localEulerAngles;
            else
                tempRotation.x = prevRot.x;
        }
        tempRotation.x *= Xaxisfactor;
        tempRotation.y *= Yaxisfactor;
        tempRotation.z *= Zaxisfactor;
        if (debugMode)
        {
            Debug.Log($"{gameObject.name}: rotation: {tempRotation}");
        }
        return tempRotation;
    }

    /// <summary>
    /// Handles the calibration changed event and updates the self-calibrate flag.
    /// </summary>
    /// <param name="value">The new calibration value.</param>
    void CalibrateChanged(bool value)
    {
        selfCalibrate = value;
    }
}