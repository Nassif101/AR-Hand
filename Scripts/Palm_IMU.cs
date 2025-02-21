using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;

public class PalmQuatReader : MonoBehaviour
{
    public string topicName = "palm_angle";
    private Quaternion quat_ref;

    public bool Calibrate {
        get {
            return GlobalConfig.Instance.calibrate;
        }
        set { 
            GlobalConfig.Instance.calibrate = value;
        }
    }
    // private int count = 0;
    // private float mainlastUpdate = 0;
    // private float updateInterval = 1.0f;
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<QuaternionMsg>(topicName, QuaternionChanged);

        GlobalConfig.calibrateEvent += CalibrateChanged;
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
        if(Calibrate){
            quat_ref = msg.From<FLU>();
        }
        Quaternion quat_curr = msg.From<FLU>();
        Quaternion quat_diff = Quaternion.Inverse(quat_ref) * quat_curr;
        transform.rotation =  quat_diff;
        // transform.rotation = msg.From<FLU>();
    }

    void CalibrateChanged(bool value)
    {
        Calibrate = value;
    }
}

// Draw property field in inspector
#if UNITY_EDITOR
[UnityEditor.CustomEditor(typeof(PalmQuatReader))]
public class PalmQuatReaderEditor : UnityEditor.Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        PalmQuatReader myScript = (PalmQuatReader)target;
        if(GUILayout.Button("Calibrate"))
        {
            myScript.Calibrate = true;
        }
    }
}
#endif