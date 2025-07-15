using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class GlobalConfig : MonoBehaviour
{
    public static GlobalConfig Instance;
    public bool calibrate = true;  

    public static UnityAction<bool> calibrateEvent;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(this.gameObject);
        } else
        {
            Destroy(this.gameObject);
        }
    }

    public void Update()
    {
        if(SensorsCalibration())
        {
            StartCoroutine(changeCalibrate());
        }
    }

    IEnumerator changeCalibrate()
    {
        Debug.Log("Calibration done");
        yield return new WaitForSeconds(1);
        calibrate = false;
    }

    public void SetCalibrate(bool value)
    {
        calibrate = value;
        calibrateEvent?.Invoke(value);
    }

    public bool SensorsCalibration()
    {
        bool cal = false;
        QuatReader[] quatReaders = FindObjectsOfType<QuatReader>();
        foreach (QuatReader quatReader in quatReaders)
        {
            if (quatReader.SelfCalibrate)
            {
                cal = true;
            }
        }

        return cal;
    }
}
