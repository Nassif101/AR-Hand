using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTextureChanger : MonoBehaviour
{
    public bool change = false;
    public Material transparent;
    public Material skin;
    public Material nails;
    public bool isTransparent = false;

    public CustomRaycaster raycaster;

    void Start()
    {
        SwitchToSkin();
        GestureController.OnClick += ChangeMaterial;
    }

    void Update()
    {
        if (change)
        {
            Material[] mats = GetComponent<SkinnedMeshRenderer>().materials;
            mats[0] = transparent;
            mats[1] = transparent;
            GetComponent<SkinnedMeshRenderer>().materials = mats;
            change = false;
        }
    }

    private void ChangeMaterial() {
        if(!raycaster.isOnUIElement)
            return;

        if (isTransparent)
        {
            SwitchToSkin();
        }
        else
        {
            SwitchToTransparent();
        }
    }

    private void SwitchToTransparent()
    {
        Material[] mats = GetComponent<SkinnedMeshRenderer>().materials;
        mats[0] = transparent;
        mats[1] = transparent;
        GetComponent<SkinnedMeshRenderer>().materials = mats;

        isTransparent = true;
    }

    private void SwitchToSkin()
    {
        Material[] mats = GetComponent<SkinnedMeshRenderer>().materials;
        mats[0] = skin;
        mats[1] = nails;
        GetComponent<SkinnedMeshRenderer>().materials = mats;

        isTransparent = false;
    }
}
