using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomRaycaster : MonoBehaviour
{
  // create a raycast in the update function from the transform to forward vector and only hit the layer of the ui
    public LayerMask layerMask;
    public float raycastDistance = 100f;
    public Transform origin;

    private LineRenderer lineRenderer;


    public bool isOnUIElement = false;

    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        lineRenderer.enabled = true;
        lineRenderer.useWorldSpace = true;
        lineRenderer.positionCount = 2;
        lineRenderer.startWidth = 0.003f;
    }

    void FixedUpdate()
    {
        RaycastHit hit;
        lineRenderer.SetPosition(0, origin.position);
        lineRenderer.SetPosition(1, origin.position + origin.forward * raycastDistance);
        if (Physics.Raycast(origin.position, origin.forward, out hit, raycastDistance, layerMask))
        {
            Debug.DrawRay(origin.position, origin.forward * hit.distance, Color.yellow);
            Debug.Log(hit.collider.gameObject.name);
            lineRenderer.startColor = Color.green;
            isOnUIElement = true;
        }
        else
        {
            lineRenderer.startColor = Color.red;
            Debug.DrawRay(origin.position, origin.forward * raycastDistance, Color.white);
            isOnUIElement = false;
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawRay(origin.position, origin.forward * raycastDistance);
    }

}
