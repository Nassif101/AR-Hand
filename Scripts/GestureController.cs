using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class GestureController : MonoBehaviour
{
    private Collider HandCollider;
    [SerializeField] private Collider ThumbCollider;

    public bool isClicked = false;

    public static UnityAction OnClick;

    void Start()
    {
        HandCollider = GetComponent<Collider>();
    }

    void OnTriggerEnter(Collider other)
    {
        if (other == ThumbCollider)
        {
            isClicked = true;
            OnClick?.Invoke();
        }
    }

    public void ConsumeClick()
    {
        isClicked = false;
    }
}
