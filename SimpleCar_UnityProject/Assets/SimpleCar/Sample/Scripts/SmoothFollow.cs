using UnityEngine;

namespace SimpleCar.Sample
{
    public class SmoothFollow : MonoBehaviour
    {
        [SerializeField] private Transform target;
        [SerializeField] private Vector3 offset;
        [SerializeField] private float followSpeed = 4f;

        private void LateUpdate()
        {
            transform.position = Vector3.Lerp(transform.position, target.position + offset, followSpeed * Time.deltaTime);
            transform.LookAt(target.position);
        }
    }
}
