using UnityEngine;

namespace SimpleCar
{
    public class Car : MonoBehaviour
    {
        [SerializeField] private float targetWheelDistance = 1f;
        [SerializeField] private float minWheelDistance = 0.5f;
        [SerializeField] private float suspension = 300f;
        [SerializeField] private float suspensionDamper = 30f;
        [SerializeField] [Range(0f, 1f)] private float tireGrip = 0.5f;
        [SerializeField] private float steerAngle = 45f;
        [SerializeField] private float carMaxSpeed = 15f;
        [SerializeField] private float accelSpeed = 50f;
        [SerializeField] [Range(0f, 1f)] private float brakeForce = 1f;
        [SerializeField] private float wheelRadius = 0.5f;
        [SerializeField] private float wheelRotationSpeed = 90f;
        [SerializeField] private float wheelTorgueRotationSpeed = 3f;
        [SerializeField] private Vector3 centerOfMass = Vector3.zero;
        [SerializeField] private AnimationCurve steerCurve;
        [SerializeField] private AnimationCurve torqueCurve;
        [SerializeField] private AnimationCurve gripCurve;
        [SerializeField] private AnimationCurve brakeCurve;
        [SerializeField] private Transform[] suspensions;
        [SerializeField] private Transform[] wheels;
        [SerializeField] private Transform[] steeringSuspensions;
        [SerializeField] private Transform[] inverseSteeringSuspensions;
        [SerializeField] private LayerMask groundMask;

        private Rigidbody rb;
        private RaycastHit hit;

        private float currentAccel = 0f;
        private float currentBrake = 0f;
        private float currentSpeed = 0f;
        private float currentGrip = 0f;
        private float currentVelocity = 0f;

        private const float accelUpLerpSpeed = 2f; 
        private const float accelDownLerpSpeed = 4f;
        private const float accelMin = -0.5f;
        private const float accelMax = 1f;
        private const float accelE = 0.01f;

        private class CarInput
        {
            public float accel;
            public float steer;
            public float brake;
        }
        private CarInput carInput;

        public float Speed => currentSpeed;
        public float SpeedPerHour => currentSpeed * carMaxSpeed;

        public void Move(float steer, float accel, float brake)
        {
            carInput.steer = steer;
            carInput.accel = accel;
            carInput.brake = brake;
        }

        private void Start()
        {
            carInput = new CarInput();
            rb = GetComponent<Rigidbody>();
            rb.centerOfMass = centerOfMass;
        }

        private void Update()
        {
            SteerControl();
            AccelControl();
            BrakeControl();
        }

        private void FixedUpdate()
        {
            currentVelocity = Vector3.Dot(transform.forward, rb.linearVelocity);
            currentSpeed = Mathf.Clamp01(Mathf.Abs(currentVelocity) / carMaxSpeed);
            currentGrip = gripCurve.Evaluate(currentSpeed) * tireGrip;

            for (int i = 0; i < wheels.Length; i++)
            {
                UpdateWheel(suspensions[i], wheels[i], Mathf.Abs(carInput.accel));
            }
        }

        private void SteerControl()
        {
            float steer = carInput.steer * steerAngle * steerCurve.Evaluate(currentSpeed);

            foreach (var s in steeringSuspensions)
            {
                s.localRotation = Quaternion.Euler(0, steer, 0);
            }

            foreach (var s in inverseSteeringSuspensions)
            {
                s.localRotation = Quaternion.Euler(0, -steer, 0);
            }
        }

        private void AccelControl()
        {
            if (carInput.accel == 0)
            {
                currentAccel = Mathf.Abs(currentAccel) < accelE ? 0 : Mathf.Lerp(currentAccel, 0f, Time.deltaTime);
            } else
            {
                currentAccel = Mathf.Lerp(
                    currentAccel,
                    carInput.accel > 0 ? accelMax : accelMin,
                    Time.deltaTime * (carInput.accel > 0 ? accelUpLerpSpeed : accelDownLerpSpeed)
                    );
            }
        }

        private void BrakeControl()
        {
            currentBrake = carInput.brake > 0 ? Mathf.Lerp(currentBrake, 1f, Time.deltaTime) : 0f;

            if (currentBrake > 0)
            {
                currentAccel = Mathf.Lerp(currentAccel, 0f, Time.deltaTime * brakeForce);
            }
        }

        private void UpdateWheel(Transform wheelBase, Transform wheel, float absAccel)
        {
            float torqueSpeed = torqueCurve.Evaluate(currentSpeed) * accelSpeed * currentAccel;
            float hitDistance = targetWheelDistance;
            bool hasContact = false;

            if (Physics.Raycast(wheelBase.position, -wheelBase.up, out hit, targetWheelDistance, groundMask))
            {
                hitDistance = hit.distance;
                hasContact = true;
            }

            if (hasContact)
            {
                Vector3 wheelWorldVel = rb.GetPointVelocity(wheelBase.position);

                AddSuspensionForce(wheelBase, wheelWorldVel, hitDistance);
                AddGripForce(wheelBase, wheelWorldVel);
                AddAccelerationForce(wheelBase, wheelWorldVel, torqueSpeed * absAccel);
            }

            float wheelOffset = Mathf.Clamp(hitDistance, minWheelDistance, targetWheelDistance) - wheelRadius;
            wheel.position = wheelBase.position - wheelBase.up * wheelOffset;

            float wheelRotation = hasContact ?
                Mathf.Lerp(currentVelocity * wheelRotationSpeed, 0f, currentBrake * 8f) :
                torqueSpeed * wheelTorgueRotationSpeed;
            wheel.Rotate(wheelRotation * Time.fixedDeltaTime, 0, 0);
        }

        private void AddSuspensionForce(Transform wheelBase, Vector3 wheelWorldVel, float hitDistance)
        {
            float offset = targetWheelDistance - hitDistance;
            float suspensionRelativeVelocity = Vector3.Dot(wheelBase.up, wheelWorldVel);
            float suspensionForce = offset * suspension - suspensionRelativeVelocity * suspensionDamper;

            rb.AddForceAtPosition(suspensionForce * wheelBase.up, wheelBase.position, ForceMode.Force);
        }

        private void AddGripForce(Transform wheelBase, Vector3 wheelWorldVel)
        {
            float desiredVelChange = -Vector3.Dot(wheelBase.right, wheelWorldVel) * currentGrip / Time.fixedDeltaTime;

            rb.AddForceAtPosition(desiredVelChange * wheelBase.right, wheelBase.position, ForceMode.Force);
        }

        private void AddAccelerationForce(Transform wheelBase, Vector3 wheelWorldVel, float wheelTorque)
        {
            if (currentBrake > 0)
            {
                wheelTorque = -Mathf.Sign(currentVelocity) * currentSpeed * accelSpeed;

                float brake = brakeCurve.Evaluate(currentSpeed) * currentBrake;
                float brakeForce = -Vector3.Dot(wheelBase.forward, wheelWorldVel) * brake;
                rb.AddForceAtPosition(brakeForce * wheelBase.forward / Time.fixedDeltaTime, wheelBase.position, ForceMode.Force);
            }
            else
            {
                if (Mathf.Abs(currentVelocity) > carMaxSpeed)
                {
                    if (currentAccel * currentVelocity > 0)
                    {
                        wheelTorque = 0f;
                    }
                }

                if (Mathf.Abs(currentAccel) < 0.01f)
                {
                    wheelTorque = -Mathf.Sign(currentVelocity) * currentSpeed * accelSpeed;
                }
            }

            rb.AddForceAtPosition(wheelBase.forward * wheelTorque, wheelBase.position, ForceMode.Force);
        }

        private void OnValidate()
        {
            for (int i = 0; i < wheels.Length; i++)
            {
                wheels[i].position = suspensions[i].position - suspensions[i].up * (targetWheelDistance - wheelRadius);
            }
        }
    }
}