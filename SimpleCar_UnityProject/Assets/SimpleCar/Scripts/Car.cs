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
        private float currentForwardVelocity = 0f;

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
            currentForwardVelocity = Vector3.Dot(transform.forward, rb.linearVelocity);
            currentSpeed = Mathf.Clamp01(Mathf.Abs(currentForwardVelocity) / carMaxSpeed);
            currentGrip = gripCurve.Evaluate(currentSpeed) * tireGrip;

            for (int i = 0; i < wheels.Length; i++)
            {
                UpdateWheel(suspensions[i], wheels[i]);
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

        private void UpdateWheel(Transform wheelBase, Transform wheel)
        {
            Vector3 springDir = wheelBase.up;
            Vector3 wheelWorldVel = rb.GetPointVelocity(wheelBase.position);

            float torqueSpeed = torqueCurve.Evaluate(currentSpeed) * accelSpeed * currentAccel;
            float torque = torqueSpeed * Mathf.Abs(carInput.accel);
            float wheelRotation = torqueSpeed * wheelTorgueRotationSpeed;

            float hitDistance = targetWheelDistance;
            if (Physics.Raycast(wheelBase.position, -wheelBase.up, out hit, targetWheelDistance, groundMask))
            {
                hitDistance = hit.distance;

                // suspension
                float offset = targetWheelDistance - hitDistance;
                float suspensionRelativeVelocity = Vector3.Dot(springDir, wheelWorldVel);
                float suspensionForce = offset * suspension - suspensionRelativeVelocity * suspensionDamper;
                rb.AddForceAtPosition(suspensionForce * springDir, wheelBase.position, ForceMode.Force);

                // grip
                float desiredVelChange = -Vector3.Dot(wheelBase.right, wheelWorldVel) * currentGrip / Time.fixedDeltaTime;
                rb.AddForceAtPosition(desiredVelChange * wheelBase.right, wheelBase.position, ForceMode.Force);

                // acceleration
                if (currentBrake > 0)
                {
                    torque = -Mathf.Sign(currentForwardVelocity) * currentSpeed * accelSpeed;

                    // brake force
                    float brakeForce = -Vector3.Dot(wheelBase.forward, wheelWorldVel) / Time.fixedDeltaTime * brakeCurve.Evaluate(currentSpeed) * currentBrake;
                    rb.AddForceAtPosition(brakeForce * wheelBase.forward, wheelBase.position, ForceMode.Force);
                }
                else
                {
                    // fix max speed
                    if (Mathf.Abs(currentForwardVelocity) > carMaxSpeed)
                    {
                        if (currentAccel * currentForwardVelocity > 0)
                        {
                            torque = 0f;
                        }
                    }

                    if (Mathf.Abs(currentAccel) < 0.01f)
                    {
                        torque = -Mathf.Sign(currentForwardVelocity) * currentSpeed * accelSpeed;
                    }
                }

                // torque force
                rb.AddForceAtPosition(wheelBase.forward * torque, wheelBase.position, ForceMode.Force);

                wheelRotation = Mathf.Lerp(currentForwardVelocity * wheelRotationSpeed, 0f, currentBrake * 8f);
            }

            float wheelOffset = Mathf.Clamp(hitDistance, minWheelDistance, targetWheelDistance) - wheelRadius;
            
            wheel.position = wheelBase.position - wheelBase.up * wheelOffset;

            wheel.Rotate(wheelRotation * Time.fixedDeltaTime, 0, 0);
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