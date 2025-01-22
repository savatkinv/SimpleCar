using UnityEngine;

namespace SimpleCar
{
    public class Car : MonoBehaviour
    {
        [SerializeField] private float targetWheelDistance = 1f;
        [SerializeField] private float minWheelDistance = 0.5f;
        [SerializeField] private float suspension = 300f;
        [SerializeField] private float suspensionDamper = 30f;
        [SerializeField] private float tireGripFactor = 0.2f;
        [SerializeField] private float tireMass = 2f;
        [SerializeField] private float steerAngle = 45f;
        [SerializeField] private float carMaxSpeed = 15f;
        [SerializeField] private float accelSpeed = 50f;
        [SerializeField] private float brakeForce = 1f;
        [SerializeField] private float wheelRadius = 0.5f;
        [SerializeField] private float wheelRotationSpeed = 90f;
        [SerializeField] private float wheelTorgueRotationSpeed = 3f;
        [SerializeField] private Vector3 centerOfMass = Vector3.zero;
        [SerializeField] private AnimationCurve steerCurve;
        [SerializeField] private AnimationCurve torqueCurve;
        [SerializeField] private AnimationCurve gripCurve;
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
        private float currentGripFactor = 0f;

        private class CarInput
        {
            public float accel;
            public float steer;
            public float brake;
        }
        private CarInput carInput;

        public float Speed => currentSpeed;

        public void Move(float steer, float accel, float brake)
        {
            carInput.steer = steer;
            carInput.accel = accel;
            currentBrake = brake > 0 ? Mathf.Lerp(currentBrake, 1f, Time.deltaTime) : 0f;
        }

        private void Start()
        {
            carInput = new CarInput();
            rb = GetComponent<Rigidbody>();
            rb.centerOfMass = centerOfMass;
        }

        private void Update()
        {
            Control();
        }

        private void FixedUpdate()
        {
            for (int i = 0; i < wheels.Length; i++)
            {
                UpdateWheel(suspensions[i], wheels[i]);
            }

            float currentVelocity = Mathf.Abs(Vector3.Dot(transform.forward, rb.linearVelocity));

            currentSpeed = Mathf.Clamp01(currentVelocity / carMaxSpeed);
            currentGripFactor = gripCurve.Evaluate(currentSpeed);
        }

        private void Control()
        {
            float steer = carInput.steer * steerAngle * steerCurve.Evaluate(currentSpeed) * currentGripFactor;

            foreach (var s in steeringSuspensions)
            {
                s.localRotation = Quaternion.Euler(0, steer, 0);
            }

            foreach (var s in inverseSteeringSuspensions)
            {
                s.localRotation = Quaternion.Euler(0, -steer, 0);
            }

            if (carInput.accel > 0)
            {
                currentAccel = Mathf.Lerp(currentAccel, 1f, Time.deltaTime * 2f);
            } else if (carInput.accel < 0)
            {
                currentAccel = Mathf.Lerp(currentAccel, -0.5f, Time.deltaTime * 4f);
            } else
            {
                currentAccel = Mathf.Lerp(currentAccel, 0f, Time.deltaTime);

                if (Mathf.Abs(currentAccel) < 0.01f)
                    currentAccel = 0f;
            }
        }

        private void UpdateWheel(Transform wheelBase, Transform wheel)
        {
            Vector3 springDir = wheelBase.up;
            Vector3 wheelWorldVel = rb.GetPointVelocity(wheelBase.position);

            float carSpeed = Vector3.Dot(transform.forward, rb.linearVelocity);
            float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carMaxSpeed);
            float torque = torqueCurve.Evaluate(normalizedSpeed) * currentAccel * accelSpeed;

            float wheelRotation = torque * wheelTorgueRotationSpeed * Time.fixedDeltaTime;

            float hitDistance = targetWheelDistance;
            if (Physics.Raycast(wheelBase.position, -wheelBase.up, out hit, targetWheelDistance, groundMask))
            {
                // suspension
                hitDistance = hit.distance;

                float offset = targetWheelDistance - hitDistance;
                float vel = Vector3.Dot(springDir, wheelWorldVel);
                float force = offset * suspension - vel * suspensionDamper;

                rb.AddForceAtPosition(force * springDir, wheelBase.position, ForceMode.Force);

                // grip
                float desiredVelChange = -Vector3.Dot(wheelBase.right, wheelWorldVel) * tireGripFactor * currentGripFactor;

                rb.AddForceAtPosition(desiredVelChange * tireMass * wheelBase.right / Time.fixedDeltaTime, wheelBase.position, ForceMode.Force);

                // acceleration
                Vector3 accelDir = wheelBase.forward;

                if (currentBrake > 0)
                {
                    torque = - Mathf.Sign(carSpeed) * Mathf.Clamp01(Mathf.Abs(carSpeed) / carMaxSpeed) * accelSpeed * brakeForce;
                }
                else
                {
                    // fix max speed
                    if (Mathf.Abs(carSpeed) > carMaxSpeed)
                    {
                        if (currentAccel * carSpeed > 0)
                        {
                            torque = 0f;
                        }
                    }

                    if (Mathf.Abs(currentAccel) < 0.01f)
                    {
                        torque = -Mathf.Sign(carSpeed) * Mathf.Clamp01(Mathf.Abs(carSpeed) / carMaxSpeed) * accelSpeed;
                    }
                }


                rb.AddForceAtPosition(accelDir * torque, wheelBase.position, ForceMode.Force);

                wheelRotation = carSpeed * wheelRotationSpeed * Time.fixedDeltaTime;
            }

            float wheelOffset = Mathf.Clamp(hitDistance, minWheelDistance, targetWheelDistance) - wheelRadius;
            
            wheel.position = wheelBase.position - wheelBase.up * wheelOffset;

            wheel.Rotate(wheelRotation, 0, 0);
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