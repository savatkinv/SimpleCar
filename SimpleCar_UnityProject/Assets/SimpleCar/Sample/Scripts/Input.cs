using UnityEngine;
using UnityEngine.InputSystem;

namespace SimpleCar.Sample
{
    public class Input : MonoBehaviour
    {
        [SerializeField] private Car car;
        [SerializeField] private PlayerInput playerInput;

        private float steer;
        private float accel;
        private float brake;

        public void OnMove(InputAction.CallbackContext context)
        {
            Vector2 move = context.ReadValue<Vector2>();

            steer = move.x;
            accel = move.y;
        }

        public void OnBrake(InputAction.CallbackContext context)
        {
            if (context.phase == InputActionPhase.Started)
            {
                brake = 1;
            } else if (context.phase == InputActionPhase.Canceled)
            {
                brake = 0;
            }
        }

        private void Update()
        {
            car.Move(steer, accel, brake);
        }
    }
}
