using UnityEngine;
using UnityEngine.InputSystem;

public class callibrator : MonoBehaviour
{
    
    [Tooltip("Body that needs to be rotated")]
    public Transform rotationBody;

    //[ x: rotate in x axis, y: rotate in z axis]
    private Vector2 Calibration = Vector2.zero;     
    private Vector3 applyRotation;
    //Using New Input system --> InputAction
    private InputHandlingSystem inputhand;
    void Awake()
    {
        inputhand = new InputHandlingSystem();
    }
    private void OnEnable()
    {
        inputhand.Enable();
        InputSystem.EnableDevice(Accelerometer.current);
    }
    private void OnDisable()
    {
        inputhand.Disable();
        InputSystem.DisableDevice(Accelerometer.current);
    }


    // Call this from UI Button to Calibrate rotation to that tilt
    public void SetCalibration()
    {
        if (Application.platform == RuntimePlatform.WindowsEditor || Application.platform == RuntimePlatform.WindowsPlayer)
            return;
        if (Accelerometer.current.enabled)
        {
            //Read [AngularAcceleration] Input Using from Accelerometer using new InputSystem
            Vector3 dir = inputhand.FindAction("Accelerometer").ReadValue<Vector3>().normalized;

            //------------------------------------------------------------------------------

            //Convert Input Direction into Rotational Angle for Calibration angles
            Vector3 reading = new Vector3(Mathf.Abs(dir.x), Mathf.Abs(dir.y), Mathf.Abs(dir.z));
            Vector3 negative = new Vector3(dir.x / reading.x, dir.y / reading.y, dir.z / reading.z);

            Vector3 angle = new Vector3(
                -negative.x * Mathf.Asin(reading.x) * Mathf.Rad2Deg,
                negative.y * Mathf.Asin(reading.y) * Mathf.Rad2Deg, 0);

            Calibration = new Vector2(angle.y, angle.x);
        }
    }
    void FixedUpdate()
    {
        if (Application.platform == RuntimePlatform.WindowsEditor || Application.platform == RuntimePlatform.WindowsPlayer)
            return;
        if (Accelerometer.current.enabled)
        {
            //Read [AngularAcceleration] Input Using from Accelerometer using new InputSystem
            Vector3 dir = inputhand.FindAction("Accelerometer").ReadValue<Vector3>().normalized;

            //------------------------------------------------------------------------------

            //Creating Calibrated Tilt Direction
            Vector2 yzPlane = new Vector2(dir.z, dir.y);    // rotate in x-axis
            Vector2 rotatedYZ = new Vector2(
                yzPlane.x * Mathf.Cos(Calibration.x * Mathf.Deg2Rad) - yzPlane.y * Mathf.Sin(Calibration.x * Mathf.Deg2Rad),
                yzPlane.x * Mathf.Sin(Calibration.x * Mathf.Deg2Rad) + yzPlane.y * Mathf.Cos(Calibration.x * Mathf.Deg2Rad)
                );

            Vector2 xzPlane = new Vector2(dir.x, rotatedYZ.x);  //rotate in y-axix
            Vector2 rotatedXY = new Vector2(
                xzPlane.x * Mathf.Cos(Calibration.y * Mathf.Deg2Rad) - xzPlane.y * Mathf.Sin(Calibration.y * Mathf.Deg2Rad),
                xzPlane.x * Mathf.Sin(Calibration.y * Mathf.Deg2Rad) + xzPlane.y * Mathf.Cos(Calibration.y * Mathf.Deg2Rad)
                );

            Vector3 tilt = new Vector3(rotatedXY.x, rotatedYZ.y, rotatedXY.y);

            //------------------------------------------------------------------------------

            // Find normalized tilt to be applied to rotating body
            tilt.z = 0;
            Vector3 tiltDir = tilt.normalized;

            float tiltMag = Mathf.Clamp(tilt.magnitude * 2.85f, -1, 1);

            Vector3 reading = tiltDir * tiltMag;
            Vector3 additiveInput = new Vector3(-reading.y, 0, reading.x);

            //------------------------------------------------------------------------------

            //Calculate Rotation
            float tiltRate = Mathf.Clamp01(Time.fixedDeltaTime * 20);
            float lerpFactor = 1 - Mathf.Pow(1 - tiltRate, 3);
            applyRotation = new Vector3(
                    Mathf.Lerp(applyRotation.x, 20 * additiveInput.x, lerpFactor), 0,
                    Mathf.Lerp(applyRotation.z, 20 * additiveInput.z, lerpFactor));

            rotationBody.rotation = Quaternion.Euler(applyRotation);
        }
    }
}
