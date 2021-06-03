using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class EnsembleManager : MonoBehaviour
{
    [SerializeField]
    private float maxAcceleration = 20.0f;

    [SerializeField]
    private float turnSensitivity = 1.0f;

    [SerializeField]
    private float maxSteerAngle = 45.0f;

    [SerializeField]
    private Vector3 centerOfMass;

    [SerializeField]
    private List<Wheel> wheels;

    [Header("Sensors")]
    public float sensorLength = 20f;

    private Rigidbody rb;

    private bool respawn;

    int layerMask;

    List<Vector3> positions;

    public GameObject startArea;

    private Transform targetSpot;

    public float ensembleSize = 2;

    List<float[]> actionStack = new List<float[]>();

    List<EnsembleItem> ensembleItems = new List<EnsembleItem>();


    private int totalStepCount = 0;
    private int episodeStepCount = 0;
    private int episodeCount = 0;
    private int successfulEpisodeCount = 0;
    private List<float> angles = new List<float>();

    void Start()
    {
        layerMask = LayerMask.GetMask("Collisionable");
        targetSpot = transform.parent.Find("ParkingSpot");
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = centerOfMass;

        positions = new List<Vector3>();
    }

    void Update()
    {
        AnimateWheels();
        positions.Add(transform.position);
    }

    void Move(float scalarInput)
    {
        foreach (Wheel wheel in wheels)
        {
            wheel.collider.motorTorque = scalarInput * maxAcceleration * 500 * Time.deltaTime;
        }
    }

    void Turn(float scalarInput)
    {
        foreach (Wheel wheel in wheels)
        {
            if (wheel.axle == Axle.Front)
            {
                float _steerAngle = scalarInput * turnSensitivity * maxSteerAngle;
                wheel.collider.steerAngle = Mathf.Lerp(wheel.collider.steerAngle, _steerAngle, 0.5f);
            }
        }
    }

    void AnimateWheels()
    {
        foreach (Wheel wheel in wheels)
        {
            Quaternion _rot;
            Vector3 _pos;
            wheel.collider.GetWorldPose(out _pos, out _rot);
            wheel.model.transform.position = _pos;
            wheel.model.transform.rotation = _rot;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        OnTriggerEnterOrStay(other);
    }

    void OnTriggerStay(Collider other)
    {
        OnTriggerEnterOrStay(other);
    }

    void OnTriggerEnterOrStay(Collider collider)
    {
        if (collider.CompareTag("spot"))
        {
            Bounds carBounds = GetComponent<Transform>().Find("Model/jeep/JEEP_BODY").gameObject.GetComponent<Collider>().bounds;
            // car inside parking spot cube
            if (collider.bounds.Contains(carBounds.max) && collider.bounds.Contains(carBounds.min)) //deviation < 5
            {
                for (int i = 0; i < positions.Count - 1; i++)
                {
                    //Debug.DrawLine(positions[i], positions[i + 1], Color.green, 2f);
                }
                positions.Clear();

                successfulEpisodeCount += 1;
                angles.Add(Vector3.Angle(transform.forward, targetSpot.forward));
                BeginEpisode();
            }
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        BeginEpisode();
    }

    public void SubscribeItem(EnsembleItem item)
    {
        ensembleItems.Add(item);
    }


    public void BeginEpisode()
    {
        episodeCount += 1;
        Debug.Log($"Timestep: {totalStepCount}, Episodes: {episodeCount}, SuccessfulEpisodes: {successfulEpisodeCount}, Angle: {(angles.Count > 0 ? angles.Average() : 0)}");
        episodeStepCount = 0;

        transform.localPosition = new Vector3(3, 1, 1);
        transform.localRotation = Quaternion.Euler(0, -90, 0);
        rb.velocity = new Vector3(0f, 0f, 0f);
        rb.angularVelocity = new Vector3(0f, 0f, 0f);
        respawn = true;
    }

    /*
        vectorAction[]
        Index 0: acceleration (-1 = accel backwards, +1 = accel forward)
        Index 1: turn (-1 = left, +1 = right)
    */
    public void PushAction(float[] vectorAction)
    {
        actionStack.Add(vectorAction);

        if (actionStack.Count == ensembleSize)
        {
            float movement = 0f;
            float turn = 0f;

            foreach (float[] action in actionStack)
            {
                movement += action[0];
                turn += action[1];
            }

            movement /= ensembleSize;
            turn /= ensembleSize;

            actionStack.Clear();

            // Avoid vehicle movement at beginning of episode
            LockUpVehicle();

            // Perform actions
            Move(movement);
            Turn(turn);

            totalStepCount += 1;
            episodeStepCount += 1;
            if (episodeStepCount == 5000)
            {
                BeginEpisode();
            }
        }
    }

    private void LockUpVehicle()
    {
        if (respawn)
        {
            foreach (Wheel wheel in wheels)
            {
                wheel.collider.brakeTorque = Mathf.Infinity;
            }
            rb.isKinematic = true;
            respawn = false;
        }
        else
        {
            rb.isKinematic = false;
            foreach (Wheel wheel in wheels)
            {
                wheel.collider.brakeTorque = 0f;
            }
        }
    }

    public float[] GetSensorDistances()
    {
        RaycastHit hit;
        float[] distances = new float[12];

        for (int i = 0; i < 12; i++)
        {
            if (Physics.Raycast(transform.position, Quaternion.AngleAxis(i * 30, transform.up) * transform.forward, out hit, sensorLength, layerMask))
            {
                //Debug.DrawLine(transform.position, hit.point);
                distances[i] = Vector3.Distance(transform.position, hit.point);
            }
            else
            {
                distances[i] = 0f;
            }
        }
        return distances;
    }

    public void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = Input.GetAxis("Vertical");
        actionsOut[1] = Input.GetAxis("Horizontal");
    }
}
