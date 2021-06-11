using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using System.Linq;

public enum Axle
{
    Front,
    Rear
}

[Serializable]
public struct Wheel
{
    public GameObject model;
    public WheelCollider collider;
    public Axle axle;
}

public class CarController : Agent
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

    private Transform targetSpot; //scale (1, 1.5, 1.6) for perpendicular; (1.7, 1.5, 2.4) for perpendicular

    private bool respawn;

    int layerMask;

    StatsRecorder statsRecorder;

    bool parked;

    List<Vector3> positions;

    public GameObject startArea;

    public bool randomPositions;

    float initialDistance;

    void Start()
    {
        targetSpot = transform.parent.Find("ParkingSpot");
        layerMask = LayerMask.GetMask("Collisionable");
        statsRecorder = Academy.Instance.StatsRecorder;

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

                float deviation = Vector3.Angle(transform.forward, targetSpot.forward);
                if (deviation != 0)
                {
                    AddReward(10000f / deviation); //is bigger if orientation is correct
                }
                else
                {
                    AddReward(10000f);
                }
                
                Debug.Log(GetCumulativeReward());
                statsRecorder.Add("Environment/Angle", Vector3.Angle(transform.forward, targetSpot.forward));
                EndEpisode();
                parked = true;
            }
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        AddReward(-1f);
        Debug.Log(GetCumulativeReward());
        EndEpisode();
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = centerOfMass;
    }

    public Vector3 RandomPosition()
    {
        Bounds bounds = startArea.GetComponent<Collider>().bounds;
        return new Vector3(
            UnityEngine.Random.Range(bounds.min.x, bounds.max.x),
            1,
            UnityEngine.Random.Range(bounds.min.z, bounds.max.z)
        );
    }

    public override void OnEpisodeBegin()
    {
        if (randomPositions)
        {
            transform.localPosition = RandomPosition();
            transform.localRotation = Quaternion.LookRotation(targetSpot.position - transform.position);
        }
        else
        {
            transform.localPosition = new Vector3(3, 1, 1);
            transform.localRotation = Quaternion.Euler(0, -90, 0);
        }
        
        rb.velocity = new Vector3(0f,0f,0f); 
        rb.angularVelocity = new Vector3(0f,0f,0f);
        respawn = true;

        initialDistance = Vector3.Distance(transform.localPosition, targetSpot.localPosition);

        if (parked)
        {
            statsRecorder.Add("Environment/Success", 1f);
        }
        else
        {
            statsRecorder.Add("Environment/Success", 0f);
        }
        parked = false;
    }

    /*
        vectorAction[]
        Index 0: acceleration (-1 = accel backwards, +1 = accel forward)
        Index 1: turn (-1 = left, +1 = right)
    */
    public override void OnActionReceived(float[] vectorAction)
    {
        // distance weight: 1, angle weight: 4
        float weightedReward = 0f;

        float distance = Vector3.Distance(transform.localPosition, targetSpot.localPosition);

        // Calculate reward using tanh
        // initial distance perpendicular: 3.995925, parallel: 3.633735
        // 3.995925 / 2 = 1.9979625
        // 3.633735 / 2 = 1.8168675
        float distanceReward;
        if (randomPositions)
        {
            distanceReward = (float)(Math.Tanh((3.995925 / 2) - distance) / 2) + 0.5f;
        }
        else
        {
            distanceReward = (float)(Math.Tanh((initialDistance / 2) - distance) / 2) + 0.5f;
        }

        // Calculate reward linearly
        // max distance: 5.3
        //float distanceReward = 1 - distance / 5.3f;

        weightedReward += distanceReward;

        // Angle reward in range: -1:0
        if (distance < 2)
        {
            float deviation = Vector3.Angle(transform.forward, targetSpot.forward);
            // Calculate angle reward linearly
            //float angleReward = - deviation / 180;
            
            // Calculate angle reward with tanh
            float angleReward = (float)(Math.Tanh(2 - deviation/45) / 2) - 0.5f;
            
            //Debug.Log("d: "+deviation+ " l: "+angleReward+" t: "+ tanhAngleReward);
            weightedReward += angleReward * 4;
        }

        // Per - step punishment
        weightedReward += -0.2f;

        AddReward(weightedReward);

        // Avoid vehicle movement at beginning of episode
        LockUpVehicle();

        // Perform actions
        Move(vectorAction[0]);
        Turn(vectorAction[1]);
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

    public override void CollectObservations(VectorSensor sensor)
    {
        float[] distances = GetSensorDistances();

        // 12 observations, which correspond to the GetSensorDistances() output array size
        foreach (float distance in distances)
        {
            sensor.AddObservation(distance);
        }

        // Add euclidean distance to target spot
        sensor.AddObservation(Vector3.Distance(transform.localPosition, targetSpot.localPosition));
    }

    private float[] GetSensorDistances()
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

    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = Input.GetAxis("Vertical");
        actionsOut[1] = Input.GetAxis("Horizontal");
    }
}
