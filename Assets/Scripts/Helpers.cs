using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System;

public static class Helpers
{
    public static float StandardDeviation(this IEnumerable<float> values)
    {
        float avg = values.Average();
        return (float)Math.Sqrt(values.Average(v => Math.Pow(v - avg, 2)));
    }

    public static Vector3 RandomPosition(GameObject startArea)
    {
        Bounds bounds = startArea.GetComponent<Collider>().bounds;
        return new Vector3(
            UnityEngine.Random.Range(bounds.min.x, bounds.max.x),
            1,
            UnityEngine.Random.Range(bounds.min.z, bounds.max.z)
        );
    }
}
