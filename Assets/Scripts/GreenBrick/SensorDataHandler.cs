using System.Collections;
using System.Collections.Generic;
using System.Security.Policy;
using UnityEngine;

public class SensorDataHandler
{
    public DataFrame currentDataFrame;

    public Quaternion GetQuatFromMessage(string message)
    {
        currentDataFrame = new DataFrame();
        currentDataFrame = JsonUtility.FromJson<DataFrame>(message);

        return new Quaternion(currentDataFrame.imus[0].q0, currentDataFrame.imus[0].q1, currentDataFrame.imus[0].q2, currentDataFrame.imus[0].q3);
    }
    
}
