using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class DataFrame
{
    public string client;
    public int time;
    public List<imuframe> imus;
    public int gsr;

    public int t;
    public int h;
}


[Serializable]
public class imuframe
{
    public float ax;
    public float ay;
    public float az;

    public float gx;
    public float gy;
    public float gz;

    public float mx;
    public float my;
    public float mz;

    public float q0;
    public float q1;
    public float q2;
    public float q3;
}
