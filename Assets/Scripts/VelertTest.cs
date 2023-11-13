using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class NewBehaviourScript : MonoBehaviour
{
    public bool simulate = true;
    
    public List<GameObject> particles;

    public List<Vector3> nextPosition;
    public List<Vector3> nowPosition;
    public List<Vector3> prevPosition;
    
    public Vector3 fext = Vector3.zero;                 // 外力
    public Vector3 gravity = new(0, -9.8f, 0);    // 重力
    
    public float dt = 0.002f;                   // 时间微分
    public float mass;                          // 质量
    public float stiffness;                     // 刚度
    [Range(0.0f, 1.0f)]public float damping;    // 阻尼

    public float accTime;
    
    private void Start()
    {
        int cnt = particles.Count;
        Debug.Log("Exsits " + cnt + " particles");
        prevPosition = new List<Vector3>();
        nowPosition = new List<Vector3>();
        nextPosition = new List<Vector3>();
        
        for (var i = 0; i < cnt; i++)
        {
            Debug.Log("particle, position:" + particles[i].transform.position + " index:" + i);
            prevPosition.Add(particles[i].transform.position);
            nowPosition.Add(particles[i].transform.position);
            nextPosition.Add(particles[i].transform.position);
        }
    }

    private void Update()
    {
        // None simulate, quick return
        if (!simulate || !CheckNecessaryCondition())
        {
            return;
        }
        
        // calculate elapsed time(by dt), and do the simulator
        accTime += Time.deltaTime;
        int cnt = (int)(accTime / dt);

        for (int i = 0; i < cnt; i++)
        {
            Simulator();
        }

        accTime %= dt;
    }

    private void OnDestroy()
    {
        Debug.Log("Done");
    }

    /// <summary>
    /// The necessary condition of simulation
    /// </summary>
    /// <returns>true - satisfy the condition</returns>
    public bool CheckNecessaryCondition()
    {
        // check particles
        if (particles == null || particles.Count <= 0)
        {
            return false;
        }

        return true;
    }

    /// <summary>
    /// Do the simulator
    /// </summary>
    public void Simulator()
    {
        // Do Gravite
        SimulatorGravite();
        // Do Force extend
        SimulatorForceExtend();
        // Do Collider
        SimulatorCollider();
        // Do Constraint
        SimulatorConstraint();
        // Update
        SimulatorUpdate();
    }

    public void SimulatorGravite()
    {
        for (int i = 0; i < nextPosition.Count; i++)
        {
            nextPosition[i] = nowPosition[i] + (1 - damping) * (nowPosition[i] - prevPosition[i]) + gravity * dt * dt;
        }
    }

    public void SimulatorForceExtend()
    {
        
    }

    public void SimulatorCollider()
    {
        
    }

    public void SimulatorConstraint()
    {
        
    }

    public void SimulatorUpdate()
    {
        for (int i = 0; i < particles.Count; i++)
        {
            particles[i].transform.position = nextPosition[i];
            nowPosition[i] = nextPosition[i];
            prevPosition[i] = nowPosition[i];
        }
    }
}
