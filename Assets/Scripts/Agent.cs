using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();

    private HashSet<GameObject> collidingNeighbors = new HashSet<GameObject>();
    void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
    {
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        }
        else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (false)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }

        if (true)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    #endregion

    #region Incomplete Functions

    private Vector3 ComputeForce()
    {
        var force = CalculateGoalForce() + CalculateAgentForce(); //subject to change

        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        }
        else
        {
            return Vector3.zero;
        }
    }

    private Vector3 CalculateGoalForce()
    {
        var goalDir = Vector3.Normalize(nma.destination - transform.position);
        var goalForce = rb.mass * (Parameters.maxSpeed * goalDir - GetVelocity()) / Time.deltaTime;
        return goalForce;
    }

    private Vector3 CalculateAgentForce()
    {
        var A = Parameters.A;
        var B = Parameters.B;
        var k = Parameters.k;
        var kappa = Parameters.Kappa;

        var agentForce = Vector3.zero;

        foreach (var j in perceivedNeighbors)
        {
            if (!AgentManager.IsAgent(j))
            {
                continue;
            }
            var neighbor = AgentManager.agentsObjs[j];

            var dir = (transform.position - neighbor.transform.position).normalized;
            var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, neighbor.transform.position);

            agentForce += A * Mathf.Exp(overlap / B) * dir;
            agentForce += k * (overlap > 0f ? overlap : 0) * dir;

            var tangent = Vector3.Cross(Vector3.up, dir);
            agentForce += kappa * (overlap > 0f ? overlap : 0) * Vector3.Dot(rb.velocity - neighbor.GetVelocity(), tangent) * tangent;
        }
        return agentForce;
    }

    private Vector3 CalculateWallForce()
    {
        return Vector3.zero;
    }

    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            //agent detected
            perceivedNeighbors.Add(other.gameObject);
            // Debug.Log( name +" Detected " + other.name);
        }
    }

    public void OnTriggerExit(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            //agent detected
            perceivedNeighbors.Remove(other.gameObject);

            // Debug.Log( name +" Detected " + other.name);
        }
    }

    public void OnCollisionEnter(Collision collision)
    {
        if (WallManager.IsWall(collision.gameObject))
        {

        }
    }

    public void OnCollisionExit(Collision collision)
    {
        if (WallManager.IsWall(collision.gameObject))
        {

        }
    }

    #endregion
}
