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
    private Vector3 dest;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();
    private Dictionary<GameObject, Vector3> perceivedWalls = new Dictionary<GameObject, Vector3>();
    private bool evader;
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

        if (false)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions
    public void setDest(Vector3 t)
    {
        dest = t;
    }
    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        nma.ResetPath();
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;
    }

    //

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    #endregion

    #region Incomplete Functions

    private Vector3 ComputeForce()
    {

        //var force = CalculateCrowdFollow(4);
        //var force = CalculateFollowLeader(2);
        //var force = CalculatePursueEvade(2);
        //var force = CalculateSpiralForce(.025f);
        var force = CalculateAgentForce(.5f) + CalculateGoalForce(2) + CalculateWallForce(.5f);
        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        }
        else
        {
            return Vector3.zero;
        }
    }

    private Vector3 CalculateGoalForce(float maxSpeed)
    {

        var goalForce = Vector3.zero;
        if (path.Count == 0)
        {
            return goalForce;
        }


        var goalDir = (path[0] - transform.position).normalized; // direction agent must 'steer' in
        var desiredSpeed = goalDir.magnitude * maxSpeed; // find the speed that we need to travel

        goalForce = ((goalDir * desiredSpeed) - GetVelocity()) / Parameters.T;

        return goalForce;
    }
    private float g(float x)
    {
        //Returns 0 if they dont touch, x if they do.

        if (x > 0.0)
            return x;

        return 0;
    }
    private Vector3 CalculateAgentForce(float multiplier)
    {

        var agentForce = Vector3.zero;

        foreach (var n in perceivedNeighbors)
        {
            if (AgentManager.IsAgent(n))
            {
                var neighbor = AgentManager.agentsObjs[n];

                var dir = (transform.position - neighbor.transform.position).normalized;

                var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, n.transform.position);

                var collisionAvoidance = Parameters.A * Mathf.Exp(overlap / Parameters.B);
                var nonPenetrating = multiplier * Parameters.k * g(overlap);

                var tangent = Vector3.Cross(Vector3.up, dir);
                var tanVelDiff = Vector3.Dot(GetVelocity() - neighbor.GetVelocity(), tangent);
                var slidingForce = Parameters.Kappa * g(overlap) * tanVelDiff * tangent;

                agentForce += ((collisionAvoidance + nonPenetrating) * dir + slidingForce);
            }
        }
        return agentForce;
    }

    private Vector3 CalculateWallForce(float multiplier)
    {

        var wallForce = Vector3.zero;

        foreach (var w in perceivedWalls)
        {

            var normal = w.Value;

            var overlap = radius - Vector3.Distance(transform.position, w.Key.transform.position);

            var collisionAvoidance = Parameters.WALL_A * Mathf.Exp(overlap / Parameters.WALL_B);
            var nonPenetrating = Parameters.WALL_k * g(overlap);

            var tan = Vector3.Cross(Vector3.up, normal);
            var slidingForce = (Parameters.WALL_Kappa * g(overlap)) * Vector3.Dot(GetVelocity(), tan) * tan;

            wallForce += ((collisionAvoidance + nonPenetrating) * normal - slidingForce);
        }
        return wallForce;
    }
    private Vector3 CalculateCrowdFollow(float maxSpeed)
    {

        if (path.Count == 0)
            return Vector3.zero;

        GetComponent<SphereCollider>().radius = 5;

        var goalDir = path[0] - transform.position;
        float dist = Vector3.Distance(path[0], transform.position);

        var agentForce = Vector3.zero;
        var nonPen = 0f;
        var collisionAvoid = 0f;
        Vector3 slideForce = Vector3.zero;

        foreach (var n in perceivedNeighbors)
        {
            if (AgentManager.IsAgent(n))
            {
                var neighbor = AgentManager.agentsObjs[n];

                float neighborDist = Vector3.Distance(path[0], n.transform.position);

                if (dist > neighborDist)
                {
                    // Basically if one of the neighbors is closer to the goal, follow the neighbor.
                    goalDir = n.transform.position - transform.position;
                }
                var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, n.transform.position);
                collisionAvoid += Mathf.Exp(overlap / Parameters.B);
                nonPen += Parameters.k * g(overlap);
            }
        }

        var desiredVel = goalDir.normalized * maxSpeed;

        agentForce += (desiredVel - GetVelocity()) / Parameters.T;

        agentForce += (collisionAvoid + nonPen) * goalDir + slideForce;

        /*  var tangent = Vector3.Cross(Vector3.up, goalDir.normalized);
          agentForce += tangent;*/

        foreach (var w in perceivedWalls)
        {

            var normal = w.Value;

            var overlap = radius - Vector3.Distance(transform.position, w.Key.transform.position);

            var collisionAvoidance = Parameters.WALL_A * Mathf.Exp(overlap / Parameters.WALL_B);
            var nonPenetrating = Parameters.WALL_k * g(overlap);

            var tan = Vector3.Cross(Vector3.up, normal);
            var slidingForce = (Parameters.WALL_Kappa * g(overlap)) * Vector3.Dot(GetVelocity(), tan) * tan;

            agentForce += ((collisionAvoidance + nonPenetrating) * normal - slidingForce);
        }
        return agentForce;


    }


    public Vector3 CalculateFollowLeader(float maxSpeed)
    {
        if (path.Count == 0)
        {
            return Vector3.zero;
        }

        var agentForce = Vector3.zero;
        var speed = 0.1f;
        var panic = .5f;
        GetComponent<SphereCollider>().radius = 10;

        bool isLeader = (int.Parse(name.Split(' ')[1])) == 0;
        if (isLeader)
        {
            Debug.DrawLine(transform.position, transform.position + Vector3.up * 3, Color.green, 0.1f);


            var goal = path[0] - transform.position;
            var desiredSpeed = goal.normalized * maxSpeed * 1.5f;

            agentForce += (desiredSpeed - GetVelocity()) / Parameters.T;

            foreach (var n in perceivedNeighbors)
            {
                if (AgentManager.IsAgent(n))
                {
                    var neighbor = AgentManager.agentsObjs[n];

                    var dir = (transform.position - neighbor.transform.position).normalized;

                    var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, n.transform.position);

                    var collisionAvoidance = Parameters.A * Mathf.Exp(overlap / Parameters.B);
                    var nonPenetrating = Parameters.k * g(overlap);

                    var tangent = Vector3.Cross(Vector3.up, dir);
                    var tanVelDiff = Vector3.Dot(GetVelocity() - neighbor.GetVelocity(), tangent);
                    var slidingForce = Parameters.Kappa * g(overlap) * tanVelDiff * tangent;

                    agentForce += ((collisionAvoidance + nonPenetrating) * dir + slidingForce);
                }
            }
        }
        else
        {

            Debug.DrawLine(transform.position, transform.position + Vector3.up * 3, Color.red, 0.1f);

            var leaderPosition = AgentManager.leader[0].transform.position;
            var avgDir = Vector3.zero;
            var collisionAvoid = 0f;
            var nonPen = 0f;
            var slideForce = Vector3.zero;
            foreach (var n in perceivedNeighbors)
            {
                if (AgentManager.IsAgent(n))
                {
                    //making sure agents are not too close to the leader
                    if (Vector3.Distance(leaderPosition, n.transform.position) > 3f)
                    {
                        var neighbor = AgentManager.agentsObjs[n];
                        var goal = leaderPosition - n.transform.position;

                        avgDir += goal;

                        var overlap = (radius + neighbor.radius) - Vector3.Distance(leaderPosition, n.transform.position);
                        collisionAvoid += Parameters.A * Mathf.Exp(overlap / Parameters.B);
                        /* nonPen += Parameters.k * g(overlap);*/

                        var tangent = Vector3.Cross(Vector3.up, goal);
                        var tanVelDiff = Vector3.Dot(GetVelocity() - neighbor.GetVelocity(), tangent);
                        slideForce += Parameters.Kappa * g(overlap) * tanVelDiff * tangent;
                    }
                }
            }

            var goalDir = leaderPosition - transform.position;
            avgDir /= perceivedNeighbors.Count;


            var panicDir = ((1 - panic) * goalDir + panic * avgDir).normalized;

            agentForce += (collisionAvoid + nonPen) * panicDir + slideForce;
            var desiredSpeed = panicDir * maxSpeed;
            agentForce += ((desiredSpeed - GetVelocity()) / Parameters.T);

        }
        foreach (var w in perceivedWalls)
        {

            var normal = w.Value;

            var overlap = radius - Vector3.Distance(transform.position, w.Key.transform.position);

            var collisionAvoidance = Parameters.WALL_A * Mathf.Exp(overlap / Parameters.WALL_B);
            var nonPenetrating = Parameters.WALL_k * g(overlap);

            var tan = Vector3.Cross(Vector3.up, normal);
            var slidingForce = (Parameters.WALL_Kappa * g(overlap)) * Vector3.Dot(GetVelocity(), tan) * tan;

            agentForce += ((collisionAvoidance + nonPenetrating) * normal - slidingForce);
        }
        return agentForce;
    }
    private Vector3 CalculatePursueEvade(float maxSpeed)
    {
        if (path.Count == 0)
        {
            return Vector3.zero;
        }
        var agentForce = Vector3.zero;

        GetComponent<SphereCollider>().radius = 10;

        bool isEvader = (int.Parse(name.Split(' ')[1]) % 5) == 0;
        if (isEvader)
        {
            Debug.DrawLine(transform.position, transform.position + Vector3.up * 3, Color.green, 0.1f);
            evader = true;
            //float remainingDist = Vector3.Distance(path[0], transform.position);
            /*  if (remainingDist < 3f || path.Count == 0)
              {
                  var d = Random.insideUnitCircle * 20;
                  ComputePath(d);
              }*/

            if (path.Count == 0)
            {
                return Vector3.zero;
            }

            var goalDir = (path[0] - transform.position).normalized; // direction agent must 'steer' in
            var desiredSpeed = goalDir.magnitude * maxSpeed * perceivedNeighbors.Count / 3; // find the speed that we need to travel

            agentForce = (((goalDir * desiredSpeed) - GetVelocity()) / Parameters.T);

            foreach (var n in perceivedNeighbors)
            {
                if (AgentManager.IsAgent(n))
                {

                    var neighbor = AgentManager.agentsObjs[n];
                    if (!neighbor.evader)
                    {
                        var dir = (transform.position - n.transform.position).normalized;

                        var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, n.transform.position);

                        var collisionAvoidance = Parameters.A * Mathf.Exp(overlap / Parameters.B);
                        var nonPenetrating = Parameters.k * g(overlap);

                        var tangent = Vector3.Cross(Vector3.up, dir);
                        var tanVelDiff = Vector3.Dot(GetVelocity() - neighbor.GetVelocity(), tangent);
                        var slidingForce = Parameters.Kappa * g(overlap) * tanVelDiff * tangent;

                        agentForce += ((collisionAvoidance + nonPenetrating) * dir + slidingForce);
                    }
                }
            }
        }
        else
        {
            evader = false;
            Debug.DrawLine(transform.position, transform.position + Vector3.up * 3, Color.red, 0.1f);
            foreach (var n in perceivedNeighbors)
            {
                if (AgentManager.IsAgent(n))
                {

                    var neighbor = AgentManager.agentsObjs[n];
                    var dir = (transform.position - n.transform.position).normalized;

                    var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, n.transform.position);

                    var collisionAvoidance = Parameters.A * Mathf.Exp(overlap / Parameters.B);
                    var nonPenetrating = Parameters.k * g(overlap);

                    var tangent = Vector3.Cross(Vector3.up, dir);
                    var tanVelDiff = Vector3.Dot(GetVelocity() - neighbor.GetVelocity(), tangent);
                    var slidingForce = Parameters.Kappa * g(overlap) * tanVelDiff * tangent;

                    agentForce += ((collisionAvoidance + nonPenetrating) * dir + slidingForce);

                    if (neighbor.evader)
                    {
                        dir = (n.transform.position - transform.position).normalized; // direction agent must 'steer' in
                        var desSpeed = dir.magnitude * maxSpeed; // find the speed that we need to travel

                        agentForce = (((dir * desSpeed) - GetVelocity()) / Parameters.T);
                    }

                }
            }

        }
        foreach (var w in perceivedWalls)
        {

            var normal = w.Value;

            var overlap = radius - Vector3.Distance(transform.position, w.Key.transform.position);

            var collisionAvoidance = Parameters.WALL_A * Mathf.Exp(overlap / Parameters.WALL_B);
            var nonPenetrating = Parameters.WALL_k * g(overlap);

            var tan = Vector3.Cross(Vector3.up, normal);
            var slidingForce = (Parameters.WALL_Kappa * g(overlap)) * Vector3.Dot(GetVelocity(), tan) * tan;

            agentForce += ((collisionAvoidance + nonPenetrating) * normal - slidingForce);
        }
        return agentForce;
    }

    private Vector3 CalculateSpiralForce(float maxSpeed)
    {
        var origin = Vector3.zero - transform.position;
        if (origin.sqrMagnitude > 0)
            return Vector3.Cross(Vector3.up, origin).normalized * Mathf.Sin(maxSpeed);

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
            /*    from ta: For all visible walls, compute the normal vector
                You can find the normal by comparing the agent's xz to the wall's xz
                if the x-component is greater than the z-component, zero out the z component and normalize the vector
                this is the normal
                if the zcomponent is greater, zero out the x and normalize*/

            var wall = collision.gameObject;
            var normal = wall.transform.position;
            if (transform.position.x > wall.transform.position.z)
            {
                normal.z = 0;
            }
            else
            {
                normal.x = 0;
            }

            perceivedWalls.Add(collision.gameObject, normal.normalized);
        }
    }

    public void OnCollisionExit(Collision collision)
    {
        if (WallManager.IsWall(collision.gameObject))
        {
            perceivedWalls.Remove(collision.gameObject);
        }
    }

    #endregion
}
