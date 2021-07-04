using System;
using System.Collections.Generic;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class MassSpringCloth : MonoBehaviour {
    /// <summary>
    /// Default constructor. Zero all. 
    /// </summary>
    public MassSpringCloth() {
        paused = true;
        gravity = new Vector3(0.0f, -9.81f, 0.0f);
        integration = Integration.Symplectic;
    }

    /// <summary>
    /// Integration method.
    /// </summary>
    public enum Integration {
        Explicit = 0,
        Symplectic = 1
    }

    #region InEditorVariables

    public bool paused;
    public Vector3 gravity;
    public Integration integration;
    public float massPerNode = 1;
    public float tractionStiffness = 100;
    public float flexionStiffness = 50;

    public bool enableDamping = true;
    public float damping = 1.0f;

    public bool windEnabled = false;
    public float windFriction = 1.0f;
    public Vector3 windVel;


    public float k = 1;
    
    public GameObject floor;
    public float floorHeight;
    public bool floorCollision;
    public float floorOffset;

    public GameObject sphere;
    public float ratius;
    public bool sphereCollision;
    public float sphereOffset;

    public Animator anim;
    public bool fixerMove;
    #endregion

    #region OtherVariables

    private Mesh _mesh;
    private Vector3[] _vertices;
    private Spring[] _springs;

    public Node[] Nodes { get; private set; }
    public Triangle[] Triangles;

    #endregion

    #region MonoBehaviour

    public void Awake() {
        _mesh = GetComponent<MeshFilter>().mesh;
        _vertices = _mesh.vertices;

        if (fixerMove) {
            anim.enabled = !paused ? true : false;
        }
            
        
        Nodes = new Node[_vertices.Length];

        for (var i = 0; i < _vertices.Length; i++) {
            Nodes[i] = new Node(transform.TransformPoint(_vertices[i]));
        }
        
        
        
        GenerateSprings(_mesh.triangles);
    }

    public void Update() {
        if (Input.GetKeyUp(KeyCode.P))
            paused = !paused;

        if(anim)
            anim.enabled = !paused ? true : false;
        
        for (var i = 0; i < Nodes.Length; i++) {
            _vertices[i] = transform.InverseTransformPoint(Nodes[i].Position);
        }

        foreach (var spring in _springs) {
            Debug.DrawLine(spring.nodeA.Position, spring.nodeB.Position,
                spring.Flexion ? Color.blue : Color.red);
        }

        _mesh.vertices = _vertices;
    }

    public void FixedUpdate() {
        if (paused)
            return; // Not simulating

        foreach (var node in Nodes) {
            node.ComputeGravity(gravity, massPerNode);
            if (enableDamping) node.ComputeDamping(damping);
        }

        foreach (var spring in _springs) {
            spring.ComputeBasicSpringForces(tractionStiffness, flexionStiffness);
            if (enableDamping) spring.ComputeDamping(damping);
        }
        
        if(floorCollision)
            floorHeight = floor.transform.position.y + floorOffset;
        
        if(sphereCollision)
            ratius = (sphere.transform.localScale.x/ 2) + sphereOffset;
        
        foreach (var node in Nodes) {
            if (floorCollision) {
                if (node.Position.y < floorHeight)
                    node.Force += k * Math.Abs(floorHeight - node.Position.y) * Vector3.up;
            }

            if (sphereCollision) {
                if((node.Position - sphere.transform.position).magnitude < ratius)
                    node.Force += k * Math.Abs(ratius - (node.Position - sphere.transform.position).magnitude) * (node.Position - sphere.transform.position).normalized;
            }
                
                
            node.solveForces(integration, Time.fixedDeltaTime, massPerNode);
        }

        foreach (var spring in _springs) {
            spring.UpdateLength();
        }

        foreach (var triangle in Triangles) {
            if (windEnabled) {
                triangle.ComputeWind(windFriction, windVel);
            }
        }
    }

    #endregion

    private void GenerateSprings(int[] triangles) {
        var comparer = Comparer<Vector3Int>.Create((o1, o2) => {
            var value = o1.x.CompareTo(o2.x);
            if (value == 0) value = o1.y.CompareTo(o2.y);
            if (value == 0) value = o1.z.CompareTo(o2.z);
            return value;
        });

        var set = new SortedSet<Vector3Int>(comparer);
        Triangles = new Triangle [_mesh.triangles.Length / 3];
        
        var i = 0;
        var t = 0;
        while (i < triangles.Length) {
            var a = triangles[i++];
            var b = triangles[i++];
            var c = triangles[i++];

            set.Add(new Vector3Int(Math.Min(a, b), Math.Max(a, b), c));
            set.Add(new Vector3Int(Math.Min(a, c), Math.Max(a, c), b));
            set.Add(new Vector3Int(Math.Min(b, c), Math.Max(b, c), a));

            Triangles[t++] = new Triangle(Nodes[a], Nodes[b], Nodes[c]);
        }

        var list = new List<Spring>();
        var previous = new Vector3Int(-1, -1, -1);
        foreach (var edge in set) {
            Node first, second;
            bool flexion;

            if (edge.x == previous.x && edge.y == previous.y) {
                first = Nodes[edge.z];
                second = Nodes[previous.z];
                flexion = true;
            }
            else {
                first = Nodes[edge.x];
                second = Nodes[edge.y];
                flexion = false;
            }

            list.Add(new Spring(first, second, flexion));
            previous = edge;
        }

        _springs = list.ToArray();
    }

    public class Node {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 Force;
        public bool Fixed;

        public Node(Vector3 position) {
            Position = position;
            Velocity = Vector3.zero;
            Force = Vector3.zero;
            Fixed = false;
        }

        public void ComputeGravity(Vector3 gravity, float mass) {
            Force += mass * gravity;
        }

        public void ComputeDamping(float damping) {
            Force -= Velocity * damping;
        }

        public void solveForces(Integration integration, float delta, float mass) {
            
            if (!Fixed) {
                switch (integration) {
                    case Integration.Explicit:
                        Position += Velocity * delta;
                        Velocity += Force * delta / mass;
                        break;
                    case Integration.Symplectic:
                        Velocity += Force * delta / mass;
                        Position += Velocity * delta;
                        break;
                }
            }

            Force = Vector3.zero;
        }
        
    }

    public class Triangle{
        private Node NodeA, NodeB, NodeC;

        public Triangle(Node nodeA, Node nodeB, Node nodeC) {
            NodeA = nodeA;
            NodeB = nodeB;
            NodeC = nodeC;
        }

        public void ComputeWind(float windFriction, Vector3 windVel) {
            var vel = (NodeA.Velocity + NodeB.Velocity + NodeC.Velocity) / 3;
            var n = Vector3.Cross(NodeB.Position - NodeA.Position, NodeC.Position - NodeA.Position);

            var a = n.magnitude / 2;

            NodeA.Force += windFriction * a *(Vector3.Dot(n.normalized, (windVel - vel)))*n;
            NodeB.Force += windFriction * a *(Vector3.Dot(n.normalized, (windVel - vel)))*n;
            NodeC.Force += windFriction * a *(Vector3.Dot(n.normalized, (windVel - vel)))*n;

        }
    }

    public class Spring {
        public Node nodeA, nodeB;

        public float Length0;
        public float Length;
        public bool Flexion;

        public Spring(Node nodeA, Node nodeB, bool flexion) {
            this.nodeA = nodeA;
            this.nodeB = nodeB;
            Flexion = flexion;

            UpdateLength();
            Length0 = Length;
        }

        public void UpdateLength() {
            Length = (nodeA.Position - nodeB.Position).magnitude;
        }

        public void ComputeBasicSpringForces(float tractionStiffness, float flexionStiffness) {
            var u = nodeA.Position - nodeB.Position;
            u.Normalize();

            var stiffness = Flexion ? flexionStiffness : tractionStiffness;

            var force = -stiffness * (Length - Length0) * u;
            nodeA.Force += force;
            nodeB.Force -= force;
        }

        public void ComputeDamping(float damping) {
            var u = nodeA.Position - nodeB.Position;
            u.Normalize();

            var force = -damping * Vector3.Dot(u, nodeA.Velocity - nodeB.Velocity) * u;
            nodeA.Force += force;
            nodeB.Force -= force;
        }
    }
}