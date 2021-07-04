using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class DeformableSolid : MonoBehaviour {
    /// <summary>
    /// Default constructor. Zero all. 
    /// </summary>
    public DeformableSolid() {
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
    public float tractionStiffness = 100;
    
    public float damping = 1.0f;

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

    public TextAsset nodesText;
    public TextAsset tetraText;

    public float density;
    #endregion

    #region OtherVariables

    private Mesh _mesh;
    private Vector3[] _vertices;

    public List<Node> Nodes;
    public List<Spring> Springs;
    public List<Tetrahedron> Tetras;

    private Dictionary<Vector2, Spring> uniqueEdges;

    #endregion

    #region MonoBehaviour

    public void Awake() {
        _mesh = GetComponent<MeshFilter>().mesh;
        
        string[] lines = TextParser(nodesText);
        Nodes = new List<Node>();
        Springs = new List<Spring>();

        for (int i = 5; i < lines.Length; i+=4)
        {
            float x = float.Parse(lines[i], CultureInfo.InvariantCulture);
            float y = float.Parse(lines[i+1], CultureInfo.InvariantCulture);
            float z = float.Parse(lines[i+2], CultureInfo.InvariantCulture);

            Vector3 position = new Vector3(x, y, z);
            Nodes.Add(new Node(transform.TransformPoint(position), Nodes.Count));
        }
        
        

        lines = TextParser(tetraText);
        Tetras = new List<Tetrahedron>();
        
        uniqueEdges = new Dictionary<Vector2, Spring>();

        for (int i = 4; i < lines.Length; i+=5) {
            var node1 = int.Parse(lines[i], CultureInfo.InvariantCulture) -1;
            var node2 = int.Parse(lines[i+1], CultureInfo.InvariantCulture) -1;
            var node3 = int.Parse(lines[i+2], CultureInfo.InvariantCulture) -1;
            var node4 = int.Parse(lines[i+3], CultureInfo.InvariantCulture) -1;

            var tetra = new Tetrahedron(Nodes[node1], Nodes[node2], Nodes[node3], Nodes[node4]);
            Tetras.Add(tetra);
            var tetraVolume = tetra.Volume();

            int [] auxNodes = {node1, node2, node3, node4};


            for (int j = 0; j < auxNodes.Length-1; j++) {
                for (int k= j + 1; k < auxNodes.Length; k++) {
                    Vector2Int edge = new Vector2Int(Mathf.Min(auxNodes[j], auxNodes[k]), Mathf.Max(auxNodes[j], auxNodes[k]));

                    if (!uniqueEdges.ContainsKey(edge)) {
                        Spring sp = new Spring(Nodes[edge.x], Nodes[edge.y]);
                        sp.TetraMass(tetraVolume / 6);
                        uniqueEdges.Add(edge, sp);
                    }
                }
            }
        }
        
        foreach (var s in uniqueEdges.Values)
        {
            Springs.Add(s);
        }
        
        
        
        for (int i = 0; i < _mesh.vertices.Length; i++)
        {
            for (int j = 0; j < Tetras.Count; j++) {
                var volume = Tetras[j].Volume();


                if (Tetras[j].IsInside(transform.TransformPoint(_mesh.vertices[i]))) {
                    var w1 = TetraVolume(transform.TransformPoint(_mesh.vertices[i]), Tetras[j].Node2.Position, Tetras[j].Node3.Position, Tetras[j].Node4.Position) / volume;
                    var w2 = TetraVolume(transform.TransformPoint(_mesh.vertices[i]), Tetras[j].Node1.Position, Tetras[j].Node3.Position, Tetras[j].Node4.Position) / volume;
                    var w3 = TetraVolume(transform.TransformPoint(_mesh.vertices[i]), Tetras[j].Node1.Position, Tetras[j].Node2.Position, Tetras[j].Node4.Position) / volume;
                    var w4 = TetraVolume(transform.TransformPoint(_mesh.vertices[i]), Tetras[j].Node1.Position, Tetras[j].Node2.Position, Tetras[j].Node3.Position) / volume;
                    
                    
                    Tetras[j].WeightedVertices.Add(new VertexWeight(i, w1, w2, w3, w4));
                }

                var tetraMass = volume * density;
                
                Tetras[j].Node1.UpdateMass(tetraMass * 0.25f);;
                Tetras[j].Node2.UpdateMass(tetraMass * 0.25f);
                Tetras[j].Node3.UpdateMass(tetraMass * 0.25f);
                Tetras[j].Node4.UpdateMass(tetraMass * 0.25f);
                
                
            }
        }
        

        foreach (var n in Nodes) {
            n.mass = n.mass / n.tetraNum;
        }
        
        foreach (var s in Springs) {
            s.mass = s.mass / s.tetraNum;
        }
        
        if (fixerMove) {
            anim.enabled = !paused ? true : false;
        }
        
        
        
    }

    public void Update() {
        if (Input.GetKeyUp(KeyCode.P))
            paused = !paused;

        if(anim)
            anim.enabled = !paused ? true : false;
        
        
        _vertices = _mesh.vertices;
        
        foreach (var t in Tetras)
        {
            foreach (var v in t.WeightedVertices)
            {
                _vertices[v.meshIdx] = t.Node1.Position * v.w1 + t.Node2.Position * v.w2 + t.Node3.Position * v.w3 + t.Node4.Position * v.w4;
                _vertices[v.meshIdx] = transform.InverseTransformPoint(_vertices[v.meshIdx]);
                
            }

        }

        _mesh.vertices = _vertices;

        

    }

    public void FixedUpdate() {
        if (paused)
            return; // Not simulating

        foreach (var node in Nodes) {
            node.Force = Vector3.zero;
            node.ComputeForces(gravity, damping);
        }

        foreach (var spring in Springs) {
            spring.ComputeBasicSpringForces(tractionStiffness, density, damping);
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
                
                
            node.SolveForces(integration, Time.fixedDeltaTime);
        }
        
        foreach (var spring in Springs) {
            spring.UpdateLength();
        }
        
        
        

    }

    #endregion
    
    
    
    public string[] TextParser(TextAsset file) {
        return file.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);
    }
    
    public float TetraVolume(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return Mathf.Abs(Vector3.Dot((p1 - p0), Vector3.Cross((p2 - p0), (p3 - p0))))/6;
    }

    public class Node {
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 Force;
        public int Id;
        public bool Fixed;
        public float mass;
        public int tetraNum;

        public Node(Vector3 position, int id) {
            Position = position;
            Id = id;
            Velocity = Vector3.zero;
            Force = Vector3.zero;
            Fixed = false;
            //mass = 3;
        }

        public void ComputeForces(Vector3 gravity, float damping) {
            Force += mass * gravity;
            Force -= Velocity * damping;
        }

        public void SolveForces(Integration integration, float delta) {
            
            if (!Fixed) {
                switch (integration) {
                    case Integration.Explicit:
                        Position += Velocity * delta;
                        Velocity += Force * delta;
                        break;
                    case Integration.Symplectic:
                        Velocity += Force * delta;
                        Position += Velocity * delta;
                        break;
                }
            }
            
        }

        public void MassPerTetra() {
            mass = mass / tetraNum;
        }

        public void UpdateMass(float m) {
            mass += m;
            tetraNum++;
        }
        
        
    }
    

    public class Spring {
        public Node nodeA, nodeB;

        public float Length0, Length;

        public int tetraNum;
        public float mass;

        public Spring(Node nodeA, Node nodeB) {
            this.nodeA = nodeA;
            this.nodeB = nodeB;

            UpdateLength();
            Length0 = Length;
        }

        public void UpdateLength() {
            Length = (nodeA.Position - nodeB.Position).magnitude;
        }

        public void ComputeBasicSpringForces(float tractionStiffness, float density, float damping) {
            var u = nodeA.Position - nodeB.Position;
            u.Normalize();
            
            Vector3 force = ((nodeA.Position- nodeB.Position) / Length) * (-(mass / (Length0 * Length0)) * density * (Length - Length0) * tractionStiffness);


            force += -damping * (nodeA.Velocity - nodeB.Velocity);
            force += -damping * Vector3.Dot(u, (nodeA.Velocity - nodeB.Velocity)) * u;
            nodeA.Force += force;
            nodeB.Force -= force;
        }

        public void TetraMass(float w) {
            mass += w;
            tetraNum++;
        }
    }

    public class Tetrahedron {
        public Node Node1;
        public Node Node2;
        public Node Node3;
        public Node Node4;
        public List<VertexWeight> WeightedVertices;
        
        public Tetrahedron(Node n1, Node n2, Node n3, Node n4) {
            Node1 = n1;
            Node2 = n2;
            Node3 = n3;
            Node4 = n4;

            WeightedVertices = new List<VertexWeight>();
        }

        public float Volume() {
            return Mathf.Abs(Vector3.Dot((Node2.Position - Node1.Position), Vector3.Cross((Node3.Position - Node1.Position), (Node4.Position - Node1.Position))))/6;
        }

        public bool IsInside(Vector3 pos) {
            var faceCenter = (Node1.Position + Node2.Position + Node3.Position) / 3;
            var normal = Vector3.Cross(Node1.Position - Node2.Position, Node3.Position - Node2.Position);

            if (Vector3.Dot(faceCenter - pos, normal) > 0) {
                faceCenter = (Node1.Position + Node3.Position + Node4.Position) / 3;
                normal = Vector3.Cross(Node1.Position - Node3.Position, Node4.Position - Node3.Position);
                
                if (Vector3.Dot(faceCenter - pos, normal) > 0) {
                    faceCenter = (Node1.Position + Node2.Position + Node4.Position) / 3;
                    normal = Vector3.Cross(Node2.Position - Node1.Position, Node4.Position - Node1.Position);
                    
                    if (Vector3.Dot(faceCenter - pos, normal) > 0) {
                        faceCenter = (Node2.Position + Node3.Position + Node4.Position) / 3;
                        normal = Vector3.Cross(Node3.Position - Node2.Position, Node4.Position - Node3.Position);
                
                        if (Vector3.Dot(faceCenter - pos, normal) > 0) {
                            return true;
                        }
                    }
                }
            }

            return false;
        }
    }
    
    public struct VertexWeight {
        public int meshIdx;
        
        public float w1;
        public float w2;
        public float w3;
        public float w4;
        
        public VertexWeight(int idx, float w1, float w2, float w3, float w4) {
            meshIdx = idx;
            
            this.w1 = w1;
            this.w2 = w2;
            this.w3 = w3;
            this.w4 = w4;
        }
    }
    
    public void OnDrawGizmos()
    {
        if (Nodes!= null)
        {
            foreach (var n in Nodes)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawSphere(n.Position, 0.05f);
            }
            foreach (var spring in Springs)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(spring.nodeA.Position, spring.nodeB.Position);
            }

        }
    }
}