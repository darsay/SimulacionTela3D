using System.Collections.Generic;
using UnityEngine;

public class Fixer : MonoBehaviour {
    public GameObject clothObject;

    private Bounds _bounds;
    private Dictionary<MassSpringCloth.Node, Vector3> _nodes;

    // Possibilities of the Fixer
    void Start() {
        var cloth = clothObject.GetComponent<MassSpringCloth>();
        var nodes = cloth.Nodes;

        _bounds = GetComponent<Collider>().bounds;
        _nodes = new Dictionary<MassSpringCloth.Node, Vector3>();

        foreach (var node in nodes) {
            if (!_bounds.Contains(node.Position)) continue;
            node.Fixed = true;
            _nodes.Add(node, transform.InverseTransformPoint(node.Position));
        }
    }

    private void FixedUpdate() {
        foreach (var pair in _nodes) {
            pair.Key.Position = transform.TransformPoint(pair.Value);
        }
    }
}