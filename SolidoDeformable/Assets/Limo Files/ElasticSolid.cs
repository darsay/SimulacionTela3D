using UnityEngine;
using System.Collections.Generic;
using System;
using System.Globalization;


public class ElasticSolid : MonoBehaviour
{
  
    public ElasticSolid()
    {
        this.Paused = true;
        this.TimeStep = 0.01f;
        this.Gravedad = new Vector3(0.0f, -9.81f, 0.0f);
        this.IntegrationMethod = Integration.Symplectic;
    }

    public enum Integration
    {
        Explicit = 0,
        Symplectic = 1,
    };


    public TextAsset archivoNodos;
    public TextAsset archivoTetra;
    public bool Paused;
    public float TimeStep;
    public Vector3 Gravedad;
    public Integration IntegrationMethod;
    public List<Node> listaNodos = new List<Node>();
    public List<Tetraedro> listaTetra= new List<Tetraedro>();
    public List<Spring> listaSprings = new List<Spring>();
    
    public float densidad = 1;
    public float stiffness = 100;
    public float damping;
    public float substeps=10;

    Mesh malla;      
      
    public void Awake()
    {
        malla = GetComponentInChildren<MeshFilter>().mesh;

        string[] archivo = getTextParser(archivoNodos);

        //Se recorre el txt y se añaden los distintos nodos a la lista
        for (int i = 5; i < archivo.Length; i += 4)
        {
            float x = float.Parse(archivo[i], CultureInfo.InvariantCulture);
            float y = float.Parse(archivo[i + 1], CultureInfo.InvariantCulture);
            float z = float.Parse(archivo[i + 2], CultureInfo.InvariantCulture);

            Vector3 positionAux = new Vector3(x, y, z);
            positionAux = transform.TransformPoint(positionAux);
            listaNodos.Add(new Node(positionAux, false, listaNodos.Count));

        }


        archivo = getTextParser(archivoTetra);    
        //Diccionario para las aristas sin repetir.
        Dictionary<Arista, Spring> aristasDic = new Dictionary<Arista, Spring>();        

        //Se recorre el otro txt y se añaden los tetraedros a la lista.
        for (int i = 4; i < archivo.Length; i += 5)
        {       

            int nodo1 = int.Parse(archivo[i], CultureInfo.InvariantCulture) - 1;
            int nodo2 = int.Parse(archivo[i + 1], CultureInfo.InvariantCulture) - 1;
            int nodo3 = int.Parse(archivo[i + 2], CultureInfo.InvariantCulture) - 1;
            int nodo4 = int.Parse(archivo[i + 3], CultureInfo.InvariantCulture) - 1;


            Tetraedro tetraedro = new Tetraedro(listaNodos[nodo1], listaNodos[nodo2], listaNodos[nodo3], listaNodos[nodo4]);
            listaTetra.Add(tetraedro);

            float volumenTotal = tetraedro.volumen;


            List<Arista> ComprobarAristas = new List<Arista>
            {
                new Arista(nodo1, nodo2),
                new Arista(nodo1, nodo3),
                new Arista(nodo1, nodo4),
                new Arista(nodo2, nodo3),
                new Arista(nodo2, nodo4),
                new Arista(nodo3, nodo4)
            };


            //Se comprueba que las 6 posibles aristas no estén ya en el diccionario y se añaden.
            foreach (var ar in ComprobarAristas)
            {
                if (!aristasDic.ContainsKey(ar))
                {
                    Spring spring = new Spring(listaNodos[ar.n1], listaNodos[ar.n2]);
                    spring.MasaTetraedros(volumenTotal / 6);
                    aristasDic.Add(ar, spring);

                }

            }            

        }

        //Se añaden los springs de las aristas no repetidas a una lista.
        foreach (var spr in aristasDic.Values)
        {
           listaSprings.Add(spr);
        }
        
        //Se recorren los vertices de la malla real y se comprueba que esten dentro de cada tetraedro de la lista.
        for (int i = 0; i < malla.vertices.Length; i++)
        {
            for (int j = 0; j < listaTetra.Count; j++)
            {
                
                float vTotal = listaTetra[j].volumen;

                
                if (listaTetra[j].VerticeDentro(transform.TransformPoint(malla.vertices[i])))
                {
                    //Si está dentro, se guarda el peso de cada nodo para ese vértice.
                    float peso1 = listaTetra[j].VTetraedro(transform.TransformPoint(malla.vertices[i]), listaTetra[j].nodo2.pos, listaTetra[j].nodo3.pos, listaTetra[j].nodo4.pos) / vTotal;
                    float peso2 = listaTetra[j].VTetraedro(transform.TransformPoint(malla.vertices[i]), listaTetra[j].nodo1.pos, listaTetra[j].nodo3.pos, listaTetra[j].nodo4.pos) / vTotal;
                    float peso3 = listaTetra[j].VTetraedro(transform.TransformPoint(malla.vertices[i]), listaTetra[j].nodo1.pos, listaTetra[j].nodo2.pos, listaTetra[j].nodo4.pos) / vTotal;
                    float peso4 = listaTetra[j].VTetraedro(transform.TransformPoint(malla.vertices[i]), listaTetra[j].nodo1.pos, listaTetra[j].nodo2.pos, listaTetra[j].nodo3.pos) / vTotal;
                    
                    
                    
                    listaTetra[j].vertexIn.Add(new DistribucionPesos(i, peso1, peso2, peso3, peso4));
                }

                float masaTetraedro = vTotal * densidad;
                float masaPorNodo = masaTetraedro * 0.25f;
                
                listaTetra[j].nodo1.MasaNodos(masaPorNodo);
                listaTetra[j].nodo2.MasaNodos(masaPorNodo);
                listaTetra[j].nodo3.MasaNodos(masaPorNodo);
                listaTetra[j].nodo4.MasaNodos(masaPorNodo);


            }
        }
        /*
        //Se actualizan las masas de los nodos y springs dependiendo de los tetraedros y pesos.
        for (int i = 0; i < listaNodos.Count; i++)
        {
            listaNodos[i].MasaEntreTetra();
        }
        for (int i = 0; i < listaSprings.Count; i++)
        {
            listaSprings[i].MediaPesos();
        }

*/
    }

    public void Update()
    {
        if (Input.GetKeyUp(KeyCode.P))
            this.Paused = !this.Paused;

    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; 

        for (int i = 0; i < substeps; i++)
        {
            switch (this.IntegrationMethod)
            {
                case Integration.Explicit: this.stepExplicit(); break;
                case Integration.Symplectic: this.stepSymplectic(); break;
                default:
                    throw new System.Exception("[ERROR] Should never happen!");
            }
        }

        Vector3[] verticesAux = malla.vertices;

        //Se actualiza la posición de la malla real en función de la posición de los nodos que le afectan con su peso.
        foreach (var tetraedro in listaTetra)
        {
            foreach (var vertice in tetraedro.vertexIn)
            {

                verticesAux[vertice.indice] = tetraedro.nodo1.pos * vertice.peso0 + tetraedro.nodo2.pos * vertice.peso1 + tetraedro.nodo3.pos * vertice.peso2 + tetraedro.nodo4.pos * vertice.peso3;
                verticesAux[vertice.indice] = transform.InverseTransformPoint(verticesAux[vertice.indice]);

            }

        }
        malla.vertices = verticesAux;
    }

   

    private string[] getTextParser(TextAsset file)
    {
        string[] textString = file.text.Split(new string[] { " ", "\n", "\r" }, StringSplitOptions.RemoveEmptyEntries);
        return textString;

    }

    private void stepExplicit()
    {
        foreach (var nodo in listaNodos)
        {
            nodo.force = Vector3.zero;
            nodo.ComputeForces(Gravedad, damping);
        }      

        foreach (var spring in listaSprings)
        {          
            spring.ComputeForces(stiffness, densidad, damping);
        }

        foreach (var nodo in listaNodos)
        {
            if (nodo.isfixed == false)
            {
                Vector3 vAux = nodo.vel;
                nodo.vel += ((TimeStep / substeps) / densidad) * nodo.force;
                nodo.pos +=(TimeStep / substeps) * vAux;
            }

        }

        foreach (var spring in listaSprings)
        {
            spring.UpdateLength();
        }
    }

    
    private void stepSymplectic()
    {

        foreach (var nodo in listaNodos)
        {
            nodo.force = Vector3.zero;
            nodo.ComputeForces(Gravedad, damping);

        }       

        foreach (var spring in listaSprings)
        {
            spring.ComputeForces(stiffness, densidad, damping);
        }


        foreach (var nodo in listaNodos)
        {

            if (nodo.isfixed == false)
            {
                nodo.vel += ((TimeStep / substeps) / densidad) * nodo.force;
                nodo.pos +=  (TimeStep / substeps) * nodo.vel;
            }

        }

        foreach (var spring in listaSprings)
        {
            spring.UpdateLength();
        }
    }

    void OnDrawGizmos()
    {
        if (listaNodos != null)
        {
            foreach (var nodo in listaNodos)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawSphere(nodo.pos, 0.05f);
            }
            foreach (var spring in listaSprings)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(spring.nA.pos, spring.nB.pos);
            }

        }
    }
}

public class Tetraedro
{
    public Node nodo1;
    public Node nodo2;
    public Node nodo3;
    public Node nodo4;
    public float volumen;

    public List<DistribucionPesos> vertexIn;

    public Tetraedro(Node n1, Node n2, Node n3, Node n4)
    {
        nodo1 = n1;
        nodo2 = n2;
        nodo3 = n3;
        nodo4 = n4;
        vertexIn = new List<DistribucionPesos>();
        volumen = VTetraedro(nodo1.pos, nodo2.pos, nodo3.pos, nodo4.pos);
    }

    public bool VerticeDentro(Vector3 vertice)
    {

        Vector3 centro1 = (nodo1.pos + nodo2.pos + nodo3.pos) / 3;
        Vector3 normal1 = Vector3.Cross(nodo1.pos - nodo2.pos, nodo3.pos - nodo2.pos);

        Vector3 centro2 = (nodo1.pos + nodo3.pos + nodo4.pos) / 3;
        Vector3 normal2 = Vector3.Cross(nodo1.pos - nodo3.pos, nodo4.pos - nodo3.pos);

        Vector3 centro3 = (nodo1.pos + nodo2.pos + nodo4.pos) / 3;
        Vector3 normal3 = Vector3.Cross(nodo2.pos - nodo1.pos, nodo4.pos - nodo1.pos);

        Vector3 centro4 = (nodo2.pos + nodo3.pos + nodo4.pos) / 3;
        Vector3 normal4 = Vector3.Cross(nodo3.pos - nodo2.pos, nodo4.pos - nodo3.pos);

        if (Vector3.Dot(centro1 - vertice, normal1) > 0)
        {
            if (Vector3.Dot(centro2 - vertice, normal2) > 0)
            {
                if (Vector3.Dot(centro3 - vertice, normal3) > 0)
                {
                    if (Vector3.Dot(centro4 - vertice, normal4) > 0)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    public float VTetraedro(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        float volumen = Mathf.Abs(Vector3.Dot((p1 - p0), Vector3.Cross((p2 - p0), (p3 - p0)))) / 6;
        return volumen;
    }


}

public struct DistribucionPesos
{
    public float peso0;
    public float peso1;
    public float peso2;
    public float peso3;

    public int indice;

    public DistribucionPesos(int id, float p0, float p1, float p2, float p3)
    {
        indice = id;
        peso0 = p0;
        peso1 = p1;
        peso2 = p2;
        peso3 = p3;
    }
}

public struct Arista
{
    public int n1;
    public int n2;

    public Arista(int n1, int n2)
    {
        if (n1 < n2)
        {
            this.n1 = n1;
            this.n2 = n2;
        }
        else
        {
            this.n1 = n2;
            this.n2 = n1;
        }
    }
}

public class Node
{

    public bool isfixed;

    public Vector3 pos;
    public Vector3 vel;
    public Vector3 force;
    public int id;

    float masa;
    int tetraedrosIn;

    public Node(Vector3 pos, bool isfixed, int id)
    {
        this.pos = pos;
        this.id = id;
        this.isfixed = isfixed;

        force = Vector3.zero;
        vel = Vector3.zero;
    }

    public void MasaNodos(float peso)
    {
        masa += peso;
        tetraedrosIn++;
    }

    public void MasaEntreTetra()
    {
        masa = masa / tetraedrosIn;
    }

    public void ComputeForces(Vector3 gravity, float damping)
    {
        force += masa * gravity;
        force += -damping * vel;
    }

}
public class Spring
{
    public Node nA;
    public Node nB;
    public float length, length0;

    float volumen;
    int tetraedrosIn;
    public Spring(Node nodeA, Node nodeB)
    {
        this.nA = nodeA;
        this.nB = nodeB;

        UpdateLength();
        length0 = length;

    }

    public void UpdateLength()
    {
        length = (nA.pos - nB.pos).magnitude;
    }
    public void MasaTetraedros(float peso)
    {
        volumen += peso;
        tetraedrosIn++;
    }

    public void MediaPesos()
    {
        volumen /= tetraedrosIn;
    }

    public void ComputeForces(float stiffness, float densidad, float damping)
    {
        Vector3 u = (nA.pos - nB.pos).normalized;
        Vector3 fuerza = -(volumen / (length0 * length0)) * densidad * (length - length0) * ((nA.pos - nB.pos) / length) * stiffness;


        fuerza += -damping * (nA.vel - nB.vel);
        fuerza += -damping * Vector3.Dot(u, (nA.vel - nB.vel)) * u;
        nA.force += fuerza;
        nB.force -= fuerza;
    }



}
