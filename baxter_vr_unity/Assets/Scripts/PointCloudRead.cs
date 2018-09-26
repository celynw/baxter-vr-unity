using System.Collections;
using UnityEngine;
using System.IO;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class PointCloudRead : MonoBehaviour {
	private static FileStream fsSource = new FileStream("cloud.txt", FileMode.Open, FileAccess.Read);

	private Mesh mesh;
	int numPoints = 60000;

	void Start() {
		mesh = new Mesh();

		GetComponent<MeshFilter>().mesh = mesh;
		CreateMesh();
	}

	void CreateMesh() {
		//fs.Seek(fileOffset, SeekOrigin.Begin);
		int numBytesToRead = 1000; // Amount to read at a time
		byte[] buffer = new byte[numBytesToRead];
		int numBytesRead = 0;
		while (numBytesToRead > 0) {
			int n = fsSource.Read(buffer, numBytesRead, numBytesToRead);
			if (n == 0)
				break;
		}

		Vector3[] points = new Vector3[numPoints];
		int[] indicies = new int[numPoints];
		Color[] colors = new Color[numPoints];
		for(int i = 0; i<points.Length; i++) {
			points[i] = new Vector3(Random.Range(-10, 10), Random.Range (-10, 10), Random.Range (-10, 10));
			indicies[i] = i;
			colors[i] = new Color(Random.Range(0.0f, 1.0f),Random.Range (0.0f, 1.0f),Random.Range(0.0f, 1.0f), 1.0f);
		}

		mesh.vertices = points;
		mesh.colors = colors;
		mesh.SetIndices(indicies, MeshTopology.Points,0);
	}
}
