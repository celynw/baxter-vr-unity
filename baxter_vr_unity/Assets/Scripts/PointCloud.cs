using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic; // For std List and vector
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.ConstrainedExecution;
using System.Security;
using Microsoft.Win32.SafeHandles;

// using UnityEditor;

public class PointCloud : MonoBehaviour {
	//Shared Memory
	[DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Auto)]
	static extern SafeFileHandle OpenFileMapping(
		uint dwDesiredAccess,
		bool bInheritHandle,
		string lpName
	);

	[DllImport("kernel32.dll", SetLastError = true)]
	static extern IntPtr MapViewOfFile(
		SafeFileHandle hFileMappingObject,
		UInt32 dwDesiredAccess,
		UInt32 dwFileOffsetHigh,
		UInt32 dwFileOffsetLow,
		UIntPtr dwNumberOfBytesToMap
	);

	[DllImport("kernel32.dll", SetLastError = true)]
	[ReliabilityContract(Consistency.WillNotCorruptState, Cer.Success)]
	[SuppressUnmanagedCodeSecurity]
	[return: MarshalAs(UnmanagedType.Bool)]
	static extern bool CloseHandle(IntPtr hObject);

	const UInt32 STANDARD_RIGHTS_REQUIRED = 0x000F0000;
	const UInt32 SECTION_QUERY = 0x0001;
	const UInt32 SECTION_MAP_WRITE = 0x0002;
	const UInt32 SECTION_MAP_READ = 0x0004;
	const UInt32 SECTION_MAP_EXECUTE = 0x0008;
	const UInt32 SECTION_EXTEND_SIZE = 0x0010;
	const UInt32 SECTION_ALL_ACCESS = (
		STANDARD_RIGHTS_REQUIRED |
		SECTION_QUERY |
		SECTION_MAP_WRITE |
		SECTION_MAP_READ |
		SECTION_MAP_EXECUTE |
		SECTION_EXTEND_SIZE
	);
	const UInt32 FILE_MAP_ALL_ACCESS = SECTION_ALL_ACCESS;
	private SafeFileHandle sHandle;
	private IntPtr hHandle;
	private IntPtr pBuffer;
	bool attachSuccessful;

	// My stuff
	string nsmName = "ROScloud";
	private Mesh mesh;
	public Material material;
	int patches = 0;
	//int numPoints = 60000; // Max allowed

	// To be instantiated after init phase
	GameObject live;
	MeshFilter meshFilter;
	MeshRenderer meshRenderer;

	bool read = false;
	bool started = false;
	float startTime;
	bool patchesDone = false; //TODO changeme

	void OnEnable() {
		sHandle = new SafeFileHandle(hHandle, true);
		attachSuccessful = Attach(nsmName, (uint)65000); // Expecting up to 65536ish

		mesh = new Mesh();

		if (!attachSuccessful) {
			Debug.Log(nsmName + ": attach unsuccessful");
			AppHelper.Quit();
		}
	}

	void OnApplicationQuit() {
		if (attachSuccessful)
			Detach();
	}

	unsafe public bool Attach(string SharedMemoryName, UInt32 NumBytes) {
		if (!sHandle.IsInvalid) return false;
		sHandle = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, SharedMemoryName);
		if (sHandle.IsInvalid) return false;
		Debug.Log("Cloud: shared mem opened");
		pBuffer = MapViewOfFile(sHandle, FILE_MAP_ALL_ACCESS, 0, 0, new UIntPtr(NumBytes));
		Debug.Log("Cloud: shared mem mapped");
		return true;
	}

	unsafe public void Detach() {
		if (!sHandle.IsInvalid && !sHandle.IsClosed) {
			CloseHandle(hHandle);
			sHandle.Close();
		}
		pBuffer = IntPtr.Zero;
	}

	void Update() {
		read = !read;

		//get Shared memory Input
		if (attachSuccessful)// && read)
			read_memory();
	}

	// Reads the shared memory. Format described at the top of this file
	void read_memory() {
		int offset = 0;
		byte[] tempBytes = new byte[sizeof(float)];

		// Read data length (numPoints)
		for (int b = 0; b < sizeof(float); b++)
			tempBytes[b] = Marshal.ReadByte(pBuffer, offset + b);
		offset += sizeof(float);
		int numPoints = (int)BitConverter.ToSingle(tempBytes, 0);
		numPoints = numPoints / 2; //TODO WHY 2??

		Vector3[] points = new Vector3[numPoints];
		int[] indices = new int[numPoints];
		Color[] colors = new Color[numPoints];
		// Should never happen unless intentional. Not 0 for obvious reasons
		if ((numPoints != 0) && (!started)) {
			started = true;
			startTime = Time.time;
			Debug.Log("Started!!!");
		}
		if (started && !patchesDone && (Time.time > startTime + 5.0f)) {//((numPoints == 1) || (Time.time > startTime + 5.0f))) {
			patchesDone = true;
			mesh = new Mesh(); // This is the final live mesh
			float diff = Time.time - startTime;
			Debug.Log("Finished patches: time="+diff+", patches="+patches);
			live = new GameObject("LIVE");
			live.transform.parent = this.transform;
			live.transform.localPosition = Vector3.zero;
			live.transform.localEulerAngles = Vector3.zero;
			live.AddComponent<MeshFilter>();
			live.AddComponent<MeshRenderer>();
			return;
		} else {
			mesh.Clear();
		}
		// Assumes 4 floats per message: {X, Y, Z, C}
		for (int point=0; point < numPoints; point++) {
			// Position {X, Y, Z}
			for (int part=0; part < 3; part++) {
				for (int b=0; b < sizeof(float); b++)
					tempBytes[b] = Marshal.ReadByte(pBuffer, offset + b);
				offset += sizeof(float);
				float num = BitConverter.ToSingle(tempBytes, 0);
				if (!System.Single.IsNaN(num) && !System.Single.IsInfinity(num))
					points[point][part] = num;
				if (part == 2) // Invert this axis
					points[point][part] = -points[point][part];
			}
			offset += sizeof(float); // Spacing
			indices[point] = point;
			// Colour {C}
			//for (int b = 0; b < 3; b++)
				//colors[point][b] = ((float)Marshal.ReadByte(pBuffer, offset + b))/256.0f;
			colors[point][0] = ((float)Marshal.ReadByte(pBuffer, offset + 2)) / 256.0f;
			colors[point][1] = ((float)Marshal.ReadByte(pBuffer, offset + 1)) / 256.0f;
			colors[point][2] = ((float)Marshal.ReadByte(pBuffer, offset + 0)) / 256.0f;
			offset += sizeof(float)*4; // Spacing
		}
		mesh.vertices = points;
		mesh.colors = colors;
		mesh.SetIndices(indices, MeshTopology.Points, 0);

		// Get preliminary point cloud patches if early
		// MESSY
		if (started) {
			if (!patchesDone) {
				mesh = new Mesh();
				patches++;
				GameObject preliminary = new GameObject("preliminary" + patches);
				preliminary.transform.parent = this.transform;
				preliminary.transform.localPosition = Vector3.zero;
				preliminary.transform.localEulerAngles = Vector3.zero;
				meshFilter = preliminary.AddComponent<MeshFilter>();
				meshRenderer = preliminary.AddComponent<MeshRenderer>();
			} else {
				meshFilter = live.GetComponent<MeshFilter>();
				meshRenderer = live.GetComponent<MeshRenderer>();
			}
			meshFilter.mesh = mesh;
			meshRenderer.material = material;
		}
	}
}
