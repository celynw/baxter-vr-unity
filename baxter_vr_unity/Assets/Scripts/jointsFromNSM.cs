using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic; // For std List and vector
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.ConstrainedExecution;
using System.Security;
using Microsoft.Win32.SafeHandles;

/* Memory layout
 * 
 * Offset	Description			Notes
 *  0		 Number of joints	 Updated as more are added
 *
 *  1		 Joint position		 4 bytes long (from float)
 *  5		 Joint name length	 Number of characters
 *  6		 Joint name			 As long as joint name length
 *
 *  after	 Joint position		 Repeat above block
 *			 Joint name length	 etc.
 *			 Joint name			 etc.
 */

public class JointsFromNSM : MonoBehaviour {
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
	string nsmJointsName = "ROSjoints";
	public List<GameObject> joints;
	public List<int> offsets; //TODO make private
	int offsetEnd = 1; // Start of empty memory. [0] is number of joints prefix

	void OnEnable() {
		// NSM stuff
		sHandle = new SafeFileHandle(hHandle, true);
		attachSuccessful = Attach(nsmJointsName, (uint)1000); // Expecting ~250
															  // Will be filled as they appear in shared memory
		if (!attachSuccessful) {
			Debug.Log(nsmJointsName + ": attach unsuccessful");
			AppHelper.Quit();
		}

		joints = new List<GameObject>();
	}

	void OnApplicationQuit() {
		if (attachSuccessful)
			Detach();
	}

	unsafe public bool Attach(string SharedMemoryName, UInt32 NumBytes) {
		if (!sHandle.IsInvalid) return false;
			sHandle = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, SharedMemoryName);
		if (sHandle.IsInvalid) return false;
			Debug.Log("Joints: shared mem opened");
		pBuffer = MapViewOfFile(sHandle, FILE_MAP_ALL_ACCESS, 0, 0, new UIntPtr(NumBytes));
		Debug.Log("Joints: shared mem mapped");
		return true;
	}

	unsafe public void Detach() {
		if (!sHandle.IsInvalid && !sHandle.IsClosed) {
			CloseHandle(hHandle);
			sHandle.Close();
		}
		pBuffer = IntPtr.Zero;
	}

	void LateUpdate() {
		//get Shared memory Input
		if (attachSuccessful)
			read_memory();
	}

	// Reads the shared memory. Format described at the top of this file
	void read_memory() {
		// Add new joints to list if not present
		// offsetEnd is kept at end of memory in use
		int missing = (Marshal.ReadByte(pBuffer, 0) - joints.Count);
		for (int i=0; i < missing; i++) {
			int offsetData = offsetEnd; // Store for now
			offsetEnd += 4; // Skip over value
			int nameLen = Marshal.ReadByte(pBuffer, offsetEnd);
			offsetEnd++;
			char[] chars = new char[nameLen];
			for (int j=0; j<nameLen; j++) {
				chars[j] = (char)Marshal.ReadByte(pBuffer, offsetEnd);
				offsetEnd++;
			}
			string name = new string(chars);
			// The "URDF joint"s are the ones which should accept rotation
			// Sometimes link has the same name, so need to use tags
			GameObject obj = null;
			GameObject[] objs = GameObject.FindGameObjectsWithTag("URDF joint");
			foreach (GameObject candidate in objs) {
				if (candidate.name == name)
					obj = candidate;
			}
			if (obj == null) {
				Debug.LogError("Cannot find "+name+"!");
			} else {
				joints.Add(obj);
				offsets.Add(offsetData);
			}
		}
		// Reload the values (at known offsets)
		for (int i=0; i<joints.Count; i++) {
			byte[] tempBytes = new byte[sizeof(float)];
			for (int j=0; j < sizeof(float); j++)
				tempBytes[j] = Marshal.ReadByte(pBuffer, offsets[i] + j);
			update_value(i, BitConverter.ToSingle(tempBytes, 0));
		}
	}

	// Fills GameObject parameter with obtained value
	void update_value(int select, float value) {
		// Prepare joints
		if (joints[select].GetComponent<urdfAttributes>() == null) {
			Debug.LogError("urdfAttributes script not found on URDF link: " + joints[select].name);
		}
		// Reset rotations
		Vector3 angles = joints[select].GetComponent<urdfAttributes>().Rotation;
		joints[select].transform.localEulerAngles = Vector3.zero;
		joints[select].transform.Rotate(0, 0, -angles.z, Space.Self);
		joints[select].transform.Rotate(0, -angles.y, 0, Space.Self);
		joints[select].transform.Rotate(angles.x, 0, 0, Space.Self);
		// Do what subscribed topic is doing
		joints[select].transform.Rotate(0, 0, -value * Mathf.Rad2Deg, Space.Self);
	}
}
