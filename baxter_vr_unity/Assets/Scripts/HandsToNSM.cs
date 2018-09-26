using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic; // For std List and vector
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.ConstrainedExecution;
using System.Security;
using Microsoft.Win32.SafeHandles;

public class HandsToNSM : MonoBehaviour {
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
	public bool leftHand;

	// My stuff
	string nsmHandsName = "ROShands";
	private int childCount = 0; // Track changes
	byte[] tempBytes = new byte[sizeof(float)];

	void Start() {
		sHandle = new SafeFileHandle(hHandle, true);
		attachSuccessful = Attach(nsmHandsName, (uint)50); // Expecting 29
		if (!attachSuccessful)
			Debug.Log("Hands: attach unsuccessful");
	}

	void OnApplicationQuit() {
		if (attachSuccessful)
			Detach();
	}

	unsafe public bool Attach(string SharedMemoryName, UInt32 NumBytes)	{
		if (!sHandle.IsInvalid) return false;
		sHandle = OpenFileMapping(FILE_MAP_ALL_ACCESS, false, SharedMemoryName);
		if (sHandle.IsInvalid) return false;
		Debug.Log("Hands: shared mem opened");
		pBuffer = MapViewOfFile(sHandle, FILE_MAP_ALL_ACCESS, 0, 0, new UIntPtr(NumBytes));
		Debug.Log("Hands: shared mem mapped");
		return true;
	}

	unsafe public void Detach()	{
		if (!sHandle.IsInvalid && !sHandle.IsClosed) {
			CloseHandle(hHandle);
			sHandle.Close();
		}
		pBuffer = IntPtr.Zero;
	}

	void LateUpdate() {
		// Something changed
		if ((transform.childCount != 0) &&
			(transform.childCount != childCount)) {

			childCount = transform.childCount;
			GameObject nextTarget = transform.GetChild(0).gameObject;
			float[] queue = new float[7];
			queue[0] = nextTarget.transform.position.x;
			queue[1] = nextTarget.transform.position.y;
			queue[2] = nextTarget.transform.position.z;
			queue[3] = nextTarget.transform.rotation.w;
			queue[4] = nextTarget.transform.rotation.x;
			queue[5] = nextTarget.transform.rotation.y;
			queue[6] = nextTarget.transform.rotation.z;

			// Write
			int offset = 0;
			if (!leftHand)
				offset += (queue.Length * sizeof(float)) + sizeof(float);
			// Data
			for (int i=0; i < queue.Length; i++) {
				tempBytes = BitConverter.GetBytes(queue[i]);
				//Debug.Log("bytes: " + tempBytes[0] + " " + tempBytes[1] + " " + tempBytes[2] + " " + tempBytes[3]);
				//Debug.Log("float: " + queue[i]);
				if (attachSuccessful) {
					for (int j=0; j < tempBytes.Length; j++) {
						// Stupid argument order...
						Marshal.WriteByte(pBuffer, offset, tempBytes[j]);
						Debug.Log("W: " + tempBytes[j] + " @ " + offset);
						offset++; //offset++; // WHY TWO??
					}
				}
			}
			// Status byte
			offset = 0;
			if (!leftHand)
				offset += (queue.Length * sizeof(float)) + sizeof(float);
			Marshal.WriteByte(pBuffer, offset, (byte)1);
		}
	}
}
