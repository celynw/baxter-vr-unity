using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.ConstrainedExecution;
using System.Security;
using Microsoft.Win32.SafeHandles;

public class UserInputKBM : MonoBehaviour {
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
	public bool isLeftHand;
	public bool enableOp;
	string nsmHandsName = "ROShands_"; // Can only use one hand!
	public float[] rosPosition; // What we will tell ROS for Pose
	byte[] tempBytes = new byte[sizeof(float)];
	float playerSpeed = 1.0f;
	float moveSpeed = 1.0f;
	float rotateSpeed = 80.0f;
	public GameObject prefabHand;
	public GameObject baseTransform; // To match mesh geometry
	GameObject hand;
	GameObject target;
	bool grabbing = false; // Used to toggle the grippers
	int commands = 0; // Count number of inputs!

	void OnEnable() {
		enableOp = isLeftHand;
		if (isLeftHand)
			nsmHandsName = nsmHandsName + 'l';
		else
			nsmHandsName = nsmHandsName + 'r';
		hand = (GameObject)Instantiate(prefabHand);
		hand.transform.SetParent(baseTransform.transform, true);
		if (isLeftHand) {
			hand.name = "Left Target Vis";
			hand.transform.localPosition = new Vector3(-0.7f, 0.2f, 0); // Place hand in front of Baxter
		} else {
			hand.name = "Right Target Vis";
			hand.transform.localPosition = new Vector3(-0.7f, -0.2f, 0); // Place hand in front of Baxter
		}
		target = new GameObject("ROStarget");
		target.transform.SetParent(baseTransform.transform, true);
		// Shared memory stuff
		sHandle = new SafeFileHandle(hHandle, true);
		attachSuccessful = Attach(nsmHandsName, (uint)50); // Expecting 32 (4*7 + 4)
		if (!attachSuccessful) {
			Debug.Log(nsmHandsName + ": attach unsuccessful");
			AppHelper.Quit();
		}
		else {
			// Status byte
			int offset = 0;
			Marshal.WriteByte(pBuffer, offset, (byte)0);
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
		Debug.Log(nsmHandsName + ": shared mem opened");
		pBuffer = MapViewOfFile(sHandle, FILE_MAP_ALL_ACCESS, 0, 0, new UIntPtr(NumBytes));
		Debug.Log(nsmHandsName + ": shared mem mapped");
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
		Cursor.visible = false;
		Cursor.lockState = CursorLockMode.Locked;

		// User changes hand
		if (Input.GetButtonDown("Change Target"))
			enableOp = !enableOp;

		transform.eulerAngles += new Vector3(-Input.GetAxis("Mouse Y"), Input.GetAxis("Mouse X"), 0);
		Vector3 playerMove = new Vector3(Input.GetAxis("Player Horizontal"), 0, Input.GetAxis("Player Forwards"));
		Vector3 targetMove = new Vector3(Input.GetAxis("Target Horizontal"), Input.GetAxis("Target Vertical"), Input.GetAxis("Target Forwards"));
		Vector3 targetRotate = new Vector3(Input.GetAxis("Target RX"), Input.GetAxis("Target RY"), 0);
		playerMove = transform.TransformDirection(playerMove);
		playerMove.y = 0;
		transform.position += Time.deltaTime * playerMove * playerSpeed;
		if (enableOp) {
			hand.transform.position += Time.deltaTime * targetMove * moveSpeed;
			hand.transform.Rotate(Time.deltaTime * targetRotate * rotateSpeed);
		}

		// User submits this position
		if (Input.GetButtonDown("Submit") && enableOp) {
			commands++;
			Debug.Log("submit: " + commands + ", " + Time.time);
			// Shared memory
			int offset = 0;
			rosPosition = new float[7];
			// Can't set as hand.transform as uses pointer access?
			Vector3 targetPosition;
			Quaternion targetRotation;
			targetPosition = hand.transform.localPosition;
			targetRotation = hand.transform.localRotation;
			target.transform.localPosition = targetPosition;
			target.transform.localRotation = targetRotation;
			target.transform.Rotate(-90, 0, 90, Space.Self);
			rosPosition[0] = -target.transform.localPosition.x;// - transform_offset.transform.position.x;
			rosPosition[1] = target.transform.localPosition.y;// + 0.08f;// - transform_offset.transform.position.y;
			rosPosition[2] = target.transform.localPosition.z;// - transform_offset.transform.position.z;
			rosPosition[3] = target.transform.localRotation.w;
			rosPosition[4] = -target.transform.localRotation.z;
			rosPosition[5] = target.transform.localRotation.y;
			rosPosition[6] = -target.transform.localRotation.x;
			// Write data, status byte written afterwards (but at beginning of data)
			offset += sizeof(float);
			// Data
			for (int i = 0; i < 7; i++) {
				tempBytes = BitConverter.GetBytes(rosPosition[i]);
				// Data
				for (int j = 0; j < tempBytes.Length; j++) {
					Marshal.WriteByte(pBuffer, offset, tempBytes[j]); // Argument order!
					offset++;
				}
			}
			Marshal.WriteByte(pBuffer, 0, (byte)1); // Status byte
		}

		if (Input.GetButtonDown("Grab") && enableOp) {
			commands++;
			Debug.Log("gripper: " + commands + ", " + Time.time);
			if (grabbing)
				Marshal.WriteByte(pBuffer, 8 * sizeof(float), (byte)'u'); // Argument order!
			else
				Marshal.WriteByte(pBuffer, 8 * sizeof(float), (byte)'g'); // Argument order!
			grabbing = !grabbing;
		}
	}
}
