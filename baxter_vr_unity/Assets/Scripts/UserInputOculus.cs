using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic; // For std List and vector
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.ConstrainedExecution;
using System.Security;
using Microsoft.Win32.SafeHandles;

public class UserInputOculus : MonoBehaviour {
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
	string nsmHandsName = "ROShands_"; // To be appended with 'l' or 'r'
	byte[] tempBytes = new byte[sizeof(float)];
	// private SteamVR_TrackedController controller;
	private Vector3 cylStartPos;
	private GameObject cyl; // Dynamic object, current cyl being drawn
	private GameObject hand = null; // Dynamic object, next hand position VIEW for user
	private GameObject target = null; // Dynamic object, next hand position POSE for ROS
	public GameObject prefabHand; // Fill with hand prefab
	public GameObject prefabCylinder; // Fill with cyl prefab
	public Material[] handMaterials; // To make hand translucent
	public GameObject baxterGripperBase; // Fill with Baxter's existing hands
	public GameObject baseTransform; // To match mesh geometry
	public float[] rosPosition; // What we will tell ROS for Pose
	int commands = 0; // Count number of inputs!


	private SteamVR_TrackedObject trackedObj;
	private SteamVR_Controller.Device Controller {
		get { return SteamVR_Controller.Input((int)trackedObj.index); }
	}
	bool triggered = false;

	void OnEnable() {
		// Register controller callbacks
		///////////////////////////////////////////////////////////////////////
		// TODO
		// https://docs.unity3d.com/Manual/OpenVRControllers.html
		// Move trigger to gripper toggle
		// Even better, not a toggle. Use value not click
		// Move face button to set target locations
		// Target: Axis1D.PrimaryIndexTrigger, Axis1D.SecondaryIndexTrigger (squeeze/touch)
		// 9, 10
		// Grip: Axis1D.PrimaryHandTrigger, Axis1D.SecondaryHandTrigger (squeeze/touch)
		// 11, 12
		///////////////////////////////////////////////////////////////////////
		// controller = GetComponent<SteamVR_TrackedController>();
		// controller.TriggerClicked += handle_trigger_clicked;
		// controller.TriggerUnclicked += handle_trigger_unclicked;
		// controller.Gripped += handle_gripped;
		// Shared memory stuff
		sHandle = new SafeFileHandle(hHandle, true);
		if (isLeftHand)
			nsmHandsName = nsmHandsName + 'l';
		else
			nsmHandsName = nsmHandsName + 'r';
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


	void OnDisable() {
		// Unegister controller callbacks
		/*
		controller.TriggerClicked -= handle_trigger_clicked;
		controller.TriggerUnclicked -= handle_trigger_unclicked;
		controller.Gripped -= handle_gripped;
		*/
	}

	// Grab or ungrab
	private void handle_gripped(bool close) {
		commands++;
		Debug.Log("gripper: " + commands + ", " + Time.time);
		if (close)
			Marshal.WriteByte(pBuffer, 8 * sizeof(float), (byte)'g'); // Argument order!
		else
			Marshal.WriteByte(pBuffer, 8 * sizeof(float), (byte)'u'); // Argument order!
	}

	// Start drawing the cylinder, draw hand
	private void handle_trigger_clicked() {
		// Start drawing cylinder
		if (hand == null) {
			cylStartPos = baxterGripperBase.transform.position;
			cyl = (GameObject)Instantiate(prefabCylinder);
			cyl.GetComponent<CylinderEndPositions>().start = cylStartPos;
			hand = (GameObject)Instantiate(prefabHand);
			hand.transform.SetParent(baseTransform.transform, true);
			target = new GameObject("ROStarget");
			target.transform.SetParent(baseTransform.transform, true);
			hand.transform.GetChild(0).GetComponent<Renderer>().materials = handMaterials;
		}
		// Hide controller, draw hand instead
		transform.GetChild(0).gameObject.SetActive(false);
	}

	// Finished drawing the cylinder
	private void handle_trigger_unclicked() {
		commands++;
		Debug.Log("submit: " + commands + ", " + Time.time);
		cyl.GetComponent<CylinderEndPositions>().end = transform.position;
		// Make controller reappear
		transform.GetChild(0).gameObject.SetActive(true);
		if (target == null) {
			Debug.Log("Tried to publish, but target is null!");
			return;
		}
		// Shared memory
		int offset = 0;
		rosPosition = new float[7];
		// Can't set as hand.transform as uses pointer access?
		Vector3 targetPosition = hand.transform.localPosition;
		Quaternion targetRotation = hand.transform.localRotation;
		target.transform.localPosition = targetPosition;
		target.transform.localRotation = targetRotation;
		// Account for endpoint
		/*
		target.transform.localPosition = new Vector3(
			target.transform.localPosition.x,
			target.transform.localPosition.y,
			target.transform.localPosition.z - 0.13f
		);
		*/
		target.transform.Rotate(-90, 0, 90, Space.Self);
		//tempTransform.Rotate(-tempTransform.localEulerAngles.x, 0, 0, Space.Self);
		//tempTransform.Rotate(0, tempTransform.localEulerAngles.y, 0, Space.Self);
		//tempTransform.Rotate(0, 0, tempTransform.localEulerAngles.z, Space.Self);

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
		for (int i=0; i < 7; i++) {
			tempBytes = BitConverter.GetBytes(rosPosition[i]);
			// Data
			for (int j = 0; j < tempBytes.Length; j++) {
				Marshal.WriteByte(pBuffer, offset, tempBytes[j]); // Argument order!
				offset++;
			}
		}
		Marshal.WriteByte(pBuffer, 0, (byte)1); // Status byte
		// Check for IK solver status
		offset++;
		char ik_stat = (char)Marshal.ReadByte(pBuffer, offset);
		if (ik_stat == 'f') {
			if (hand != null) {
				// IK failed, remove that target from the queue
				Destroy(hand);
				Destroy(target);
			} else
				Debug.Log("IK fail message but queue was empty!");
			// If we got something other than 'i' (IDLE), set it to this
			Marshal.WriteByte(pBuffer, offset, (byte)'i');
		}
		else if (ik_stat == 'c') {
			if (hand != null) {
				// IK finished, remove that target from the queue
				Destroy(hand);
				Destroy(target);
			} else
				Debug.Log("IK complete message but queue was empty!");
			// If we got something other than 'i' (IDLE), set it to this
			Marshal.WriteByte(pBuffer, offset, (byte)'i');
		}
		else {
			return;
		}
	}

	// Handle cyl logic
	private void update_cylinder(GameObject cyl, Vector3 start, Vector3 end) {
		var direction = end - start;
		var midpoint = direction / 2;
		var magnitude = direction.magnitude;

		cyl.transform.rotation = Quaternion.FromToRotation(Vector3.up, direction);
		cyl.transform.localScale = new Vector3(0.05f, direction.magnitude / 2, 0.05f);
		cyl.transform.position = midpoint + start;
	}

	void Awake() {
		trackedObj = GetComponent<SteamVR_TrackedObject>();
	}

	void Update() {
		
		if (hand != null) {
			cylStartPos = baxterGripperBase.transform.position;
			if (triggered) {
				update_cylinder(cyl, cylStartPos, transform.position);
				hand.transform.position = transform.position;
				hand.transform.rotation = transform.rotation;
			} else
				update_cylinder(cyl, baxterGripperBase.transform.position, cyl.GetComponent<CylinderEndPositions>().end);
		}



		if (Controller.GetHairTriggerDown()) {
			handle_trigger_clicked();
			triggered = true;
		}
		if (Controller.GetHairTriggerUp()) {
			handle_trigger_unclicked();
			triggered = false;
		}
		if (Controller.GetPressDown(SteamVR_Controller.ButtonMask.Grip))
			handle_gripped(true);
		if (Controller.GetPressUp(SteamVR_Controller.ButtonMask.Grip))
			handle_gripped(false);
	}
}
