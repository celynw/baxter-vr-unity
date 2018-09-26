using UnityEngine;
using System.Collections;
using System.Collections.Generic; //For List
using System.Runtime.InteropServices; //For DLLImport

public class RosSubscribe : MonoBehaviour {
	[DllImport ("rosserial_unity")]
	private static extern void Connect();
	[DllImport ("rosserial_unity")]
	private static extern void Subscribe();
	[DllImport ("rosserial_unity")]
	private static extern void Spin();

	public GameObject Baxter; // Put "Baxter" GameObject here in editor

	void Start() {
		Connect();
		Subscribe();
	}

	void Update() {
		Spin();
	}

	void OnApplicationQuit() {

	}
}
