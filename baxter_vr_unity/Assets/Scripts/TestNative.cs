using UnityEngine;
using System.Collections;
using System.Runtime.InteropServices; //For DLLImport

public class TestNative : MonoBehaviour {
	[DllImport ("rosserial_unity")]
	private static extern int GetRandom();

	void Start () {
		print("Native random number: "+GetRandom());
	}
}
