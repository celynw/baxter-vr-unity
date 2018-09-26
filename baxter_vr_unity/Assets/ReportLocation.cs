using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ReportLocation : MonoBehaviour {
	public GameObject prefabHand; // Fill with hand prefab
	public Vector3 position;
	public Vector3 rotation;
	public Quaternion quaternion;

	GameObject hand;

	// Use this for initialization
	void Start () {
		//hand = (GameObject)Instantiate(prefabHand);
	}
	
	// Update is called once per frame
	void LateUpdate () {
		Vector3 angles = Vector3.zero;

		// Inverse of URDF script (?)
		//hand.transform.Rotate(-angles.x, 0, 0, Space.Self);
		//hand.transform.Rotate(0, angles.y, 0, Space.Self);
		//hand.transform.Rotate(0, 0, angles.z, Space.Self);

		//hand.transform.position = transform.position;
		//hand.transform.rotation = transform.rotation;

		position = transform.position;
		rotation = transform.eulerAngles;
		quaternion = transform.rotation;
	}
}
