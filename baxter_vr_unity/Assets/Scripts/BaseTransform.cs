using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BaseTransform : MonoBehaviour {
	public Vector3 angles;
	private Vector3 hiddenAngles;

	void Start() {
		angles = new Vector3();
		hiddenAngles = new Vector3();
	}

	void Update() {
		hiddenAngles = angles * Mathf.Rad2Deg;
		gameObject.transform.localEulerAngles = Vector3.zero;
		gameObject.transform.Rotate(0, 0, -hiddenAngles.z, Space.Self);
		gameObject.transform.Rotate(0, -hiddenAngles.y, 0, Space.Self);
		gameObject.transform.Rotate(hiddenAngles.x, 0, 0, Space.Self);
	}
}
