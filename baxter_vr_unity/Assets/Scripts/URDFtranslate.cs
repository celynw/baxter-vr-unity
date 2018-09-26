using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class URDFtranslate : MonoBehaviour {
	public Vector3 locations;
	public Vector3 angles;

	void Start() {
		transform.Translate(-locations.x, locations.y, locations.z);
		transform.Rotate(0, 0, -angles.z, Space.Self);
		transform.Rotate(0, -angles.y, 0, Space.Self);
		transform.Rotate(angles.x, 0, 0, Space.Self);
	}

	void Update() {
	}
}
