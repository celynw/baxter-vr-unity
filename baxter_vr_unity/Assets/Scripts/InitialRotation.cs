using UnityEngine;
using System.Collections;

public class InitialRotation : MonoBehaviour {
	public Vector3 input = new Vector3();
	public Vector3 angles = new Vector3(0.0f, 0.0f, 0.0f);
	public Vector3 upd_input = new Vector3(0.0f, 0.0f, 0.0f);
	public Vector3 upd_angles = new Vector3(0.0f, 0.0f, 0.0f);
	private Vector3 saved;

	// Use this for initialization
	void Start () {
		angles.x = input.x * Mathf.Rad2Deg;
		angles.y = input.y * Mathf.Rad2Deg;
		angles.z = input.z * Mathf.Rad2Deg;

		transform.localEulerAngles = Vector3.zero;
		transform.Rotate(0, 0, -angles.z, Space.Self);
		transform.Rotate(0, -angles.y, 0, Space.Self);
		transform.Rotate(angles.x, 0, 0, Space.Self);
		saved = transform.localEulerAngles;
	}

	// Update is called once per frame
	void Update () {
		upd_angles.x = upd_input.x * Mathf.Rad2Deg;
		upd_angles.y = upd_input.y * Mathf.Rad2Deg;
		upd_angles.z = upd_input.z * Mathf.Rad2Deg;
		transform.localEulerAngles = saved;
		transform.Rotate(0, 0, -upd_angles.z, Space.Self);
		transform.Rotate(0, -upd_angles.y, 0, Space.Self);
		transform.Rotate(upd_angles.x, 0, 0, Space.Self);
	}
}
