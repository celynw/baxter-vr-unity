using UnityEngine;
using System.Collections;

public class urdfAttributes : MonoBehaviour {

	[SerializeField]
	private Vector3 axis;
	public Vector3 Axis {
		get { return axis; }
		set { axis = value; }
	}

	[SerializeField]
	private Vector3 position;
	public Vector3 Position {
		get { return position; }
		set { position = value; }
	}

	[SerializeField]
	private Vector3 rotation;
	public Vector3 Rotation {
		get { return rotation; }
		set { rotation = value; }
	}
}
