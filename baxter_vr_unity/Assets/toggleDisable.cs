using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class toggleDisable : MonoBehaviour {
	public MeshRenderer meshRenderer;
	
	void Update () {
		if (Input.GetButtonDown("Toggle Disable"))
			meshRenderer.enabled = !meshRenderer.enabled;
	}
}
