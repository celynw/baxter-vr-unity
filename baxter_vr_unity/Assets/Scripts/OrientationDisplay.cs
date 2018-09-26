using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI; // For Text

public class OrientationDisplay : MonoBehaviour {
	public int orientation = 3; // [1]st or [3]rd person, start facing Baxter
	public Text orientationText; // Fill with orientation text
	public GameObject[] orientationImages;

	void Start() {
		for (int i = 0; i < orientationImages.Length; i++)
			orientationImages[i].SetActive(false);
		orientationImages[0].SetActive(true);  // Should be front, 3rd person
		orientationText.text = "3rd person";
		orientation = 3;
	}

	void Update() {
	}

	public void swap() {
		bool fPs = (orientation == 1);
		orientationText.text = fPs ? "1st person" : "3rd person";
		orientationImages[1].SetActive(fPs); // Should be back, 1st person
		orientationImages[0].SetActive(!fPs); // Should be front, 3rd person
		orientation = fPs ? 3 : 1;
	}
}
