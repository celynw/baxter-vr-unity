using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

// Place this script into the Editor folder
public class EditorShortCutKeys : ScriptableObject {
	[MenuItem("Edit/Run _F5")] // Press F5 to Play/Exit playmode
	static void PlayGame() {
		if (!Application.isPlaying)
			EditorSceneManager.SaveScene(SceneManager.GetActiveScene(), "", false); // optional: save before run
		EditorApplication.ExecuteMenuItem("Edit/Play");
	}
}
