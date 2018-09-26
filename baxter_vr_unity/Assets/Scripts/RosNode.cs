using System; // Exceptions
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics; // Process
using UnityEngine;
//using UnityEditor;

// Enable quitting nicely
public static class AppHelper {
	#if UNITY_WEBPLAYER
			public static string webplayerQuitURL = "http://google.com";
	#endif
	public static void Quit() {
		#if UNITY_EDITOR
			UnityEditor.EditorApplication.isPlaying = false;
		#elif UNITY_WEBPLAYER
			Application.OpenURL(webplayerQuitURL);
		#else
			Application.Quit();
		#endif
	}
}

public class RosNode : MonoBehaviour {
	Process process = new Process();
	public bool stayInEditor;
	public bool CONNECT;
	public bool enableJoints;
	public JointsFromNSM jointsScript;
	public bool enableCloud;
	public PointCloud pointCloudScript;
	public bool enableLeftHand;
	public UserInput leftHandScript;
	public bool enableRightHand;
	public UserInput rightHandScript;
	public bool enableLeftHandOculus;
	public UserInputOculus leftHandScriptOculus;
	public bool enableRightHandOculus;
	public UserInputOculus rightHandScriptOculus;

	void Start() {
		jointsScript.enabled = false;
		pointCloudScript.enabled = false;
		leftHandScript.enabled = false;
		rightHandScript.enabled = false;
		leftHandScriptOculus.enabled = false;
		rightHandScriptOculus.enabled = false;

		//if (stayInEditor)
		//UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));

		if (CONNECT) {
			try {
				// process.StartInfo.FileName = Application.dataPath + "/Executables/ROSnode.exe";
				// process.StartInfo.FileName = Environment.CurrentDirectory + "/Assets/Executables/ROSnode.exe";
				process.StartInfo.FileName = Environment.CurrentDirectory + "/ROSnode.exe";
				//process.StartInfo.RedirectStandardOutput = true;
				//process.StartInfo.RedirectStandardError = true;
				//process.StartInfo.CreateNoWindow = true;
				process.StartInfo.UseShellExecute = false; // Required for IO redirect
				process.EnableRaisingEvents = false;
				//process.OutputDataReceived += new DataReceivedEventHandler(coutReceived);
				//process.ErrorDataReceived += new DataReceivedEventHandler(cerrReceived);
				//process.WaitForInputIdle();
				//process.StartInfo.WindowStyle = ProcessWindowStyle.Minimized;
				process.Start();
				//process.BeginOutputReadLine();
				System.Threading.Thread.Sleep(500);
			} catch (Exception e) {
				// Ambiguous namespace with System.Diagnostics
				UnityEngine.Debug.LogError(e);
			}
		}
		// Now we can enable the scripts which rely on the process
		if (CONNECT) {
			jointsScript.enabled = enableJoints;
			pointCloudScript.enabled = enableCloud;
			leftHandScript.enabled = enableLeftHand;
			rightHandScript.enabled = enableRightHand;
			leftHandScriptOculus.enabled = enableLeftHandOculus;
			rightHandScriptOculus.enabled = enableRightHandOculus;
		}
	}

	void coutReceived(object sender, DataReceivedEventArgs e) {
		if (!String.IsNullOrEmpty(e.Data))
			UnityEngine.Debug.Log("<color=blue>node: " + e.Data + "</color>");
	}

	void cerrReceived(object sender, DataReceivedEventArgs e)
	{
		if (!String.IsNullOrEmpty(e.Data))
			UnityEngine.Debug.Log("<color=orange>node: " + e.Data + "</color>");
	}

	/*
	private void LateUpdate() {
		if (!process.HasExited) {
			UnityEngine.Debug.Log("<color=cyan>" + process.StandardOutput.ReadLine() + "</color>");
			UnityEngine.Debug.Log("<color=orange>" + process.StandardError.ReadLine() + "</color>");
		}
	}
	*/

	void OnApplicationQuit() {
		// Would be better to exit properly...
		process.Kill();
	}
}
