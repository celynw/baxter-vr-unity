using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic; // For lists
using System.IO; // For Path
using System.Xml; // For urdf

public class urdfPostProcessor : AssetPostprocessor
{
	private static void OnPostprocessAllAssets(string[] importedAssets, string[] deletedAssets, string[] movedAssets, string[] movedFromPath) {
		List<string> done = new List<string>();
		// Check for .urdf.xacro files first
		foreach (string str in importedAssets) {
			string ext = Path.GetExtension(str);
			if (ext == ".xacro") {
				ext = Path.GetExtension(Path.GetFileNameWithoutExtension(str));
				if (ext == ".urdf") {
					parseURDF(str);
					done.Add(Path.GetFileNameWithoutExtension(str));
					Debug.Log("Importing from <color=lime>"+Path.GetFileName(str)+"</color>");
				}
			}
		}
		// Check for .urdf files, don't duplicate xacro ones
		foreach (string str in importedAssets)
			if (!done.Contains(str)) {
				string ext = Path.GetExtension(str);
				if (ext == ".urdf")
					parseURDF(str);
			}
	}

	// Parse the XML in the URDF file
	private static void parseURDF(string filename) {
		// Load file
		var xmlData = new XmlDocument();
		xmlData.Load(filename);
		// Set root node
		XmlNode rootNode = xmlData.DocumentElement;
		string name = rootNode.Attributes["name"].Value;
		GameObject rootObj = new GameObject(name);
		rootObj.transform.Rotate(-90, 0, 0, Space.Self);
		rootObj.tag = "URDF link";
		rootObj.AddComponent<JointsFromNSM>();
		// Start recursive alternating functions to reach bottom of hierarchy
		XmlNodeList joints = rootNode.SelectNodes("joint/parent[@link='base']");
		foreach (XmlNode jointNode in joints)
			generateJoint(jointNode, rootObj, rootNode);
	}
	private static void generateJoint(XmlNode node, GameObject parentObj, XmlNode rootNode) {
		// Create joint
		node = node.ParentNode;
		string name = node.Attributes["name"].Value;
		Debug.Log("joint: "+name);
		GameObject obj = new GameObject(name);
		obj.transform.parent = parentObj.transform;
		obj.transform.localPosition = Vector3.zero;
		obj.transform.localEulerAngles = Vector3.zero;
		obj.tag = "URDF joint";
		obj.AddComponent<urdfAttributes>();
		// Get origin rotation and translation parameters
		getOrigin(node.SelectSingleNode("origin"), obj);
		// Get axis of rotation
		getAxis(node.SelectSingleNode("axis"), obj);
		// Find the child link
		XmlNode link = rootNode.SelectSingleNode("link[@name='"+node.SelectSingleNode("child").Attributes["link"].Value+"']");
		generateLink(link, obj, rootNode);
	}
	private static void generateLink(XmlNode node, GameObject parentObj, XmlNode rootNode) {
		// Drag in the meshes (Create link)
		string name = node.Attributes["name"].Value;
		Debug.Log("link: "+name);
		GameObject obj = new GameObject(name); // Will be replaced by model instance if possible
		XmlNode meshNode = node.SelectSingleNode("visual/geometry/mesh");
		if (meshNode != null) {
			Object.DestroyImmediate(obj); // We've found a mesh, replace obj with that
			string path = meshNode.Attributes["filename"].Value;
			System.Uri uri = new System.Uri(path);
			path = uri.Host + uri.PathAndQuery; // Removes "package://"
			obj = GameObject.Instantiate(AssetDatabase.LoadAssetAtPath("Assets/ROS/"+path, typeof(GameObject))) as GameObject;
			// The child has what we want, so swap it out
			if (obj.GetComponent(typeof(MeshFilter)) == null) {
				var comp = obj.GetComponentInChildren(typeof(MeshFilter)) as MeshFilter;
				GameObject newobj = comp.gameObject;
				newobj.transform.parent = parentObj.transform;
 				Object.DestroyImmediate(obj);
				obj = newobj;
			} else {
				Component.DestroyImmediate(obj.GetComponent(typeof(Animator)));
			}
			obj.name = name;
		}
		// Create link
		obj.transform.parent = parentObj.transform;
		obj.transform.localPosition = Vector3.zero;
		obj.transform.localEulerAngles = Vector3.zero;
		obj.tag = "URDF link";
		obj.AddComponent<urdfAttributes>();
		// Get origin rotation and translation parameters
		XmlNode originNode = node.SelectSingleNode("visual/origin");
		if (originNode != null) //Not all links have a visual node
			getOrigin(originNode, obj);
		// Find all joints who have this link as a parent
		XmlNodeList joints = rootNode.SelectNodes("joint/parent[@link='"+name+"']");
		foreach (XmlNode jointNode in joints)
			generateJoint(jointNode, obj, rootNode);
	}

	private static void getOrigin(XmlNode node, GameObject obj) {
		// Parse for translation
		string[] xyzStr = node.Attributes["xyz"].Value.Split(' ');
		Vector3 locations = new Vector3();
		locations.x = float.Parse(xyzStr[0]);
		locations.y = float.Parse(xyzStr[1]);
		locations.z = float.Parse(xyzStr[2]);
		obj.GetComponent<urdfAttributes>().Position = locations;
		// Parse for rotation
		string[] rpyStr = node.Attributes["rpy"].Value.Split(' ');
		Vector3 angles = new Vector3();
		angles.x = float.Parse(rpyStr[0]); // r
		angles.y = float.Parse(rpyStr[1]); // p
		angles.z = float.Parse(rpyStr[2]); // y
		angles = angles * Mathf.Rad2Deg;
		obj.GetComponent<urdfAttributes>().Rotation = angles; //TODO this isn't negated!
		// Apply Transform
		obj.transform.Translate(-locations.x, locations.y, locations.z);
		obj.transform.Rotate(0, 0, -angles.z, Space.Self);
		obj.transform.Rotate(0, -angles.y, 0, Space.Self);
		obj.transform.Rotate(angles.x, 0, 0, Space.Self);
	}

	private static void getAxis(XmlNode node, GameObject obj) {
		if (node != null && node.Attributes["xyz"] != null) {
			string[] xyzStr = node.Attributes["xyz"].Value.Split(' ');
			Vector3 axis = new Vector3();
			axis.x = int.Parse(xyzStr[0]);
			axis.y = int.Parse(xyzStr[1]);
			axis.z = int.Parse(xyzStr[2]);
			obj.GetComponent<urdfAttributes>().Axis = axis;
		}
	}
}
