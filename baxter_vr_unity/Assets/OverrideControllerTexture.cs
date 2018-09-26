// https://gist.github.com/TrebuhD/d85e288adefd75899b7d8fb285952cb8
// Edited by me though

using UnityEngine;
using System.Collections;

/// <summary>
/// Override the texture of each of the parts with your texture.
/// </summary>
public class OverrideControllerTexture : MonoBehaviour
{
	#region Public variables
	[Header("Variables")]
	public Texture2D albedoTexture;
	public Texture2D normalTexture;
	public Texture2D metallicTexture;
	public Texture2D specularTexture;
	#endregion

	void OnEnable()
	{
		//Subscribe to the event that is called by SteamVR_RenderModel, when the controller mesh + texture has been loaded completely.
		SteamVR_Events.RenderModelLoaded.Listen(OnControllerLoaded);
	}

	private void OnDisable()
	{
		//Unsubscribe from the event if this object is disabled.
		SteamVR_Events.RenderModelLoaded.Remove(OnControllerLoaded);
	}

	public void UpdateControllerTexture(Texture2D newTexture, Transform modelTransform)
	{
		// store all controller materials
		var materials = new ArrayList
		{
			modelTransform.Find("body").GetComponent<MeshRenderer>().material,
			modelTransform.Find("button").GetComponent<MeshRenderer>().material,
			modelTransform.Find("led").GetComponent<MeshRenderer>().material,
			modelTransform.Find("lgrip").GetComponent<MeshRenderer>().material,
			modelTransform.Find("rgrip").GetComponent<MeshRenderer>().material,
			modelTransform.Find("scroll_wheel").GetComponent<MeshRenderer>().material,
			modelTransform.Find("sys_button").GetComponent<MeshRenderer>().material,
			modelTransform.Find("trackpad").GetComponent<MeshRenderer>().material,
			modelTransform.Find("trackpad_scroll_cut").GetComponent<MeshRenderer>().material,
			modelTransform.Find("trackpad_touch").GetComponent<MeshRenderer>().material,
			modelTransform.Find("trigger").GetComponent<MeshRenderer>().material
		};

		// give the materials textures
		foreach (Material m in materials)
		{
			//convert to valve shader
			m.shader = Shader.Find("Standard");

			m.SetInt("_SpecularMode", 1);
			m.DisableKeyword("S_SPECULAR_NONE");
			m.DisableKeyword("S_SPECULAR_METALLIC");
			m.EnableKeyword("S_SPECULAR_BLINNPHONG");

			// set main, normal and metallic textures
			m.mainTexture = newTexture;
			m.SetTexture("_BumpMap", normalTexture);
			m.EnableKeyword("_NORMALMAP");
			m.SetTexture("_SpecGlossMap", specularTexture);
			m.EnableKeyword("_SPECGLOSSMAP");

			m.SetFloat("g_flReflectanceMax", 0.5f); // max reflectance

			// convert to metallic
			//            m.SetInt("_SpecularMode", 2);
			//            m.DisableKeyword("S_SPECULAR_NONE");
			//            m.DisableKeyword("S_SPECULAR_BLINNPHONG");
			//            m.EnableKeyword("S_SPECULAR_METALLIC");
			//            m.SetTexture("_MetallicGlossMap", metallicTexture);
			//            m.EnableKeyword("_METALLICGLOSSMAP");
		}

		// dim the less important elements (optional)
		((Material)materials[0]).SetFloat("g_flCubeMapScalar", 0.1f); // body
		((Material)materials[6]).SetFloat("g_flCubeMapScalar", 0.1f); // system btn
		((Material)materials[7]).SetFloat("g_flCubeMapScalar", 0.1f); // trackpad
		((Material)materials[8]).SetFloat("g_flCubeMapScalar", 0.1f); // trackpad
	}

	/// <summary>
	/// Call this method when the RenderModelLoaded event is triggered.
	/// </summary>
	/// <param name="args">SteamVR_RenderModel renderModel, bool success</param>
	void OnControllerLoaded(SteamVR_RenderModel renderModel, bool success)
	{
		if (!success) return;
		UpdateControllerTexture(albedoTexture, renderModel.gameObject.transform);
	}
}