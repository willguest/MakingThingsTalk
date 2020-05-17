using UnityEngine;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;
using System.Text;


#if WINDOWS_UWP
using Windows.Devices.Bluetooth;
using Windows.Devices.Enumeration;
#endif

public class BubbleClickHandler : MonoBehaviour {

	public ulong DeviceAddress = 0;
	public GameObject rootNode;

	private bool deviceHasChanged = false;
	private string newDeviceInfo = "";
	private bool isPairing = false;

#if WINDOWS_UWP
	private BluetoothLEDevice myDevice;
	private DeviceInformation myInformation;
#endif

	void Start () {

		rootNode = transform.parent.gameObject;
	}

	void Update()
	{
		// update device information UI
		if (deviceHasChanged)
		{

			if (newDeviceInfo != "")
			{
				GetComponentInChildren<TextMesh>().fontSize = 20;
				GetComponentInChildren<TextMesh>().text = newDeviceInfo;
				deviceHasChanged = false;
			}
			else
			{
				GetComponentInChildren<TextMesh>().text = gameObject.name;
			}
		}
	}

#if WINDOWS_UWP
	public void OnDoubleClick()
	{
		Task.Run(async () => { await ConnectToDevice(DeviceAddress); });
	}


	private async Task ConnectToDevice(ulong key)
	{
		myInformation = null;
		myDevice = await getBLEDeviceFromAddress(key);

		if (myDevice != null)
		{
			myInformation = await PairWithDevice(myDevice);
		}

		if (myInformation != null)
		{
			Debug.Log("Paired with: " + myInformation.Name.ToString());
			newDeviceInfo = PrintDeviceInformation(myInformation);
			deviceHasChanged = true;
		}
	}

	private async Task<DeviceInformation> PairWithDevice(BluetoothLEDevice _dev)
	{
		if (isPairing) return null;

		isPairing = true;
		DeviceInformation devInfo;
		DevicePairingResult result = await _dev.DeviceInformation.Pairing.PairAsync(DevicePairingProtectionLevel.None);

		if (result.Status == DevicePairingResultStatus.Paired || result.Status == DevicePairingResultStatus.AlreadyPaired)
		{
			devInfo = _dev.DeviceInformation;
		}
		else if (result.Status == DevicePairingResultStatus.AlreadyPaired)
		{
			devInfo = _dev.DeviceInformation;
		}
		else
		{
			devInfo = null;
		}

		isPairing = false;
		return devInfo;
	}


	private string PrintDeviceInformation(DeviceInformation devInfo)
	{
		StringBuilder sb = new StringBuilder();
		sb.AppendLine("Device information");

		foreach (KeyValuePair<string, object> kvp in devInfo.Properties)
		{
			sb.AppendLine(kvp.Key + ": " + kvp.Value);
		}

		return sb.ToString();
	}

	private async Task<BluetoothLEDevice> getBLEDeviceFromAddress(ulong btAddress)
	{
		return await BluetoothLEDevice.FromBluetoothAddressAsync(btAddress);
	}
#endif

}
