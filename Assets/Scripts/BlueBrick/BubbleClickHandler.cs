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
using Windows.Devices.Bluetooth.GenericAttributeProfile;
#endif

public class BubbleClickHandler : MonoBehaviour {

	public ulong DeviceAddress = 0;
	public GameObject rootNode;

	private bool deviceHasChanged = false;
	private string newDeviceInfo = "";

	private DataHandler _dh;


#if WINDOWS_UWP
	private BluetoothLEDevice myDevice;
	private DeviceInformation myInformation;
	private bool isPairing = false;

	
#endif

	void Start () {

		rootNode = transform.parent.gameObject;
		_dh = new DataHandler();
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
#if WINDOWS_UWP
				// use the device object to get data from the characteristic
				SubscribeToIMUCharacteristic(myDevice);
#endif
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



	private Task SubscribeToIMUCharacteristic(BluetoothLEDevice device)
	{
		GattDeviceService service = _dh.TryGetService(device, new Guid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"));
		if (service == null) { return null; }

		GattCharacteristic charac = _dh.TryGetCharacteristic(service, new Guid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"));
		if (charac == null) { return null; }

		_dh.Notify_Characteristic(charac, GattClientCharacteristicConfigurationDescriptorValue.Notify);

		_dh.OnImuDataReceived += ReceiveImuData;
		Debug.Log("Data event linked to function");

		return null;
	}

	private void ReceiveImuData(byte[] _byte)
	{
		Debug.Log(_byte.Length + " bytes received");

		// It is left as an exercise for the student
		// to convert this byte array into a quaterion 
		// and link it to the local rotation property 
		// of the blue brick.
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
