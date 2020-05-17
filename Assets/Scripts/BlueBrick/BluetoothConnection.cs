using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Linq;

using UnityEngine;
using UnityEngine.UI;


#if WINDOWS_UWP
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
#endif


public class BluetoothConnection : MonoBehaviour
{
	public Dictionary<ulong, string> addressBook = new Dictionary<ulong, string>();
	public Text deviceListText;

	#region Private Variables

	// state change variables
	private bool addressBookHasChanged = false;
	private bool isFaulted = false;
	private string errReported = "";

	


#if WINDOWS_UWP
	private BluetoothLEAdvertisementWatcher BleWatcher;

#endif

	#endregion Private Variables


	// Start is called before the first frame update
	void Start()
	{
		SetupAdvertWatcher();
	}


	// Update is called once per frame
	void Update()
	{
		// address book updates
		if (addressBookHasChanged)
		{
			deviceListText.text = PrintAddressBook();

			GetComponentInChildren<ShowTheWorld>().MenuKeys = addressBook.Keys.ToArray();
			GetComponentInChildren<ShowTheWorld>().MenuNames = addressBook.Values.ToArray();
			addressBookHasChanged = false;
		}

		// error reporting
		if (isFaulted)
		{
			deviceListText.text = errReported;
		}
	}

	private string PrintAddressBook()
	{
		int deviceCounter = 1;
		int deviceCount = addressBook.Count;
		string devicesText = "Address Book Entries: " + deviceCount + "\n ~~~~~~~~~~~~~~~~ \n";
		foreach (KeyValuePair<ulong, string> kvp in addressBook)
		{
			devicesText += deviceCounter + ": " + kvp.Value + "\n[" + kvp.Key.ToString() + "]\n";
			deviceCounter++;
		}
		return devicesText;
	}


	private void SetupAdvertWatcher()
	{
#if WINDOWS_UWP
		// BLE Advert Watcher
		BleWatcher = new BluetoothLEAdvertisementWatcher();
		BleWatcher.ScanningMode = BluetoothLEScanningMode.Active;
		BleWatcher.SignalStrengthFilter.InRangeThresholdInDBm = -65;
		BleWatcher.SignalStrengthFilter.OutOfRangeTimeout = TimeSpan.FromSeconds(1);

		BleWatcher.Received += WatcherOnReceived;
		BleWatcher.Stopped += Watcher_Stopped;

		BleWatcher.Start();
#endif
	}


#if WINDOWS_UWP
	private void HandleWatcher(BluetoothLEAdvertisementReceivedEventArgs watcherArgs)
	{
		string _name = watcherArgs.Advertisement.LocalName;
		ulong btAddr = watcherArgs.BluetoothAddress;

		if (_name.Length == 0)
		{
			_name = btAddr.ToString();
		}
		

		KeyValuePair<ulong, string> watcherResult = new KeyValuePair<ulong, string>(btAddr, _name);
		AddOrUpdateDevice(watcherResult);


		// check for connectability
		try
		{
			BluetoothLEDevice attemptDev = null;
			Task.Run(async () =>
			{
				attemptDev = await getBLEDeviceFromAddress(btAddr);

			}).ContinueWith((antecedant) =>
			{
				if (attemptDev != null)
				{
					KeyValuePair<ulong, string> attemptConnResult = new KeyValuePair<ulong, string>(btAddr, attemptDev.Name);
					AddOrUpdateDevice(attemptConnResult);
				}
			}).ConfigureAwait(false);
		}
		catch
		{
			Debug.Log("device connection attempt failed...");
		}


	}

	private void AddOrUpdateDevice(KeyValuePair<ulong,string> kvp)
	{
		if (addressBook.ContainsKey(kvp.Key) && addressBook[kvp.Key] != kvp.Value)
		{
			ulong.TryParse(kvp.Value, out ulong nP);

			if (nP != kvp.Key)
			{
				addressBook[kvp.Key] = kvp.Value;
				addressBookHasChanged = true;
				Debug.Log("Updated address book: " + kvp.Key.ToString() + " is " + kvp.Value);
			}
			
		}
		else if (!addressBook.ContainsKey(kvp.Key))
		{
			addressBook.Add(kvp.Key, kvp.Value);
			addressBookHasChanged = true;

			//Debug.Log("Added new address: " + _name);
		}
	}


	private void WatcherOnReceived(BluetoothLEAdvertisementWatcher sender, BluetoothLEAdvertisementReceivedEventArgs args)
	{
		ulong _addr = args.BluetoothAddress;
		HandleWatcher(args);

	}

	private void Watcher_Stopped(BluetoothLEAdvertisementWatcher watcher, BluetoothLEAdvertisementWatcherStoppedEventArgs eventArgs)
	{
		errReported = "Watcher Stopped. \nPlease check that bluetooth is switched on";
		isFaulted = true;
	}

	private async Task<BluetoothLEDevice> getBLEDeviceFromAddress(ulong btAddress)
	{
		return await BluetoothLEDevice.FromBluetoothAddressAsync(btAddress);
	}

#endif
}
