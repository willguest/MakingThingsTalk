using System;
using System.Collections.Generic;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;

using UnityEngine;

#if WINDOWS_UWP
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Devices.Enumeration;
#endif

public class DataHandler
{

#if WINDOWS_UWP

    public DeviceInformation DeviceInformation;

    private BluetoothLEDevice Device;

    private List<GattDeviceService> myServices = new List<GattDeviceService>();
    private List<GattCharacteristic> myCharacs = new List<GattCharacteristic>();
    private GattCharacteristic activeCharac;

    private GattDeviceServicesResult gatt = null;
    private GattDeviceService targetService;
    private GattCharacteristic IMUdataChar;

    float[] currentIMUData;

    private const GattClientCharacteristicConfigurationDescriptorValue _notify =
        GattClientCharacteristicConfigurationDescriptorValue.Notify;

    private const GattClientCharacteristicConfigurationDescriptorValue _indicate =
        GattClientCharacteristicConfigurationDescriptorValue.Indicate;

    private const GattClientCharacteristicConfigurationDescriptorValue _unnotify =
        GattClientCharacteristicConfigurationDescriptorValue.None;

    public delegate void IMUdelegate(byte[] bytes);
    public IMUdelegate OnImuDataReceived;


    #region  --------- Get Service & Characteristic ---------- 

    public GattDeviceService TryGetService(BluetoothLEDevice my_Device, Guid my_Guid)
    {
        try
        {
            return Task.Run(async () => await GetService(my_Device, my_Guid)).Result;
        }
        catch { return null; }
    }
    public GattCharacteristic TryGetCharacteristic(GattDeviceService my_Service, Guid my_Guid)
    {
        try
        {
            return Task.Run(async () => await GetCharacteristic(my_Service, my_Guid)).Result;
        }
        catch { return null; }
    }

    private async Task<GattDeviceService> GetService(BluetoothLEDevice dev, Guid serviceGuid)
    {
        var tcs = new TaskCompletionSource<GattDeviceServicesResult>();
        if (!tcs.TrySetResult(await dev.GetGattServicesAsync(BluetoothCacheMode.Uncached)))
        {
            return null;
        }
        var waiter = tcs.Task.GetAwaiter();
        tcs.Task.Wait();
        while (!tcs.Task.GetAwaiter().IsCompleted) { }
        foreach (GattDeviceService gds in tcs.Task.Result.Services)
        {
            if (gds.Uuid == serviceGuid) { return gds; }
        }
        return null;
    }

    private async Task<GattCharacteristic> GetCharacteristic(GattDeviceService service, Guid characGuid)
    {
        var tcs = new TaskCompletionSource<GattCharacteristicsResult>();
        if (!tcs.TrySetResult(await service.GetCharacteristicsForUuidAsync(characGuid)))
        {
            return null;
        }
        tcs.Task.Wait();
        while (!tcs.Task.GetAwaiter().IsCompleted) { }
        foreach (GattCharacteristic gc in tcs.Task.Result.Characteristics)
        {
            if (gc.Uuid == characGuid) { return gc; }
        }
        return null;
    }

    #endregion --------- Get Service & Characteristic ---------- 


    #region ---------- Read, Write, Notify ----------

    public byte[] Read_Characteristic(GattCharacteristic charac)
    {
        byte[] readData = new byte[0];
        Task rRes = Read(charac).ContinueWith((anteRC) =>
        {
            if (anteRC.Status == TaskStatus.RanToCompletion)
            {
                var rr = anteRC.Result;
                readData = rr.Value.ToArray();
                return readData;
            }
            else
            {
                return null;
            }
        
        });
        return null;
    }

    public void Write_to_Characteristic(GattCharacteristic charac, byte[] packet)
    {
        Task wRes = Write(charac, packet).ContinueWith((anteWC) =>
        {
            return;
        });
    }

    public GattCommunicationStatus Notify_Characteristic(GattCharacteristic charac, GattClientCharacteristicConfigurationDescriptorValue value)
    {
        Task nRes = Notify(charac, value).ContinueWith((anteNC) =>
        {
            if (anteNC.Status == TaskStatus.RanToCompletion)
            {
                var CharNotStat = anteNC.Result;
                charac.ValueChanged += ReceiveData;
                return CharNotStat;
            }
            else
            {
                return GattCommunicationStatus.Unreachable;
            }
        });
        return GattCommunicationStatus.Unreachable;
    }

    public void ReceiveData(GattCharacteristic sender, GattValueChangedEventArgs args)
    {
        OnImuDataReceived?.Invoke(args.CharacteristicValue.ToArray());
    }




    private Task<GattReadResult> Read(GattCharacteristic characteristic)
    {
        var tcs = new TaskCompletionSource<GattReadResult>();
        Task.Run(async () =>
        {
            var _myres = await characteristic.ReadValueAsync(BluetoothCacheMode.Cached);
            tcs.SetResult(_myres);
        });
        return tcs.Task;
    }

    private Task<GattCommunicationStatus> Write(GattCharacteristic charac, byte[] msg)
    {
        var tcs = new TaskCompletionSource<GattCommunicationStatus>();
        Task.Run(async () =>
        {
            await charac.WriteValueAsync(msg.AsBuffer());
        });
        return tcs.Task;
    }

    private Task<GattCommunicationStatus> Notify(GattCharacteristic charac, GattClientCharacteristicConfigurationDescriptorValue value)
    {
        var tcs = new TaskCompletionSource<GattCommunicationStatus>();
        Task.Run(async () =>
        {
            var _myres = await charac.WriteClientCharacteristicConfigurationDescriptorAsync(value);
            tcs.SetResult(_myres);
        });
        tcs.Task.Wait();
        return tcs.Task;
    }

    #endregion ---------- Read, Write, Notify ----------

#endif

}
