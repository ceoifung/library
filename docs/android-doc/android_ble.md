---
sidebar_position: 2
title: Android Ble封装类
---

以下工具类只针对android BLE连接，由我进行了二次的封装

## AndroidManifest.xml申请权限
```xml
<uses-permission android:name="android.permission.BLUETOOTH" />
<uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />
<uses-permission android:name="android.permission.BLUETOOTH_SCAN" />
<!--位置权限-->
<!--Android 10以上系统，需要ACCESS_FINE_LOCATION-->
<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION"/>
<!--Android 9以及以下系统，需要ACCESS_FINE_LOCATION-->
<uses-permission android:name="android.permission.ACCESS_COARSE_LOCATION"/>
```

## build.gradle添加fastble依赖
```gradle
implementation 'com.github.Jasonchenlijian:FastBle:2.4.0'
```

## 蓝牙广播接收器类
```java
public class BtReceiver extends BroadcastReceiver {
    private String TAG = "BtReceiverTool";
    @Override
    public void onReceive(Context context, Intent intent) {
        String action = intent.getAction();
        if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
            int bleState = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, 0);
            Log.e(TAG, "onReceive: ACTION_STATE_CHANGED" );
            EventBus.getDefault().post(new BleEvent(bleState));
        }
    }
}

```
## BleToolManager
> BLE连接以及信息接收处理工具类

```java
public class BleToolManager {
    private final static String TAG = "BleToolManager";
    private static BtReceiver btReceiverTool;
    private static BluetoothAdapter bluetoothAdapter;
    private static final String UUID_NOTIFY = "0000ffe1-0000-1000-8000-00805f9b34fb";

    //    公司蓝牙
    private static final String XRGEEK_UUID_SERVICE = "0000ffe0-0000-1000-8000-00805f9b34fb";
    private static final String XRGEEK_UUID_WRITE = "0000ffe1-0000-1000-8000-00805f9b34fb";

    private static int bleRssi = -90;
    private static BleDevice myBleDevice;
    private static boolean isBleConnected;
    private static String bleNameSuffix = "XiaoRGEEK";

    /**
     * 注册设备
     * @param context 上下文
     */
    public static void register(Context context) {
        if (btReceiverTool == null) {
            isBleConnected = false;
            btReceiverTool = new BtReceiver();
            IntentFilter intentFilter = new IntentFilter();
            intentFilter.addAction(BluetoothAdapter.ACTION_STATE_CHANGED);
            context.registerReceiver(btReceiverTool, intentFilter);
            BluetoothManager bleManager = (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);
            bluetoothAdapter = bleManager.getAdapter();
            BleManager.getInstance().init(((Activity) context).getApplication());
            setScanBleRules();
        }
    }

    public static int getBleRssi() {
        return bleRssi;
    }

    /**
     * 设置蓝牙信号的轻度
     * @param bleRssi 设置蓝牙信号的轻度
     */
    public static void setBleRssi(int bleRssi) {
        BleToolManager.bleRssi = bleRssi;
    }

    public static String getBleNameSuffix() {
        return bleNameSuffix;
    }

    public static void setBleNameSuffix(String bleNameSuffix) {
        BleToolManager.bleNameSuffix = bleNameSuffix;
    }

    /**
     * 设置扫描规则
     */
    private static void setScanBleRules() {
        String[] uuids;
        String str_uuid = XRGEEK_UUID_SERVICE;
        if (TextUtils.isEmpty(str_uuid)) {
            uuids = null;
        } else {
            uuids = str_uuid.split(",");
        }
        UUID[] serviceUuids = null;
        if (uuids != null && uuids.length > 0) {
            serviceUuids = new UUID[uuids.length];
            for (int i = 0; i < uuids.length; i++) {
                String name = uuids[i];
                String[] components = name.split("-");
                if (components.length != 5) {
                    serviceUuids[i] = null;
                } else {
                    serviceUuids[i] = UUID.fromString(uuids[i]);
                    Log.e(TAG, "setScanRule: " + UUID.fromString(uuids[i]));
                }
            }
        }

        String[] names;
//        String str_name = "XiaoRGEEK";
        if (TextUtils.isEmpty(bleNameSuffix)) {
            names = null;
        } else {
            names = bleNameSuffix.split(",");
        }
        BleScanRuleConfig scanRuleConfig = new BleScanRuleConfig.Builder()
                .setServiceUuids(serviceUuids)      // 只扫描指定的服务的设备，可选
                .setDeviceName(true, names)   // 只扫描指定广播名的设备，可选
//                .setDeviceMac(mac)                  // 只扫描指定mac的设备，可选
//                .setAutoConnect(isAutoConnect)      // 连接时的autoConnect参数，可选，默认false
                .setScanTimeOut(5000)              // 扫描超时时间，可选，默认10秒
                .build();
        BleManager.getInstance().initScanRule(scanRuleConfig);
    }

    /**
     * 开始扫描设备，并自动连接
     */
    public static void startScan() {
        BleManager.getInstance().scan(new BleScanCallback() {
            @Override
            public void onScanStarted(boolean success) {
                Log.e(TAG, "onScanStarted: 开始扫描" + success);
            }

            @Override
            public void onLeScan(BleDevice bleDevice) {
//                Log.e(TAG, "onLeScan: 进来扫描"+bleDevice.getName() );
                super.onLeScan(bleDevice);
            }

            @Override
            public void onScanning(BleDevice bleDevice) {
                Log.e(TAG, "onScanning: " + bleDevice.getName());
//                Log.e(TAG, "onScanning: " + bleRssi);
                if (!TextUtils.isEmpty(bleDevice.getName())) {
//                    ToastUtils.showBottomText(getString(R.string.ble_signal) + bleDevice.getRssi());
                    if (bleDevice.getName().contains(bleNameSuffix)) {
                        if (bleDevice.getRssi() >= bleRssi) {
                            connect(bleDevice);
                        } else {
                            Log.e(TAG, "onScanning: 没有找到设备" );
                        }
                    }
                }
            }

            @Override
            public void onScanFinished(List<BleDevice> scanResultList) {
                Log.e(TAG, "onScanFinished: 扫描完成");
                if (!isBleConnected) EventBus.getDefault().post(new ConnBean(ConnEnum.NOT_FOUND, "没有找到蓝牙设备"));
            }
        });
    }

    /**
     * 连接蓝牙设备
     * @param bleDevice 连接低功耗蓝牙设备
     */
    private static void connect(final BleDevice bleDevice) {
        Log.e(TAG, "connect: " + bleDevice.getName());
        BleManager.getInstance().connect(bleDevice, new BleGattCallback() {
            @Override
            public void onStartConnect() {
                Log.e(TAG, "onStartConnect: 开始连接蓝牙");
            }

            @Override
            public void onConnectFail(BleDevice bleDevice, BleException exception) {
                Log.e(TAG, "onConnectFail: 连接失败");
                isBleConnected = false;
                EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, "连接失败"));
            }

            @Override
            public void onConnectSuccess(BleDevice bleDevice, BluetoothGatt gatt, int status) {
                myBleDevice = bleDevice;
//                XRConfig.isConnected = true;

                isBleConnected = true;
                BleManager.getInstance().notify(bleDevice, XRGEEK_UUID_SERVICE, UUID_NOTIFY,
                        new BleNotifyCallback() {
                            @Override
                            public void onNotifySuccess() {
                                Log.e(TAG, "onNotifySuccess: 通知成功");
                                EventBus.getDefault().post(new ConnBean(ConnEnum.CONNECTED, "蓝牙已连接"));
                            }

                            @Override
                            public void onNotifyFailure(BleException exception) {
                                Log.e(TAG, "onNotifyFailure: 通知失败");
                                EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, "没有找到设备"));
                            }

                            @Override
                            public void onCharacteristicChanged(byte[] data) {
                                if (data.length > 0)
                                    EventBus.getDefault().post(new BleReceiveEvent(data));
                            }
                        });

                BleManager.getInstance().cancelScan();
            }

            @Override
            public void onDisConnected(boolean isActiveDisConnected, BleDevice bleDevice, BluetoothGatt gatt, int status) {
                if (isActiveDisConnected) {
                    Log.e(TAG, "onDisConnected: 蓝牙断开");
                } else {
                    Log.e(TAG, "onDisConnected: 连接断开");
                }
//                XRConfig.isConnected = false;
                isBleConnected = false;
                EventBus.getDefault().post(new ConnBean(ConnEnum.REMOVE, "连接已断开"));
                BleManager.getInstance().disconnectAllDevice();
                BleManager.getInstance().destroy();

            }
        });
    }

    /**
     * 发送数据
     * @param command 16进制字符串
     */
    public static void sendData(String command) {
        if (myBleDevice != null) {
            sendData(HexUtils.hexStringToByteArray(command));
        }
    }

    /**
     * 真实的数据发送函数
     *
     * @param cmd byte数组
     */
    private static void sendData(byte[] cmd) {
        BleManager.getInstance().write(myBleDevice, XRGEEK_UUID_SERVICE, XRGEEK_UUID_WRITE,
                cmd, new BleWriteCallback() {
                    @Override
                    public void onWriteSuccess(int current, int total, byte[] justWrite) {
                        Log.e(TAG, "onWriteSuccess: 写入数据成功");
                    }

                    @Override
                    public void onWriteFailure(BleException exception) {
                        Log.e(TAG, "onWriteFailure: " + exception.getDescription());
                    }
                });
    }

    /**
     * 取消注册监听
     * @param context 上下文
     */
    public static void unregister(Context context) {
        if (btReceiverTool != null) {
            context.unregisterReceiver(btReceiverTool);
        }
    }

    /**
     * 蓝牙设备是否连接
     * @return 连接状态
     */
    public static boolean isConnected() {
        return isBleConnected;
    }

    /**
     * 蓝牙开关是否打开了
     * @return 是否打开了蓝牙开关
     */
    public static boolean isEnable() {
        if (bluetoothAdapter != null)
            return bluetoothAdapter.isEnabled();
        return false;
    }

    /**
     * 打开蓝牙开关
     * @param context 上下文
     * @param requestCode 请求码
     */
    public static void openBle(Context context, int requestCode) {
        if (bluetoothAdapter != null) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                PermissionUtils.checkAndRequestPermission(
                        context,
                        Manifest.permission.BLUETOOTH_CONNECT, requestCode
                );
            }
            bluetoothAdapter.enable();
        }
    }

    /**
     * 关闭蓝牙开关
     * @param context 上下文
     * @param requestCode 请求码
     */
    public static void disableBle(Context context, int requestCode) {
        if (bluetoothAdapter != null) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                PermissionUtils.checkAndRequestPermission(
                        context,
                        Manifest.permission.BLUETOOTH_CONNECT, requestCode
                );
            }
            bluetoothAdapter.disable();
        }
    }
}

```

## Activity中使用
- 监听蓝牙开启状态
```kotlin
/**
     * 监听蓝牙是否使能成功
     */
    @Subscribe(threadMode = ThreadMode.MAIN)
    fun onBleEvent(event: BleEvent) {
        when (event.message) {
            BluetoothAdapter.STATE_TURNING_ON -> binding.bleView.setBleLevel(BleLevel.LOADING)
            BluetoothAdapter.STATE_ON -> {
                binding.bleView.setBleLevel(BleLevel.CONNECTING)
                BleToolManager.startScan()
            }
            BluetoothAdapter.STATE_TURNING_OFF -> {
//                binding.btnBleSettings.setDescription(getString(R.string.ble_turning_off))
                binding.bleView.setBleLevel(BleLevel.LOADING)
            }
            BluetoothAdapter.STATE_OFF -> {
                binding.bleView.setBleLevel(BleLevel.DISCONNECTED)
            }
            BluetoothAdapter.STATE_CONNECTING -> Log.e(TAG, "正在连接蓝牙")
            BluetoothAdapter.ERROR -> Log.e(TAG, "onReceive: BluetoothAdapter.ERROR")
            BluetoothAdapter.STATE_CONNECTED -> Log.e(TAG, "onBleEvent: 蓝牙已连接")
            BluetoothAdapter.STATE_DISCONNECTED -> {
//                binding.btnBleSettings.setDescription(getString(R.string.ble_disconnected))
//                binding.btnBleSettings.isSelected = false
                binding.bleView.setBleLevel(BleLevel.DISCONNECTED)
            }
        }
    }
```
- 监听蓝牙数据回传
```kotlin
@Subscribe(threadMode = ThreadMode.MAIN)
    fun onBtReceiveEvent(event: BleReceiveEvent) {
        //decodeData(HexUtils.bytes2HexStr(event.message))
    }
```
- onStart
```kotlin
override fun onStart() {
        super.onStart()
        BleToolManager.register(this)
    }
```

- onDestory
```kotlin
override fun onDestroy() {
        super.onDestroy()
        BleToolManager.unregister(this)
    }
```