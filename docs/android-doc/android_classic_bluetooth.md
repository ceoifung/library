---
sidebar_position: 4
title: Android经典蓝牙
---

android经典蓝牙的控制方式与ble的控制方式不一致，下面是一个封装好的经典蓝牙控制类
## ClassicBluetoothManager
```java
public class ClassicBluetoothManager {
    private final static String TAG = "ClassicBluetoothManager";
    private static BluetoothAdapter bluetoothAdapter;
    private static final String UUID_NOTIFY = "0000ffe1-0000-1000-8000-00805f9b34fb";

    //    公司蓝牙
//    00000000-0000-1000-8000-00805f9b34fb
    public static final String XRGEEK_UUID_SERVICE = "00001101-0000-1000-8000-00805f9b34fb";
    private static final String XRGEEK_UUID_WRITE = "00001101-0000-1000-8000-00805f9b34fb";

    private static ScanCallBack scanCallBack;
    private static BluetoothSocket mBluetoothSocket;
    private static boolean isRead;
    private static String bleSuffix = "SkyTarp";
    private static boolean foundDevice;

    public static boolean isIsRead() {
        return isRead;
    }

    public static void setIsRead(boolean isRead) {
        ClassicBluetoothManager.isRead = isRead;
    }

    public static String getBleSuffix() {
        return bleSuffix;
    }

    public static void setBleSuffix(String bleSuffix) {
        ClassicBluetoothManager.bleSuffix = bleSuffix;
    }

    /**
     * 注册设备
     *
     * @param context 上下文
     */
    public static void register(Context context) {
        if (bluetoothAdapter == null) {
            IntentFilter intentFilter = new IntentFilter();
            intentFilter.addAction(BluetoothAdapter.ACTION_STATE_CHANGED);
            intentFilter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
            intentFilter.addAction(BluetoothDevice.ACTION_FOUND);
            intentFilter.addAction(BluetoothDevice.ACTION_PAIRING_REQUEST);
            intentFilter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
            intentFilter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
            context.registerReceiver(mFindBlueToothReceiver, intentFilter);
            BluetoothManager bleManager = (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);
            bluetoothAdapter = bleManager.getAdapter();
        }
    }

    /**
     * 取消注册监听
     *
     * @param context 上下文
     */
    public static void unregister(Context context) {
        context.unregisterReceiver(mFindBlueToothReceiver);
        disConnect();
    }

    /**
     * 开始扫描蓝牙设备
     * @param context 上下文
     * @param callBack 回调
     */
    public static void startScan(Context context, ScanCallBack callBack) {
        if (bluetoothAdapter != null) {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                Log.e(TAG, "startScan: start scanning...");
                PermissionUtils.requestPermission(context, Manifest.permission.BLUETOOTH_SCAN, 1001);
                if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                    return;
                }
                bluetoothAdapter.startDiscovery();
            } else {
                Log.e(TAG, "startScan: 开始扫描");
                bluetoothAdapter.startDiscovery();
            }
            scanCallBack = callBack;
        } else {
            Log.e(TAG, "startScan: bluetoothAdapter is null");
            EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, "该设备不支持蓝牙链接"));
        }
    }

    /**
     * 判断设备是否连接
     * @param context 上下恩
     * @return true or false
     */
    public static boolean isConnected(Context context) {
        if (getConnectedBtDevice(context) != null) {
            if (mBluetoothSocket != null)
                return mBluetoothSocket.isConnected();
            else return false;
        } else
            return false;
    }

    /**
     * 获取连接的蓝牙设备名称
     * @param context 上下文
     * @return true or false
     */
    private static String getConnectedBtDevice(Context context) {
        //获取蓝牙适配器
        //得到已匹配的蓝牙设备列表
        if (Build.VERSION.SDK_INT > Build.VERSION_CODES.S) {
            if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                return null;
            }
        }
        Set<BluetoothDevice> bondedDevices = bluetoothAdapter.getBondedDevices();
        if (bondedDevices != null && bondedDevices.size() > 0) {
            for (BluetoothDevice bondedDevice : bondedDevices) {
                try {
                    //使用反射调用被隐藏的方法
                    Method isConnectedMethod =
                            BluetoothDevice.class.getDeclaredMethod(
                                    "isConnected"
                            );
                    isConnectedMethod.setAccessible(true);
                    boolean isConnected = (boolean) isConnectedMethod.invoke(bondedDevice);
                    if (isConnected) {
                        return bondedDevice.getName();
                    }
                } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
                    e.printStackTrace();
                }
            }
        }
        return null;
    }

    /**
     * 蓝牙开关是否打开了
     *
     * @return 是否打开了蓝牙开关
     */
    public static boolean isEnable() {
        if (bluetoothAdapter != null)
            return bluetoothAdapter.isEnabled();
        return false;
    }

    /**
     * 关闭蓝牙开关
     *
     * @param context     上下文
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
            disConnect();
        }
    }

    /**
     * 打开蓝牙开关
     *
     * @param context     上下文
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

    @SuppressLint("MissingPermission")
    public static void cancelScan() {
        if (bluetoothAdapter != null) {
//            PermissionUtils.requestPermission(context, Manifest.permission.BLUETOOTH_SCAN, 1001);
            bluetoothAdapter.cancelDiscovery();
        }
    }

    /**
     * 配对蓝牙设备
     */
    public static void pinTargetDevice(Context context, BluetoothDevice device) {
        //在配对之前，停止搜索
        cancelScan();
        //获取要匹配的BluetoothDevice对象，后边的deviceList是你本地存的所有对象
//        BluetoothDevice device = deviceList.get(position);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {

                return;
            }
        }

        if (device.getBondState() != BluetoothDevice.BOND_BONDED) {//没配对才配对
            try {
                Log.e(TAG, "开始配对...");

                Method createBondMethod = BluetoothDevice.class.getMethod("createBond");
                Boolean returnValue = (Boolean) createBondMethod.invoke(device);

                if (returnValue != null) {
                    if (returnValue) {
                        Log.e(TAG, "配对成功...");
                        connectDevice(context, device);
                    }
                } else {
                    Log.e(TAG, "pinTargetDevice: 配对失败");
                    EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, "蓝牙配对失败"));
                }
            } catch (NoSuchMethodException | IllegalAccessException | InvocationTargetException e) {
                e.printStackTrace();
            }
        } else {
//            Log.e(TAG, "pinTargetDevice: 已经配对过" );
            connectDevice(context, device);
        }
    }

    public static void connectDevice(Context context, BluetoothDevice device) {
//        Log.e(TAG, "connectDevice: connect device...");
        connect(context, device, new ConnectBlueCallBack() {
            @Override
            public void onStartConnect() {
                Log.e(TAG, "onStartConnect: ");
                EventBus.getDefault().post(new ConnBean(ConnEnum.CONNECTING, "正在连接蓝牙"));
            }

            @Override
            public void onConnectSuccess(BluetoothDevice device, BluetoothSocket bluetoothSocket) {
                Log.e(TAG, "onConnectSuccess: ");
                mBluetoothSocket = bluetoothSocket;
                EventBus.getDefault().post(new ConnBean(ConnEnum.CONNECTED, "蓝牙已连接"));
                new Thread(ClassicBluetoothManager::loopRead).start();
            }

            @Override
            public void onConnectFail(BluetoothDevice device, String string) {
                Log.e(TAG, "onConnectFail: ");
                EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, "蓝牙连接失败"));
            }
        });
    }

    /**
     * 循环读取对方数据(若没有数据，则阻塞等待)
     */
    private static void loopRead() {
        try {
            if (mBluetoothSocket != null) {
                InputStream inputStream = mBluetoothSocket.getInputStream();//获取服务端发来的消息
                BufferedInputStream bufferedInputStream = new BufferedInputStream(inputStream);
                int count = 6;
                int readCount = 0;
                byte[] data = new byte[count];
                while (true) {
                    int available = inputStream.available();
                    if (available > 0) {
                        while (readCount < count) {
                            readCount += bufferedInputStream.read(data, readCount, count - readCount);
                        }
                        readCount = 0;
                        Log.e(TAG, "loopRead: " + HexUtils.bytes2HexStr(data));
                        EventBus.getDefault().post(new BleReceiveEvent(data));
                    }
                }
            }

        } catch (Throwable e) {
            disConnect();
        }
    }

    public static void sendData(String cmd) {
        sendData(HexUtils.hexStringToByteArray(cmd));
    }

    public static void sendData(byte[] cmd) {
        if (mBluetoothSocket != null) {
            try {
                mBluetoothSocket.getOutputStream().write(cmd);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }


    /**
     * 断开连接
     *
     * @return
     */
    public static boolean disConnect() {
        if (mBluetoothSocket != null && mBluetoothSocket.isConnected()) {
            try {
                isRead = false;
                mBluetoothSocket.close();
            } catch (IOException e) {
                e.printStackTrace();
                return false;
            }
        }
        mBluetoothSocket = null;
        return true;
    }

    /**
     * 连接 （在配对之后调用）
     *
     * @param device
     */
    private static void connect(Context context, BluetoothDevice device, ConnectBlueCallBack callBack) {
        //连接之前把扫描关闭
        cancelScan();
        new ConnectBlueTask(callBack).execute(device);
    }

    //广播接收器，当远程蓝牙设备被发现时，回调函数onReceiver()会被执行
    private static final BroadcastReceiver mFindBlueToothReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
//            Log.e(TAG, "onReceive: " + action);
            BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
            switch (action) {
                case BluetoothAdapter.ACTION_DISCOVERY_STARTED:
                    Log.e(TAG, "开始扫描...");
                    if (scanCallBack != null)
                        scanCallBack.onScanStarted();
                    break;
                case BluetoothAdapter.ACTION_DISCOVERY_FINISHED:
                    Log.e(TAG, "结束扫描...");
                    if (mBluetoothSocket == null && !foundDevice)
                        EventBus.getDefault().post(new ConnBean(ConnEnum.NOT_FOUND, "没有找到设备"));
                    if (scanCallBack != null)
                        scanCallBack.onScanFinished();
                    break;
                case BluetoothDevice.ACTION_FOUND:
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                        PermissionUtils.requestPermission(context, Manifest.permission.BLUETOOTH_CONNECT, 1001);
                        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                            return;
                        }
                    }
                    if (device.getName() != null) {
                        Log.e(TAG, "发现设备..." + device.getName());
                        if (scanCallBack != null) {
                            scanCallBack.onScanning(device);
                        }
                        if (device.getName().equals(bleSuffix)) {
                            foundDevice = true;
                            pinTargetDevice(context, device);
                        }
                    }

                    break;
                case BluetoothDevice.ACTION_BOND_STATE_CHANGED:

                    switch (device.getBondState()) {
                        case BluetoothDevice.BOND_BONDING:
                            Log.e(TAG, "正在配对......");
                            if (scanCallBack != null)
                                scanCallBack.pairStart();
                            break;
                        case BluetoothDevice.BOND_BONDED:
//                            connectDevice(context, device);
                            if (scanCallBack != null)
                                scanCallBack.pairFinish();
                            break;
                        case BluetoothDevice.BOND_NONE:
                            Log.e(TAG, "取消配对");
                            if (scanCallBack != null)
                                scanCallBack.pairCancel();
                        default:
                            break;
                    }
                    break;
                case BluetoothAdapter.ACTION_STATE_CHANGED:
                    int bleState = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, 0);
                    EventBus.getDefault().post(new BleEvent(bleState));
                    break;
                case BluetoothDevice.ACTION_ACL_CONNECTED:
                    Log.e(TAG, "onReceive: 设备已连接");
                    EventBus.getDefault().post(new ConnBean(ConnEnum.CONNECTED, "蓝牙已连接"));
                    break;
                case BluetoothDevice.ACTION_ACL_DISCONNECTED:
                    mBluetoothSocket = null;
                    EventBus.getDefault().post(new ConnBean(ConnEnum.REMOVE, "设备已断开"));
                    break;
            }
        }
    };
}
```

## 使用方式
### 初始化代码
```kotlin
override fun onStart() {
    super.onStart()
    ClassicBluetoothManager.register(this)
    if (ClassicBluetoothManager.isConnected(this)) {
    } else if (ClassicBluetoothManager.isEnable()) {}
}

// 注销监听
override fun onDestroy() {
    super.onDestroy()
    ClassicBluetoothManager.unregister(this)
}
```

### 开始扫描蓝牙设备
```kotlin
// 可以注册监听回调
ClassicBluetoothManager.startScan(this@MainActivity, null)
```

### 发送数据
```kotlin
// 由于没有内容可以显示
ClassicBluetoothManager.sendData(data)
```

### EventBus的使用
该代码用到了，请参考代码的具体使用，在MainActivity中注册相应的监听回调，以实现自己的需求

