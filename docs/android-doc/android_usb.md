---
sidebar_position: 3
title: Android USB串口使用
---

在一些环境中，我们常常要是用Android的OTG接口，进行usb串口通信，以下是我封装的usb串口工具类，经过验证，没有很大的问题

## 依赖安装
### build.gradle
```gradle
# 在app的build.gradle添加如下依赖
implementation 'com.github.mik3y:usb-serial-for-android:3.4.6'
implementation 'org.greenrobot:eventbus:3.2.0'
```
### AndroidManifest
```xml
<uses-feature android:name="android.hardware.usb.host" />
<uses-feature android:name="android.hardware.usb.accessory" />
```
## 代码设计
### 工具类
```java

public class UsbSerialManager {

    private static UsbSerialPort usbSerialPort;
    public static final String GRANTER_USB_DEVICE = "com.xiaor.skytarp.usb.permission";
    private static int deviceId, portNum;
    private static SerialInputOutputManager usbIoManager;

    private enum UsbPermission {Unknown, Requested, Granted, Denied}

    private static UsbPermission usbPermission = UsbPermission.Unknown;
    private static UsbReceiver usbReceiver;

    /**
     * 注册监听程序
     * @param context 上下文
     */
    public static void register(Context context) {
        if (usbReceiver == null) {
            usbReceiver = new UsbReceiver();
            IntentFilter intentFilter = new IntentFilter();
            intentFilter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
            intentFilter.addAction(UsbManager.ACTION_USB_ACCESSORY_ATTACHED);
            intentFilter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
            intentFilter.addAction(GRANTER_USB_DEVICE);
            context.registerReceiver(usbReceiver, intentFilter);
        }
        if (usbSerialPort == null) listUsb(context);
    }

    /**
     * 移除监听
     *
     * @param context 上下文
     */
    public static void unregister(Context context) {
        close();
        if (usbReceiver != null)
            context.unregisterReceiver(usbReceiver);
    }

    /**
     * 请求USB权限
     *
     * @param context    上下文
     * @param usbManager usbManager
     * @param usbDevice  usb设备
     */
    private static void grantedUsbPermission(Context context, UsbManager usbManager, UsbDevice usbDevice) {
        int flags = Build.VERSION.SDK_INT >= Build.VERSION_CODES.M ? PendingIntent.FLAG_IMMUTABLE : 0;
        PendingIntent pendingIntent =
                PendingIntent.getBroadcast(context, 0, new Intent(GRANTER_USB_DEVICE), flags);
        usbManager.requestPermission(usbDevice, pendingIntent);//弹出权限框，进行权限申请
    }

    /**
     * 罗列usb设备
     * @param context 上下文
     */
    public static void listUsb(Context context) {
        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        UsbSerialProber usbDefaultProber = UsbSerialProber.getDefaultProber();
        for (UsbDevice device : usbManager.getDeviceList().values()) {
            UsbSerialDriver driver = usbDefaultProber.probeDevice(device);
            if (driver != null) {
                for (int port = 0; port < driver.getPorts().size(); port++) {
                    deviceId = device.getDeviceId();
                    portNum = port;
                }
                grantedUsbPermission(context, usbManager, device);
            }
        }
    }

    /**
     * 连接串口
     * @param context 上下文
     * @param withIoManager 是否开启接收线程
     */
    public static void connect(Context context, boolean withIoManager) {
        if (usbSerialPort != null) close();
        UsbDevice device = null;
        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        for (UsbDevice v : usbManager.getDeviceList().values())
            if (v.getDeviceId() == deviceId)
                device = v;
        if (device == null) {
//            setLog("connection failed: device not found");
            EventBus.getDefault().post(new ConnBean(ConnEnum.NOT_FOUND, "device not found"));
            return;
        }
        UsbSerialDriver driver = UsbSerialProber.getDefaultProber().probeDevice(device);
//        if(driver == null) {
//            driver = CustomProber.getCustomProber().probeDevice(device);
//        }
        if (driver == null) {
//            setLog("connection failed: no driver for device");
            EventBus.getDefault().post(new ConnBean(ConnEnum.NOT_FOUND, "device not found"));
            return;
        }
        if (driver.getPorts().size() < portNum) {
//            setLog("connection failed: not enough ports at device");
            EventBus.getDefault().post(new ConnBean(ConnEnum.NOT_ENOUGH_PORT, "not enough ports at device"));
            return;
        }
        usbSerialPort = driver.getPorts().get(portNum);
        UsbDeviceConnection usbConnection = usbManager.openDevice(driver.getDevice());
        if (usbConnection == null && usbPermission == UsbPermission.Unknown && !usbManager.hasPermission(driver.getDevice())) {
            usbPermission = UsbPermission.Requested;
            grantedUsbPermission(context, usbManager, driver.getDevice());
            return;
        }
        if (usbConnection == null) {
            if (!usbManager.hasPermission(driver.getDevice())) {
                EventBus.getDefault().post(new ConnBean(ConnEnum.NOT_GRANTED, "permission denied"));
            } else {
                EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, "open failed"));
            }
            return;
        }

        try {
            usbSerialPort.open(usbConnection);
            usbSerialPort.setParameters(115200, 8, 1, UsbSerialPort.PARITY_NONE);
            if (withIoManager) {
                usbIoManager = new SerialInputOutputManager(usbSerialPort, new SerialInputOutputManager.Listener() {
                    @Override
                    public void onNewData(byte[] data) {
                        EventBus.getDefault().post(new ConnBean(ConnEnum.MESSAGE, HexUtils.bytes2HexStr(data)));
                    }

                    @Override
                    public void onRunError(Exception e) {
                        EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, e.getMessage()));
                    }
                });
                usbIoManager.start();
            }
            EventBus.getDefault().post(new ConnBean(ConnEnum.CONNECTED, "连接成功"));
        } catch (Exception e) {
//            setLog("connection failed: " + e.getMessage());
//            disconnect();
            EventBus.getDefault().post(new ConnBean(ConnEnum.ERROR, e.getMessage()));
        }
    }

    public static void sendData(String[] hexString) throws Exception {
        if (usbSerialPort == null) return;
// 在这里需要转换byte数组，因为串口按位（bit）发送和接收字节
        byte[] bytes = new byte[hexString.length];
        for (int i = 0; i < hexString.length; i++) {
            bytes[i] = (byte) Integer.parseInt(hexString[i].substring(2), 16);
        }
        usbSerialPort.write(bytes, bytes.length);
    }

    /**
     * 发送数据
     * @param cmd 字节数组
     * @return 写入成功或者失败
     */
    public static boolean sendData(byte[] cmd) {
        if (usbSerialPort == null) return false;
        try {
            usbSerialPort.write(cmd, cmd.length);
            return true;
        } catch (IOException e) {
            e.printStackTrace();
        }
        return false;
    }

    /**
     * 写入数据
     * @param cmd 16进制字符串
     * @return 写入成功或者失败
     */
    public static boolean sendData(String cmd) {
        if (usbSerialPort == null) return false;
        return sendData(HexUtils.hexStringToByteArray(cmd));
    }

    /**
     * 关闭连接
     */
    public static void close() {
        if (usbSerialPort == null) return;
        if (usbIoManager != null) {
            usbIoManager.setListener(null);
            usbIoManager.stop();
        }
        usbIoManager = null;
        try {
            usbSerialPort.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        usbSerialPort = null;
        EventBus.getDefault().post(new ConnBean(ConnEnum.DISCONNECTED, "连接已关闭"));
    }

    private static class UsbReceiver extends BroadcastReceiver {

        @Override
        public void onReceive(Context context, Intent intent) {
            switch (intent.getAction()) {
                case UsbManager.ACTION_USB_DEVICE_ATTACHED:
                case UsbManager.ACTION_USB_ACCESSORY_ATTACHED:
                    listUsb(context);
                    break;
                case UsbManager.ACTION_USB_DEVICE_DETACHED:
                    EventBus.getDefault().post(new ConnBean(ConnEnum.REMOVE, "usb连接已断开"));
                    break;
                case GRANTER_USB_DEVICE:
                    usbPermission = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)
                            ? UsbPermission.Granted : UsbPermission.Denied;
                    connect(context, true);
                    break;
            }
        }
    }
}

```
### 事件类
#### ConnBean

```java
public class ConnBean {
    private ConnEnum type;
    private String message;

    public ConnBean(ConnEnum type, String message) {
        this.type = type;
        this.message = message;
    }

    public ConnEnum getType() {
        return type;
    }

    public void setType(ConnEnum type) {
        this.type = type;
    }

    public String getMessage() {
        return message;
    }

    public void setMessage(String message) {
        this.message = message;
    }
}

```
#### ConnEnum
```java
public enum ConnEnum {
    NOT_FOUND,
    NOT_ENOUGH_PORT,
    NOT_GRANTED,
    ERROR,
    CONNECTED,
    DISCONNECTED,
    MESSAGE,
    REMOVE
}
```

### Activity中使用
#### API调用
- 初始化usb设备
```java
UsbSerialManager.register(this)
```

- 注销关闭USB串口连接
```
UsbSerialManager.unregister(this)
```

- 发送数据
```java
UsbSerialManager.sendData("")
```

#### 注册EventBus以及UsbSerialManager

```kotlin
override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
		EventBus.getDefault().register(this)
		UsbSerialManager.register(this)
}

override fun onDestory(){
	EventBus.getDefault().unregister(this)
	UsbSerialManager.unregister(this)
}
```
#### 监听事件回调
```kotlin
@Subscribe(threadMode = ThreadMode.MAIN)
fun onUsbDetectedEvent(event: ConnBean) {
    when (event.type) {
        ConnEnum.NOT_FOUND,
        ConnEnum.NOT_ENOUGH_PORT,
        ConnEnum.NOT_GRANTED,
        ConnEnum.ERROR,
        ConnEnum.DISCONNECTED,
        ConnEnum.REMOVE -> {
            binding.tvConnStatus.isSelected = false
            Toast.makeText(
                this,
                "${event.type}: ${event.message}",
                Toast.LENGTH_SHORT
            ).show()
            LOG.e(TAG, "串口状态<-${event.type}: ${event.message}")
        }
        ConnEnum.CONNECTED -> {
            Log.e(TAG, "串口状态<-${event.type}: ${event.message}")
        }
        ConnEnum.MESSAGE -> {
            Log.e(TAG, "onUsbDetectedEvent: 收到数据${event.message}")
        }
        else -> {
            Log.e(TAG, "onUsbDetectedEvent: else")
        }
    }
}
```