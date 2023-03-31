---
sidebar_position: 0
title: MT7621-Uboot开发
---

Uboot开发udp通信


## U-Boot UDP通信

参考[U-Boot 中添加自定义网络通信的方法 - 腾讯云开发者社区-腾讯云 (tencent.com)](https://cloud.tencent.com/developer/article/1155291)

### xrdup.c

在net目录下新建xrdup.c文件，编写如下内容

```shell
#include <xrudp.h>
#include <common.h>
#include <command.h>
#include <net.h>
#include "tftp.h"
#include "bootp.h"

#include <linux/string.h>

#define TIMEOUT 1000 /* Seconds to timeout for a lost pkt	*/
// #ifndef	CONFIG_NET_RETRY_COUNT
#define TIMEOUT_COUNT 10 /* # of timeouts before giving up  */
#define PROTOCOL_LEN 5
// #else
// # define TIMEOUT_COUNT  (CONFIG_NET_RETRY_COUNT * 2)
// #endif

static ulong UdpTimeoutMSecs = TIMEOUT;
static int UdpTimeoutCountMax = TIMEOUT_COUNT;

/*
 * These globals govern the timeout behavior when attempting a connection to a
 * TFTP server. UdpRRQTimeoutMSecs specifies the number of milliseconds to
 * wait for the server to respond to initial connection. Second global,
 * UdpRRQTimeoutCountMax, gives the number of such connection retries.
 * UdpRRQTimeoutCountMax must be non-negative and UdpRRQTimeoutMSecs must be
 * positive. The globals are meant to be set (and restored) by code needing
 * non-standard timeout behavior when initiating a TFTP transfer.
 */
ulong UdpRRQTimeoutMSecs = TIMEOUT;
int UdpRRQTimeoutCountMax = TIMEOUT_COUNT;

static IPaddr_t UdpServerIP;
#define XRUdpServerPort 9000 /* The UDP port at their end		*/
#define XRUdpOurPort 9000    /* The UDP port at our end		*/
static int UdpTimeoutCount;

static void XRUdpSend(uchar *);
static void UdpTimeout(void);

/**********************************************************************/
/**
 *
 */
static void
XRUdpSend(uchar *pkt_data)
{
    volatile uchar *pkt;
    volatile uchar *xp;
    int len = 0;
    int uplen = 0;
    volatile ushort *s;

    // char *pkt_data = "client start....";
    /*
     *	We will always be sending some sort of packet, so
     *	cobble together the packet headers now.
     */
    pkt = NetTxPacket + NetEthHdrSize() + IP_HDR_SIZE;
    len = strlen(pkt_data);
    memcpy(pkt, pkt_data, len);
    printf("pkt_data=%s,len=%d\n", pkt_data, len);
    NetSendUDPPacket(NetServerEther, UdpServerIP, XRUdpServerPort, XRUdpOurPort, len);
}

int RECV_FLAG = 0;
/**
 * @brief xrudp timeout callback function
 *
 */
static void UdpTimeout()
{

    int i = 0;
    // delay and wait for tcp data
    for (i = 0; i < 100; ++i)
    {
        // printf("counting......\n");
        udelay(10000);
    }
    if (RECV_FLAG == 1)
    {
        printf("------receive server data-------\n");
        NetState = NETLOOP_CONTINUE;
    }
    else
    {
        printf("----xr udp time out set netstate----\n");
        NetState = NETLOOP_SUCCESS;
    }
}
/**
 * @brief judge whether string start with substring
 *
 * @param str1 source string
 * @param str2 substring
 * @return int 1: true 0 false
 */
int isStartWith(const char *str1, char *str2)
{
    if (str1 == NULL || str2 == NULL)
        return -1;
    int len1 = strlen(str1);
    int len2 = strlen(str2);
    if ((len1 < len2) || (len1 == 0 || len2 == 0))
        return -1;
    char *p = str2;
    int i = 0;
    while (*p != '\0')
    {
        if (*p != str1[i])
            return 0;
        p++;
        i++;
    }
    return 1;
}
#define ARGV_LEN 128

typedef int (*XRCallBackFun)(uint8_t soid); // 为回调函数命名，类型命名为CallBackFun，参数为uint8_t s
XRCallBackFun XRUdpCallBack;                // 创建实例
/**
 * @brief register XRCallBackFunc
 *
 * @param P XRCallBackFun
 * @return int code
 */
int registerCallback(XRCallBackFun P) // 注册回调函数
{
    XRUdpCallBack = P;
    return 0;
}

/**
 * @brief decode received data from xrudp server
 *
 * @param data protocol string
 */
void decodeData(uchar *data)
{
    char *token;
    char *protocol[PROTOCOL_LEN] = {};
    /* 获取第一个子字符串 */
    int i = 0;
    token = strtok(data, ",");
    //    * 继续获取其他的子字符串 */
    protocol[i] = token;
    i++;
    while (token != NULL)
    {

        token = strtok(NULL, ",");
        if (i >= 5)
        {
            i = 0;
        }
        else
        {
            protocol[i] = token;
            i++;
        }
    }
    if (strcmp(protocol[1], "update") == 0)
    {
        // printf("user want to update firmware...\n");
        if (protocol[2] != NULL || protocol[3] != NULL)
        {
            setenv("bootfile", protocol[2]);
            setenv("tempVer", protocol[3]);
            // printf("temp version: %s\n", getenv("tempVer"));
            XRUdpSend("$xiaor,succeed,update,start,");
            XRUdpCallBack(0);
        }
        else
        {
            XRUdpSend("$xiaor,error,update,missing_params,");
        }
    }
    else if (strcmp(protocol[1], "ping") == 0)
    {
        // printf("testing communication....\n");
        // setenv("firVer", protocol[3]);
        XRUdpSend("$xiaor,succeed,ping,");
    }
    else if (strcmp(protocol[1], "check") == 0)
    {
        // printf("check firmware version...\n");
        char *s = getenv("firVer");
        if (s != NULL)
        {
            char *str;
            sprintf(str, "$xiaor,succeed,%s,%s,", protocol[1], s);
            XRUdpSend(str);
        }
        else
        {
            XRUdpSend("$xiaor,error,check,null,");
        }

        // XRUdpCallBack(1);
    }
    else if (strcmp(protocol[1], "boot") == 0)
    {
        XRUdpCallBack(1);
    }

    else if (strcmp(protocol[1], "uboot") == 0)
    {
        if (protocol[2] != NULL)
        {
            // printf("uboot fileName: %s\n", protocol[2]);
            setenv("bootfile", protocol[2]);
            XRUdpCallBack(2);
        }
        else
        {
            XRUdpSend("$xiaor,error,uboot,missing_params,");
        }
    }
    else
    {
        XRUdpCallBack(1);
    }
}
/**
 * @brief xrudp handler
 *
 * @param pkt received data
 * @param dest dest ip
 * @param srt source ip
 * @param len package length
 */
static void UdpHandler(uchar *pkt, unsigned dest, unsigned srt, unsigned len)
{
    ushort proto;
    ushort *s;
    int i;
    // printf("receive udp packet\n");
    s = (uchar *)pkt;
    // printf("len=%d\n", len);
    // printf("%.*s\n", len, s);
    // XRUdpSend(s);
    RECV_FLAG = 1;
    if (isStartWith(s, "$xiaor"))
    {
        decodeData(s);
    }
    else
    {
        printf("not xiaorgeek protocol, exit\n");
        RECV_FLAG = 0;
    }
}

void XRUdpStart(void)
{
    char *ep; /* Environment pointer */

    /*
     * Allow the user to choose UDP blocksize and timeout.
     * UDP protocol has a minimal timeout of 1 second.
     */

    if ((ep = getenv("udptimeout")) != NULL)
        UdpTimeoutMSecs = simple_strtol(ep, NULL, 10);

    if (UdpTimeoutMSecs < 1000)
    {
        printf("UDP timeout (%ld ms) too low, "
               "set minimum = 1000 ms\n",
               UdpTimeoutMSecs);
        UdpTimeoutMSecs = 1000;
    }

    UdpServerIP = NetServerIP;

#if defined(CONFIG_NET_MULTI)
    printf("Using %s device\n", eth_get_name());
#endif
    UdpTimeoutCountMax = UdpRRQTimeoutCountMax;

    NetSetTimeout(UdpTimeoutMSecs * CFG_HZ, UdpTimeout);
    NetSetHandler(UdpHandler);

    // UdpServerPort = 9000;//WELL_KNOWN_PORT;
    UdpTimeoutCount = 0;
    // UdpOurPort = 9000;
    // UdpPktLen = 0;

    /* zero out server ether in case the server ip has changed */
    memset(NetServerEther, 0, 6);

    XRUdpSend("$xiaor,ping,");
}
```

修改Makefile，添加xrdup.o

```makefile
OBJS	= net.o tftp.o eth.o xrudp.o
```



### xrudp.h

在include目录下，编写xrudp.h头文件

```c
#ifndef __XRUDP_H__
#define __XRUDP_H__

extern void XRUdpStart(void);
int registerCallback(XRCallBackFun);
#endif
```

### 在net.h文件添加自定的通信格式

```c
typedef enum { BOOTP, RARP, ARP, TFTP, DHCP, PING, DNS, NFS, CDP, NETCONS, /*添加这个内容*/XRUDP } proto_t;
```

### 修改net.c

修改net.c的NetLoop()函数

```c
#include<xrdup.h>
....
case TFTP:
		NetCopyIP(&NetOurIP, &bd->bi_ip_addr);
		NetOurGatewayIP = getenv_IPaddr ("gatewayip");
		NetOurSubnetMask= getenv_IPaddr ("netmask");
#ifdef CONFIG_NET_VLAN
		NetOurVLAN = getenv_VLAN("vlan");
		NetOurNativeVLAN = getenv_VLAN("nvlan");
#endif
		NetServerIP = getenv_IPaddr ("serverip");
		break;
		// 这里增加XRUDP的判断
	case XRUDP:
		// printf("-----------AMCUDP init ipaddress-----------\n");
		NetCopyIP(&NetOurIP, &bd->bi_ip_addr);
		NetOurGatewayIP = getenv_IPaddr ("gatewayip");
		NetOurSubnetMask= getenv_IPaddr ("netmask");
#ifdef CONFIG_NET_VLAN
		NetOurVLAN = getenv_VLAN("vlan");
		NetOurNativeVLAN = getenv_VLAN("nvlan");
#endif
		NetServerIP = getenv_IPaddr ("serverip");
		
		break;
....
    switch (protocol) {
		case TFTP:
			/* always use ARP to get server ethernet address */
			TftpStart();
			break;
        case XRUDP:
                // printf("------call amc udp start method------\n");
                // amc_udp_start();
                XRUdpStart();
                break;
...
```

### 修改board.c

包含头文件

```c
#include <xrudp.h>
// 定义一个宏变量
#define NEW_BOOT_OPTION
```

增加启动代码

```c
#ifndef NEW_BOOT_OPTION
	OperationSelect();
	// tftp_config(SEL_LOAD_LINUX_WRITE_FLASH, argv);

	while (timer1 > 0)
	{
		--timer1;
		/* delay 100 * 10ms */
		for (i = 0; i < 100; ++i)
		{
			if ((my_tmp = tstc()) != 0)
			{				/* we got a key press	*/
				timer1 = 0; /* no more delay	*/
				BootType = getc();
				if ((BootType < '0' || BootType > '5') && (BootType != '7') && (BootType != '8') && (BootType != '9'))
					BootType = '3';
				printf("\n\rYou choosed %c\n\n", BootType);
				break;
			}
			udelay(10000);
		}
		printf("\b\b\b%2d ", timer1);
	}
	putc('\n');
#endif
#ifdef NEW_BOOT_OPTION
	char *argv[4];
	int argc = 3;

	argv[2] = &file_name_space[0];
	memset(file_name_space, 0, ARGV_LEN);
#if (CONFIG_COMMANDS & CFG_CMD_NET)
	eth_initialize(gd->bd);
#endif
	setenv("autostart", "no");
	// do_tftpb(cmdtp, 0, argc, argv);
#ifdef DEBUG
	printf("------start udp client-----------\n");
#endif
	registerCallback(myCallback);
	udp_client();
	// if (UPDATE_FLAG == 1)
	// {
	// 	printf("update flag == 1\n");
	// 	UPDATE_FLAG = 0;
	// 	do_reset(cmdtp, 0, argc, argv);
	// }
	// else
	// {
	char *argv1[2];
	sprintf(addr_str, "0x%X", CFG_KERN_ADDR);
	argv1[1] = &addr_str[0];
	printf("   \n3: System Boot system code via Flash.\n");
	do_bootm(cmdtp, 0, 2, argv1);
	// }
#endif
```



## 通信协议

标准格式，暂定5个字段，分别包含功能码，参数1到4，注意在最后最好添加一个`,`否则可能出现由于数据发送端的异常，导致数据错乱

```shell
$xiaor,功能码,参数1,参数2,参数3,参数4
```

功能码

- update：更新固件
  - 参数1：文件名称
  - 参数2：固件版本
  - 剩余参数留空

- check：查询固件版本
- error: 底层返回错误的指令

```shell
#协议开头, error, 功能码, 错误原因
$xiaor,error,update,missing_params,
```

- succeed：底层返回指令执行成功的功能码

比如发送查询固件版本的指令

```shell
#协议开头, succeed, 功能码, 错误原因
$xiaor,succeed,check,1.3,
```

- ping：测试通讯

在进行固件更新以及版本查询的时候，用户首先需要发送test指令去与uboot的UDP进行握手，等待握手成功后，才可下发其他的相关指令，不要一开始就下发控制指令，实例代码如下：

```shell
# 请求握手
$xiaor,ping,
# 握手成功，软件返回
$xiaor,succeed,ping,
```

- uboot

进入uboot更新模式

```shell
$xiaor,uboot,<fileName>,,
```

- exit

退出udp服务，进入系统，在查询固件版本以及ping测试的时候，系统会自动挂起后面的线程，如果用户需要退出这两种模式，那么需要下发如下指令

```shell
$xiaor,exit,
```



