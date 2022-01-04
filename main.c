#include <stdio.h>

#include "xparameters.h"

#include "netif/xadapter.h"

#include "platform.h"
#include "platform_config.h"
#include "crc32.h"
#include "lwip/tcp.h"
#include "xil_cache.h"
#include "xdebug.h"
#include "pr_tftp.h"
#include "xaxidma.h"
#include "mb_interface.h"
#include "xhwicap.h"
#include "xgpio.h"
#include "xtmrctr.h"
/* defined by each RAW mode application */
void print_app_header();
int start_application();
int transfer_data();
void tcp_fasttmr(void);
void tcp_slowtmr(void);

static XAxiDma dma0;
static XHwIcap hwicap;
static XGpio gpio0;
static XTmrCtr tmr0;

/* missing declaration in lwIP */
void lwip_init();

extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;
static struct netif server_netif;
struct netif *echo_netif;

extern volatile unsigned int tftp_timer_count;
#define PR_TFTP_TIMEOUT_THRESHOLD 50
#define PR_TFTP_TIMEOUT_RETRY      3

pr_tftp_options_s transfer_opts = {
		.ReallocateMemoryIfRequired = 0,
		.IncrementAmount = 0,
		.DebugTftp = 0,
		.DebugMemoryAllocation = 0
};

void
print_ip(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
			ip4_addr3(ip), ip4_addr4(ip));
}

void
print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{

	print_ip("Board IP: ", ip);
	print_ip("Netmask : ", mask);
	print_ip("Gateway : ", gw);
}

void print_app_header(ip_addr_t *TftpServerIpAddr){
    xil_printf("======================================================================================\n\r");
    xil_printf("An example Microblaze application that fetches partial bitstreams from a TFTP server,\n\r");
    xil_printf("stores them in memory, and then programs the PRC to manage those partial bitstreams.\n\r");
    xil_printf("======================================================================================\n\r");
    xil_printf("  RPs and RMs\n\r");
    xil_printf("  -----------\n\r");
    xil_printf("  The design is based on the standard PR tutorial design consisting of SHIFT and COUNT\n\r");
    xil_printf("  RPs.  It is implemented on a KC705 board\n\r");
    xil_printf("   \n\r");
    xil_printf("  TFTP server\n\r");
    xil_printf("  -----------\n\r");
    xil_printf("  The application requires a TFTP server to be running on the computer at \n\r");
    xil_printf("  %d.%d.%d.%d.\n\r", ip4_addr1(TftpServerIpAddr), ip4_addr2(TftpServerIpAddr),
                                     ip4_addr3(TftpServerIpAddr), ip4_addr4(TftpServerIpAddr));
    xil_printf(" \n\r");
    xil_printf(" !!!! Important !!!! Your firewall may block TFTP transactions so you may need\n\r");
    xil_printf(" !!!! Important !!!! to configure it to allow them, or temporarily disable it.\n\r");
    xil_printf("======================================================================================\n\r");
    xil_printf("\n\r");
}

void RecvDataCallback (void *Arg, char * Data, int unsigned NumberOfBytes){
  int Status;

  XHwIcap *InstancePtr = (XHwIcap*) Arg;

  // Data is a non word aligned array of chars in network byte order.  Non word aligned means
  // the data can be in memory like this:
  //  Address
  //    000CD420     00DD FFFF
  //    000CD424     0022 0000
  //    000CD428     FFFF 8844
  //    000CD42C     FFFF FFFF
  //    000CD430     AA66 FFFF   <--         Part of Sync Word
  //    000CD430     0000 5599   <-- Another part of Sync Word
  //
  // Notice that the sync word (0xAA995566) is split over two 32 memory locations.
  // This means we can't access the data as 32 bit data by just reinterpreting the
  // buffer using a pointer cast.  We need to copy it to a 32 bit aligned buffer.
  //
  // If TFTP options are not used (they aren't supported in the PR TFTP library without modification)
  // then the maximum size of the data payload will be 512 bytes (128 32 bit words).  Declare a static
  // array of 128 32 bit words for speed.  Malloc and free could be used, but that would have a speed
  // impact
  //
  static u32 FormattedPayload[128];
  memcpy(FormattedPayload, Data, NumberOfBytes);


  // The bitstreams used in this example are the bin files created by write_bitstream.
  // They are in a little endian format but become big endian when transmitted over the network
  // (network byte order is always big endian).  This loop converts them from network byte order
  // (big endian) back to host byte order (little endian in this example).
  //
  // If the bitstreams from the Partials directory were used instead, this code wouldn't be needed.
  // The bitstreams in the Partials directory are created using "write_cfgmem -interface SMAPx32"
  // which converts them to big endian for the ICAP.  When they are transmitted over TFTP they are
  // byte swapped back to little endian, so there's nothing we need to do here.

//  for (int i = 0; i < NumberOfBytes/4; i++) {
//    FormattedPayload[i] = ntohl(FormattedPayload[i]);
//  }


  Status = XHwIcap_DeviceWrite(InstancePtr,              // A pointer to the XHwIcap instance.
                               FormattedPayload,         // A pointer to the data to be written to the ICAP device.
							   (u32)(NumberOfBytes / 4)  // The number of words to write to the ICAP device.
                               );
  if (Status != XST_SUCCESS) {
    xil_printf("RecvDataCallback DeviceWrite failed\n\r");
  }

  if (NumberOfBytes != 512) {
    xil_printf("RecvDataCallback finished loading partial bitstream\n\r");
  }
}

char *file = (u8 *) 0x85000000U;
u32 file_size = 0x02000000U;
u32 bytes_stored = 0;
u32 *file_buffer = 0x84000000U;
int main()
{
	memset(file, 0, file_size);
	ip_addr_t local_ipaddr, server_ipaddr, netmask, gw;

	/* the mac address of the board. this should be unique per board */
	unsigned char mac_ethernet_address[] =
	{ 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

	echo_netif = &server_netif;

	init_platform();

	/* initliaze IP addresses to be used */
	IP4_ADDR(&local_ipaddr,  192, 168,   2, 10);
	IP4_ADDR(&netmask, 255, 255, 255,  0);
	IP4_ADDR(&gw,      192, 168,   2,  1);
	IP4_ADDR(&server_ipaddr, 192, 168, 2, 11);

	print_app_header(&server_ipaddr);

	lwip_init();

	XTmrCtr_Initialize(&tmr0, 0);

	/* gpio */
	XGpio_Initialize(&gpio0, 0);
	/* dma */
	int status;
	XAxiDma_Config *dma0_cfg = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
    status = XAxiDma_CfgInitialize(&dma0, dma0_cfg);

    /* icap */
    XHwIcap_Config *hwicap_cfg = XHwIcap_LookupConfig(XPAR_HWICAP_0_DEVICE_ID);
    status = XHwIcap_CfgInitialize(&hwicap, hwicap_cfg, XPAR_HWICAP_0_BASEADDR);

	/* Add network interface to the netif_list, and set it as default */
	if (!xemac_add(echo_netif, &local_ipaddr, &netmask,
						&gw, mac_ethernet_address,
						PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\n\r");
		return -1;
	}

	netif_set_default(echo_netif);

	/* now enable interrupts */
	platform_enable_interrupts();

	/* specify that the network if is up */
	netif_set_up(echo_netif);

	print_ip_settings(&local_ipaddr, &netmask, &gw);

	for (;;) {
		u32 io_val = XGpio_DiscreteRead(&gpio0, 2);
		char *name = NULL;

		switch (io_val) {
		case 1:
			name = "red_x32_nobitswap.bin";
			break;
		case 2:
			name = "green_x32_nobitswap.bin";
			break;
		case 4:
			name = "blue_x32_nobitswap.bin";
			break;
		default:
			name = "static_x32_nobitswap.bin";
//			name = "red_x32_nobitswap.bin";
			break;
		}

		XGpio_DiscreteWrite(&gpio0, 1, 0x0);

		XHwIcap_FlushFifo(&hwicap);
		XTmrCtr_Reset(&tmr0, 1);
		u32 tmrval = XTmrCtr_GetValue(&tmr0, 1);
		xil_printf("\r\ntimer input %u", tmrval);

		XTmrCtr_Start(&tmr0, 1);
//		int err = PR_TFTP_FetchPartialToFunction(echo_netif, server_ipaddr, name,
//										RecvDataCallback, (void*) &hwicap,
//										 &tftp_timer_count,
//										PR_TFTP_TIMEOUT_THRESHOLD, PR_TFTP_TIMEOUT_RETRY,
//										&transfer_opts);

		int error = PR_TFTP_FetchPartialToMem(echo_netif, server_ipaddr, name,
					(char**) &file, &file_size, &bytes_stored, &tftp_timer_count,
					PR_TFTP_TIMEOUT_THRESHOLD, PR_TFTP_TIMEOUT_RETRY,
					&transfer_opts);

		memcpy(file_buffer, file, 666000);

		mbar(3);
		microblaze_flush_dcache();
		status = XAxiDma_SimpleTransfer(&dma0, (UINTPTR) file, bytes_stored,
											XAXIDMA_DMA_TO_DEVICE);

			XGpio_DiscreteWrite(&gpio0, 1, 0xFFFF);
	//    XAxiDma_Reset(&dma0);
		while (XAxiDma_Busy(&dma0,XAXIDMA_DMA_TO_DEVICE)) {
				/* Wait */
		}

		XHwIcap_DeviceWrite(&hwicap,      // A pointer to the XHwIcap instance.
		                               file_buffer,         // A pointer to the data to be written to the ICAP device.
									   (u32)(bytes_stored / 4)  // The number of words to write to the ICAP device.
		                               );


		XTmrCtr_Stop(&tmr0, 1);
		tmrval = XTmrCtr_GetValue(&tmr0, 1);
		xil_printf("\r\ntimer out: %u", tmrval);
		XHwIcap_FlushFifo(&hwicap);

	}

//	char *name = "red_x32_nobitswap.bin";
////	int error = PR_TFTP_FetchPartialToMem(echo_netif, server_ipaddr, name,
////				(char**) &file, &file_size, &bytes_stored, &tftp_timer_count,
////				PR_TFTP_TIMEOUT_THRESHOLD, PR_TFTP_TIMEOUT_RETRY,
////				&transfer_opts);
//	XHwIcap_FlushFifo(&hwicap);
//	u32 tmrval = XTmrCtr_GetValue(&tmr0, 1);
//	xil_printf("\r\n timer %u", tmrval);
//
//	XTmrCtr_Start(&tmr0, 1);
//	int err = PR_TFTP_FetchPartialToFunction(echo_netif, server_ipaddr, name,
//									RecvDataCallback, (void*) &hwicap,
//									 &tftp_timer_count,
//									PR_TFTP_TIMEOUT_THRESHOLD, PR_TFTP_TIMEOUT_RETRY,
//									&transfer_opts);
//
//	XTmrCtr_Stop(&tmr0, 1);
//	tmrval = XTmrCtr_GetValue(&tmr0, 1);
//	XHwIcap_FlushFifo(&hwicap);
//
//
//	int x = *(int*)file;
//	xil_printf("\r\nValue of first file byte: %x    %x", *file, x);
//	xil_printf("\r\n file size: %u", bytes_stored);
//	xil_printf("\r\n timer %u", tmrval);
//
//	XGpio_DiscreteWrite(&gpio0, 1, 0xFFFF);


//    u32 crc = rc_crc32(0xffffffff, (UINTPTR) file, bytes_stored);

//    mbar(3);
//    microblaze_flush_dcache();
//    status = XAxiDma_SimpleTransfer(&dma0, (UINTPTR) file, bytes_stored,
//										XAXIDMA_DMA_TO_DEVICE);
//
////    XAxiDma_Reset(&dma0);
//	while (XAxiDma_Busy(&dma0,XAXIDMA_DMA_TO_DEVICE)) {
//			/* Wait */
//	}

//	memmove(file_buffer, file, bytes_stored - 4);
//
//	for (int i = 0; i < (bytes_stored - 4) / 4; i++) {
//		file_buffer[i] = ntohl(file_buffer[i]);
//	}
//
//	status = XHwIcap_DeviceWrite(&hwicap, file_buffer, (bytes_stored - 4) / 4);

	for (;;) {

	}

	/* never reached */
	cleanup_platform();

	return 0;
}
