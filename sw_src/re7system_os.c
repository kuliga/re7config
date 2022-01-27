/**
 * Common includes.
 */
#include "xparameters.h"
#include "xtmrctr.h"
#include "xgpio.h"
#include "xhwicap.h"
#include "mb_interface.h"
#include "xaxidma.h"
#include "xil_printf.h"
#include "xintc.h"
#include "netif/xadapter.h"
#include "xil_cache.h"

/**
 * LwIP includes.
 */
#include "lwip/tcp.h"
#include "pr_tftp.h"

/**
 * FreeRTOS includes.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define MHZ 		(66)
#define TIMER_TLR 	(25000000*((float)MHZ/100))
#define ETH_LINK_DETECT_INTERVAL 4
#define PR_TFTP_TIMEOUT_THRESHOLD 50
#define PR_TFTP_TIMEOUT_RETRY      3

/* to be determined */
#warning
#define CRC32_POLYNOMIAL (0x48679999U) 
/*
 * Tasks declaration.
 */
static void tftp_client(void *param);
static void verify_bitstream(void *param);
static void control_icap(void *param);
static void ui(void *param);

/*
 * Task handles.
 */
static TaskHandle_t tftp_client_task;
static TaskHandle_t verify_bitstream_task;
static TaskHandle_t control_icap_task;
static TaskHandle_t ui_task;

/*
 * FreeRTOS objects.
 */
static TimerHandle_t lwip_vtmr;
static TimerHandle_t gpio_vtmr;

/*
 * Global instances.
 */
static XTmrCtr tmr0;
static XAxiDma dma0;
static XHwIcap hwicap0;
static XGpio gpio0;
static XIntc intc0;

/* lwip objects */
static ip_addr_t local_ipaddr, server_ipaddr, gateway_ipaddr, netmask;
static unsigned char mac_eth_addr[] =                                               
	    { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };
static struct netif netif;
static pr_tftp_options_s transfer_opts = {
		.ReallocateMemoryIfRequired	= 0,
		.IncrementAmount		= 0,
		.DebugTftp			= 0,
		.DebugMemoryAllocation		= 0
};

/* Global variables */
static unsigned int buttons_fsm;
static char *bitstream = 0x85000000U;
u32 *bitstream_buffer = 0x84000000U;
static char *bitstream_name;
static u32 bitstream_size;
static unsigned int verification_fail_stats;
static unsigned int bitstream_write_fail_stats;

void lwip_vtmr_callback(TimerHandle_t vtmr);
void gpio_vtmr_callback(TimerHandle_t vtmr);

/*
 * ISRs.
 */
void gpio_isr(void *param);
void crc32blaze_isr(void *param);

/* for some reason this has to be declared here */
void lwip_init();

static volatile unsigned int tftp_timer_count;
static volatile u32 gpio_pins;

int main(void)
{
	microblaze_enable_dcache();
	microblaze_enable_icache();

	int status = -1;
	/* Initialize all driver instances */
	status = XIntc_Initialize(&intc0, 0);
	if (status) {
		xil_printf("\r\nintc init fault");
		return -1;
	}

	microblaze_register_exception_handler(0, (Xil_ExceptionHandler) XIntc_DeviceInterruptHandler,
											&intc0);

	XIntc_Start(&intc0, XIN_REAL_MODE);

	XIntc_Connect(&intc0, XPAR_INTC_0_GPIO_0_VEC_ID, gpio_isr, NULL);
	XIntc_Enable(&intc0,  XPAR_INTC_0_GPIO_0_VEC_ID);
	
	XIntc_Connect(&intc0, XPAR_INTC_0_CRC32BLAZE_0_VEC_ID, crc32blaze_isr, NULL);
	XIntc_Enable(&intc0,  XPAR_INTC_0_CRC32BLAZE_0_VEC_ID);
	
	/* ISR for emaclite is registered in lwip_init() */
	XIntc_Enable(&intc0, XPAR_INTC_0_EMACLITE_0_VEC_ID);
	
	/* Timer initialization is done inside FreeRTOS. */
	
	/* Initialize all driver instances */	
	status = XGpio_Initialize(&gpio0, 0);
	if (status) {
		xil_printf("\r\ngpio init fault");
		return -1;
	}

	XGpio_InterruptGlobalEnable(&gpio0);
	
	XHwIcap_Config *hwicap0_cfg = XHwIcap_LookupConfig(XPAR_HWICAP_0_DEVICE_ID);
	status = XHwIcap_CfgInitialize(&hwicap0, hwicap0_cfg, XPAR_HWICAP_0_BASEADDR);
	if (status) {
		xil_printf("\r\nhwicap init fault");
		return -1;
	}

	XAxiDma_Config *dma0_cfg = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	status = XAxiDma_CfgInitialize(&dma0, dma0_cfg);
	if (status) {
		xil_printf("\r\ndma init fault");
		return -1;
	}

	/* initialize lwip */
	IP4_ADDR(&local_ipaddr, 192, 168, 2, 10);
	IP4_ADDR(&server_ipaddr, 192, 168, 2, 11);
	IP4_ADDR(&gateway_ipaddr, 192, 168, 2, 1); /* not relevant */
	IP4_ADDR(&netmask, 255, 255, 255, 0);

	lwip_init();

	if (!xemac_add(&netif, &local_ipaddr, &netmask, &gateway_ipaddr, mac_eth_addr, 
						XPAR_AXI_ETHERNETLITE_0_BASEADDR)) {
		xil_printf("\r\nnetwork intf add fault");
		return -1;
	}

	netif_set_default(&netif);
	netif_set_up(&netif);

	lwip_vtmr = xTimerCreate("lwip timer", TIMER_TLR, 1, (void*) 0, lwip_vtmr_callback);
	gpio_vtmr = xTimerCreate("gpio debounce timer", pdMS_TO_TICKS(200), 0, (void*) 1, 
									gpio_vtmr_callback);

	/* register rtos tasks */
	BaseType_t task_creation_status = xTaskCreate(tftp_client, "tftp client", 0x200,
							NULL, 1, &tftp_client_task);
	if (task_creation_status != pdPASS) {
		xil_printf("\r\ntftp task creation failed");
		return -1;
	}

	task_creation_status = xTaskCreate(verify_bitstream, "verify bitstream", 0x200,
							NULL, 1, &verify_bitstream_task);
	if (task_creation_status != pdPASS) {
		xil_printf("\r\nverify bitstream task creation failed");
		return -1;
	}

	task_creation_status = xTaskCreate(control_icap, "control icap", 0x200,
							NULL, 1, &control_icap_task);
	if (task_creation_status != pdPASS) {
		xil_printf("\r\ncontrol icap task creation failed");
		return -1;
	}

	task_creation_status = xTaskCreate(ui, "ui task", 0x200, NULL, 1, 
								&ui_task);
	if (task_creation_status != pdPASS) {
		xil_printf("\r\nui task creation failed");
		return -1;
	}

	microblaze_enable_interrupts();

	vTaskStartScheduler();

	for (;;) {
	
	}

	return 0;

}

static void tftp_client(void *param)
{
	xTimerStart(lwip_vtmr, 0);
	

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	

		int status = PR_TFTP_FetchPartialToMem(&netif, server_ipaddr, bitstream_name,
					(char**) &bitstream, &bitstream_size, &bitstream_size, 
					&tftp_timer_count, PR_TFTP_TIMEOUT_THRESHOLD,
					PR_TFTP_TIMEOUT_RETRY, &transfer_opts);

		/* memcpy received file */
		memcpy(bitstream_buffer, bitstream, bitstream_size);

		xTaskNotifyGive(verify_bitstream_task);
	}
}

static void verify_bitstream(void *param)
{

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	

		mbar(3);
		microblaze_flush_dcache();
			
		int status = XAxiDma_SimpleTransfer(&dma0, (UINTPTR) bitstream, bitstream_size,
									XAXIDMA_DMA_TO_DEVICE);	
		if (status) {
			XAxiDma_Reset(&dma0);	
			xil_printf("\r\ndma transfer fail");
		}

		/* basic setup with polling dma status, to be changed later */
		while (XAxiDma_Busy(&dma0, XAXIDMA_DMA_TO_DEVICE));
	}
}

static void control_icap(void *param)
{

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	

		XHwIcap_FlushFifo(&hwicap0);

		if (XHwIcap_DeviceWrite(&hwicap0, bitstream, bitstream_size / 4))
			bitstream_write_fail_stats++;
	}
}

static void ui(void *param)
{
	/* this task should be waken up from gpio_isr() */
 	XGpio_InterruptEnable(&gpio0, 2);

	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	
		
		switch (gpio_pins) {
		case 1: 
			bitstream_name = "red_x32_nobitswap.bin";
			break;
		case 2:
			bitstream_name = "green_x32_nobitswap.bin";
			break;
		case 4: 
			bitstream_name = "blue_x32_nobitswap.bin";
			break;
		default:
			bitstream_name = "static_x32_nobitswap.bin";
			break;
		}

		/* notify tftp client */
		xTaskNotifyGive(tftp_client_task);		
	}
}

void gpio_isr(void *param)
{
	UBaseType_t isr_flags = taskENTER_CRITICAL_FROM_ISR();

	XIntc_Acknowledge(&intc0, XPAR_INTC_0_GPIO_0_VEC_ID);
	gpio_pins = XGpio_DiscreteRead(&gpio0, 2);	
	buttons_fsm = 1;
	XGpio_InterruptDisable(&gpio0, 2);

	taskEXIT_CRITICAL_FROM_ISR(isr_flags);

	BaseType_t flag;

	if (!xTimerStartFromISR(&gpio_vtmr, &flag))
		xil_printf("\r\ngpio_vtmr queue in isr full");

	if (flag)
		xil_printf("\r\ncontext switch needed!");
	
}

void crc32blaze_isr(void *param)
{
	UBaseType_t isr_flags = taskENTER_CRITICAL_FROM_ISR();

	XIntc_Acknowledge(&intc0, XPAR_MICROBLAZE_0_AXI_INTC_CRC32BLAZE_0_INTERRUPT_INTR);

 	u32 checksum= Xil_In32(XPAR_CRC32BLAZE_0_S00_AXI_BASEADDR + 0xc);

	/* clear interrupt */
	Xil_Out32(XPAR_CRC32BLAZE_0_S00_AXI_BASEADDR + 0x8, 1);
	/* initialize block */
	Xil_Out32(XPAR_CRC32BLAZE_0_S00_AXI_BASEADDR + 0x4, 1);

	taskEXIT_CRITICAL_FROM_ISR(isr_flags);

	if (checksum != CRC32_POLYNOMIAL) {
		verification_fail_stats++;
		return;
	}

	BaseType_t flag;	
	vTaskNotifyGiveFromISR(control_icap_task, &flag);
	if (flag)
		xil_printf("\r\ncontext switch needed!");
}

void gpio_vtmr_callback(TimerHandle_t vtmr)
{
	u32 gpio_curr_state = XGpio_DiscreteRead(&gpio0, 2);

	switch (buttons_fsm) {
	u32 gpio_tmp_state;
	case 1:	/* gpio_pins value set in gpio_isr() */
		gpio_pins &= gpio_curr_state;
		if (gpio_pins) {
			buttons_fsm = 2;
			if (!xTimerStart(vtmr, pdMS_TO_TICKS(200)))
				xil_printf("\r\ngpio_vtmr queue in callback full");

		} else {
			goto reset_fsm;
		}
		return;
	case 2: /* gpio_pins value set in fsm */
		gpio_tmp_state = gpio_pins ^ gpio_curr_state;
		if (!gpio_tmp_state) { 
			/* the button is still being pressed */
			if (!xTimerStart(&vtmr, pdMS_TO_TICKS(200)))
				xil_printf("\r\ngpio_vtmr queue in callback full");
		} else {
			gpio_pins = gpio_tmp_state;
			xTaskNotifyGive(ui_task);		
			goto reset_fsm;
		}
		return;
	default:
		return;
	}

reset_fsm:
	buttons_fsm = 0;
	XGpio_InterruptEnable(&gpio0, 2);
}

void lwip_vtmr_callback(TimerHandle_t vtmr)
{
	static volatile int TcpFastTmrFlag = 0;
	static volatile int TcpSlowTmrFlag = 0;
	static int DetectEthLinkStatus = 0;
	/* we need to call tcp_fasttmr & tcp_slowtmr at intervals specified by lwIP.
	 * It is not important that the timing is absoluetly accurate.
	 */
	static int odd = 1;

	DetectEthLinkStatus++;
	TcpFastTmrFlag = 1;
	tftp_timer_count++;
	odd = !odd;
	if (odd) {
		TcpSlowTmrFlag = 1;
	}

	/* For detecting Ethernet phy link status periodically */
	if (DetectEthLinkStatus == ETH_LINK_DETECT_INTERVAL) {
		eth_link_detect(&netif);
		DetectEthLinkStatus = 0;
	}
}
