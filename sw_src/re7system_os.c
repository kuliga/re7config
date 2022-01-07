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
 * Global instances.
 */
static XTmrCtr tmr0;
static XAxiDma dma0;
static XHwIcap hwicap0;
static XGpio gpio0;
static XIntc intc0;

/*
 * ISRs.
 */
void gpio_isr(void *param);
void hwicap_isr(void *param);
void crc32blaze_isr(void *param);

/* for some reason this has to be declared here */
void lwip_init();

int main(void)
{
	int status;
	/* Initialize all driver instances */
	status = XIntc_Initialize(&intc, 0);
	if (status) {
		xil_printf("\r\nintc init fault");
		return -1;
	}

	XIntc_Start(&intc0, XIN_REAL_MODE);
	XIntc_Connect(&intc0, XPAR_INTC_0_GPIO_0_VEC_ID, gpio_isr, NULL);
	XIntc_Connect(&intc0, XPAR_INTC_0_HWICAP_0_VEC_ID, hwicap_isr, NULL);
	XIntc_Connect(&intc0, XPAR_INTC_0_CRC32BLAZE_0_VEC_ID, crc32blaze_isr, NULL);
	
	/*
	 * Timer initialization is done inside FreeRTOS. 
	 */
	//status = XTmrCtr_Initialize(&tmr0, 0);
	//if (status) {
	//	xil_printf("\r\ntimer init fault");
	//	return -1;
	//}
	
	status = XGpio_Initialize(&gpio0, 0);
	if (status) {
		xil_printf("\r\ngpio init fault");
		return -1;
	}
	
	XHwIcap_Config *hwicap0_cfg = XHwIcap_LookupConfig(XPAR_HWICAP_0_DEVICE_ID);
	status = XHwIcap_Initialize(&hwicap0, hwicap0_cfg, XPAR_HWICAP_0_BASEADDR);
	if (status) {
		xil_printf("\r\nhwicap init fault");
		return -1;
	}

	XAxiDma_Config *dma0_cfg = XAxiDma_LookupConfig(XPAR_AXI_DMA_0_DEVICE_ID);
	status = XAxiDma_Initialize(&dma0, dma0_cfg);
	if (status) {
		xil_printf("\r\ndma init fault");
		return -1;
	}

	for (;;) {
	
	}

	return 0;

}

static void tftp_client(void *param)
{

	for (;;) {
		
	}
}

static void verify_bitstream(void *param)
{

	for (;;) {
		
	}
}

static void control_icap(void *param)
{

	for (;;) {
		
	}
}

static void ui(void *param)
{

	for (;;) {
		
	}
}

void gpio_isr(void *param)
{

}

void hwicap_isr(void *param)
{

}
void crc32blaze_isr(void *param)
{

}
