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
static unsigned int buttons_fsm;

void lwip_vtmr_callback(TimerHandle_t vtmr);
void gpio_vtmr_callback(TimerHandle_t vtmr)

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
	int status = -1;
	/* Initialize all driver instances */
	status = XIntc_Initialize(&intc, 0);
	if (status) {
		xil_printf("\r\nintc init fault");
		return -1;
	}

	XIntc_Start(&intc0, XIN_REAL_MODE);
	XIntc_Connect(&intc0, XPAR_INTC_0_GPIO_0_VEC_ID, gpio_isr, NULL);
	XIntc_Connect(&intc0, XPAR_INTC_0_CRC32BLAZE_0_VEC_ID, crc32blaze_isr, NULL);
	
	/* Timer initialization is done inside FreeRTOS. */
	
	/* Initialize all driver instances */	
	status = XGpio_Initialize(&gpio0, 0);
	if (status) {
		xil_printf("\r\ngpio init fault");
		return -1;
	}

	XGpio_GlobalInterruptEnable(&gpio0);
	
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

	lwip_vtmr = xTimerCreate("lwip timer", TIMER_TLR, 1, lwip_vtmr_callback); 
	gpio_vmtr = XTimerCreate("gpio debounce timer", pdMS_TO_TICKS(200), 0, gpio_vtmr_callback);	

	for (;;) {
	
	}

	return 0;

}

static void tftp_client(void *param)
{
	xTimerStart(lwip_vtmr, 0);
	

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
	/* this task should be waken up from gpio_isr() */
 	XGpio_InterruptEnable(&gpio0, 2);

	for (;;) {
		
	}
}

void gpio_isr(void *param)
{
	UBaseType_t isr_flags = taskENTER_CRITICAL_FROM_ISR();

	XIntc_Acknowledge(&xintc0, XPAR_INTC_0_GPIO_0_VEC_ID);
	gpio_pins = XGpio_DiscreteRead(&gpio0, 2);	
	buttons_fsm = 1;
	XGpio_DisableInterrupts(&gpio0, 2);

	taskEXIT_CRITICAL_FROM_ISR(isr_flags);

	BaseType_t gpio_vtmr_flag;

	if (!xTimerStartFromISR(&gpio_vtmr, &gpio_vtmr_flag))
		xil_printf("\r\ngpio_vtmr queue in isr full");

	if (flag)
		xil_printf("\r\ncontext switch needed!");
	
}

void crc32blaze_isr(void *param)
{

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
			if (!xTimerStart(vtmr, pdMS_TO_TICKS(200))
				xil_printf("\r\ngpio_vtmr queue in callback full");

		} else {
			goto reset_fsm;
		}
		return;
	case 2: /* gpio_pins value set in fsm */
		gpio_tmp_state = gpio_pins ^ gpio_curr_state;
		if (!gpio_tmp_state) { 
			/* the button is still being pressed */
			if (!xTimerStart(&vtmr, pdMS_TO_TICKS(200))
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
	XGpio_InterruptsEnable(&gpio0, 2);
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
		eth_link_detect(echo_netif);
		DetectEthLinkStatus = 0;
	}
}
