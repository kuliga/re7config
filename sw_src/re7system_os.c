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


int main(void)
{
	return 0;
}

static void tftp_client(void *param)
{

}

static void verify_bitstream(void *param)
{

}

static void control_icap(void *param)
{

}

static void ui(void *param)
{

}
