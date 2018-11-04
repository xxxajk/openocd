/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2018 by Andrew J. Kroll <xxxajk@gmail.com>              *
 *   Based on opendous driver by Cahya Wirawan <cahya@gmx.at>              *
 *   Based on opendous driver by Vladimir Fonov <vladimir.fonov@gmai.com>  *
 *   Based on J-link driver by  Juergen Stuber <juergen@jstuber.net>       *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *   based on opendous driver by Spencer Oliver spen@spen-soft.co.uk       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "libusb_common.h"
#include <string.h>
#include <sys/timeb.h>
#include <time.h>

#define T3JTAG_MAX_VIDS_PIDS 2

/* define some probes with similar interface */
struct t3jtag_probe {
        char *name;
        uint16_t VID[T3JTAG_MAX_VIDS_PIDS];
        uint16_t PID[T3JTAG_MAX_VIDS_PIDS];
        uint8_t READ_EP;
        uint8_t WRITE_EP;
        uint8_t CONTROL_TRANSFER;
        int BUFFERSIZE;
};


static struct t3jtag_probe t3jtag_probes[] = {
        {"t3jtag",
                {0x16C0, 0},
                {0x03E8, 0}, 0x81, 0x02, 0x00, 64},
        {NULL,
                {0x0000},
                {0x0000}, 0x00, 0x00, 0x00, 0}
};

#define T3JTAG_WRITE_ENDPOINT   (t3jtag_probe->WRITE_EP)
#define T3JTAG_READ_ENDPOINT    (t3jtag_probe->READ_EP)

#define T3JTAG_USB_TIMEOUT      20000 // too short?

#define T3JTAG_USB_BUFFER_SIZE  (t3jtag_probe->BUFFERSIZE)
#define T3JTAG_IN_BUFFER_SIZE   (T3JTAG_USB_BUFFER_SIZE)
#define T3JTAG_OUT_BUFFER_SIZE  (T3JTAG_USB_BUFFER_SIZE)

/* Global USB buffers */
static uint8_t *usb_in_buffer;
static uint8_t *usb_out_buffer;
static uint8_t good;

/* Constants for T3JTAG command */
#define T3JTAG_MAX_TAP_TRANSMIT	((t3jtag_probe->BUFFERSIZE)-10)
#define T3JTAG_MAX_INPUT_DATA		(T3JTAG_MAX_TAP_TRANSMIT*4)

/* TAP */
#define T3JTAG_TAP_BUFFER_SIZE 65536

// SWD
//static const char * const t3jtag_transports[] = { "jtag", "swd", NULL };

struct pending_scan_result {
        int first; /* First bit position in tdo_buffer to read */
        int length; /* Number of bits to read */
        struct scan_command *command; /* Corresponding scan command */
        uint8_t *buffer;
};

static int pending_scan_results_length;
static struct pending_scan_result *pending_scan_results_buffer;

#define MAX_PENDING_SCAN_RESULTS (T3JTAG_MAX_INPUT_DATA)

/* JTAG usb commands */
#define JTAG_CMD_TAP_OUTPUT	0x0
#define JTAG_CMD_SET_TRST	0x1
#define JTAG_CMD_SET_SRST	0x2
#define JTAG_CMD_READ_INPUT	0x3
#define JTAG_CMD_TAP_OUTPUT_EMU	0x4
#define JTAG_CMD_SET_DELAY	0x5
#define JTAG_CMD_SET_SRST_TRST	0x6
#define JTAG_CMD_READ_CONFIG	0x7
// SWD commands
// Special Kinetis EZP usb commands
#define JTAG_CMD_KINETIS_ERASE  0xF
#define FUNC_WRITE_DATA       0x50
#define FUNC_READ_DATA        0x51

static char *t3jtag_type;
static struct t3jtag_probe *t3jtag_probe;


/* t3jtag lowlevel functions */
struct t3jtag_jtag {
        struct jtag_libusb_device_handle *usb_handle;
};



/* External interface functions */
static int t3jtag_execute_queue(void);
static int t3jtag_init(void);
static int t3jtag_quit(void);

/* Queue command functions */
static void t3jtag_end_state(tap_state_t state);
static void t3jtag_state_move(void);
static void t3jtag_path_move(int num_states, tap_state_t *path);
static void t3jtag_runtest(int num_cycles);
static void t3jtag_scan(int ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command);
static void t3jtag_reset(int trst, int srst);
static void t3jtag_simple_command(uint8_t command, uint8_t _data);
//static int t3jtag_get_status(void);

/* t3jtag tap buffer functions */
static void t3jtag_tap_init(void);
static int t3jtag_tap_execute(void);
static void t3jtag_tap_ensure_space(int scans, int bits);
static void t3jtag_tap_append_step(int tms, int tdi);
static void t3jtag_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command);

static struct t3jtag_jtag *t3jtag_usb_open(void);
static void t3jtag_usb_close(struct t3jtag_jtag *t3jtag_jtag);
static int t3jtag_usb_message(struct t3jtag_jtag *t3jtag_jtag, int out_length, int in_length);
static int t3jtag_usb_write(struct t3jtag_jtag *t3jtag_jtag, int out_length);
static int t3jtag_usb_read(struct t3jtag_jtag *t3jtag_jtag);


#ifdef _DEBUG_USB_COMMS_
char time_str[50];
static void t3jtag_debug_buffer(uint8_t *buffer, int length);
char *t3jtag_get_time(char *);
#endif

static struct t3jtag_jtag *t3jtag_jtag_handle;
/***************************************************************************/

/* Queue command implementations */

void t3jtag_end_state(tap_state_t state) {
        if(tap_is_state_stable(state))
                tap_set_end_state(state);
        else {
                LOG_ERROR("BUG: %i is not a valid end state", state);
                exit(-1);
        }
}

/* Goes to the end state. */
void t3jtag_state_move(void) {
        int i;
        int tms = 0;
        uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
        uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

        for(i = 0; i < tms_scan_bits; i++) {
                tms = (tms_scan >> i) & 1;
                t3jtag_tap_append_step(tms, 0);
        }

        tap_set_state(tap_get_end_state());
}

void t3jtag_path_move(int num_states, tap_state_t *path) {
        int i;

        for(i = 0; i < num_states; i++) {
                if(path[i] == tap_state_transition(tap_get_state(), false))
                        t3jtag_tap_append_step(0, 0);
                else if(path[i] == tap_state_transition(tap_get_state(), true))
                        t3jtag_tap_append_step(1, 0);
                else {
                        LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
                                tap_state_name(tap_get_state()), tap_state_name(path[i]));
                        exit(-1);
                }

                tap_set_state(path[i]);
        }

        tap_set_end_state(tap_get_state());
}

void t3jtag_runtest(int num_cycles) {
        int i;

        tap_state_t saved_end_state = tap_get_end_state();

        /* only do a state_move when we're not already in IDLE */
        if(tap_get_state() != TAP_IDLE) {
                t3jtag_end_state(TAP_IDLE);
                t3jtag_state_move();
        }

        /* execute num_cycles */
        for(i = 0; i < num_cycles; i++)
                t3jtag_tap_append_step(0, 0);

        /* finish in end_state */
        t3jtag_end_state(saved_end_state);
        if(tap_get_state() != tap_get_end_state())
                t3jtag_state_move();
}

void t3jtag_scan(int ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command) {
        tap_state_t saved_end_state;

        t3jtag_tap_ensure_space(1, scan_size + 8);

        saved_end_state = tap_get_end_state();

        /* Move to appropriate scan state */
        t3jtag_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

        if(tap_get_state() != tap_get_end_state())
                t3jtag_state_move();

        t3jtag_end_state(saved_end_state);

        /* Scan */
        t3jtag_tap_append_scan(scan_size, buffer, command);

        /* We are in Exit1, go to Pause */
        t3jtag_tap_append_step(0, 0);

        tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

        if(tap_get_state() != tap_get_end_state())
                t3jtag_state_move();
}

void t3jtag_reset(int trst, int srst) {
        LOG_DEBUG("trst: %i, srst: %i", trst, srst);

        /* Signals are active low */
#if 0
        if(srst == 0)
                t3jtag_simple_command(JTAG_CMD_SET_SRST, 1);
        else if(srst == 1)
                t3jtag_simple_command(JTAG_CMD_SET_SRST, 0);

        if(trst == 0)
                t3jtag_simple_command(JTAG_CMD_SET_TRST, 1);
        else if(trst == 1)
                t3jtag_simple_command(JTAG_CMD_SET_TRST, 0);
#endif

        srst = srst ? 0 : 1;
        trst = trst ? 0 : 2;
        t3jtag_simple_command(JTAG_CMD_SET_SRST_TRST, srst | trst);
}

void t3jtag_simple_command(uint8_t command, uint8_t _data) {
        int result;

        DEBUG_JTAG_IO("0x%02x 0x%02x", command, _data);

        usb_out_buffer[0] = 2;
        usb_out_buffer[1] = 0;
        usb_out_buffer[2] = command;
        usb_out_buffer[3] = _data;

        result = t3jtag_usb_message(t3jtag_jtag_handle, 4, 1);
        if(result != 1)
                LOG_ERROR("t3jtag command 0x%02x failed (%d)", command, result);
}

//int t3jtag_get_status(void)
//{
//	return ERROR_OK;
//}

//int t3jtag_get_version_info(void)
//{
//	return ERROR_OK;
//}

/***************************************************************************/
/* Estick tap functions */

static int tap_length;
static uint8_t tms_buffer[T3JTAG_TAP_BUFFER_SIZE];
static uint8_t tdo_buffer[T3JTAG_TAP_BUFFER_SIZE];

void t3jtag_tap_init(void) {
        tap_length = 0;
        pending_scan_results_length = 0;
}

void t3jtag_tap_ensure_space(int scans, int bits) {
        int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
        int available_bits = T3JTAG_TAP_BUFFER_SIZE / 2 - tap_length;

        if((scans > available_scans) || (bits > available_bits))
                t3jtag_tap_execute();
}

void t3jtag_tap_append_step(int tms, int tdi) {
        unsigned char _tms = tms ? 1 : 0;
        unsigned char _tdi = tdi ? 1 : 0;

        t3jtag_tap_ensure_space(0, 1);

        int tap_index = tap_length / 4;
        int bits = (tap_length % 4) * 2;

        if(tap_length < T3JTAG_TAP_BUFFER_SIZE) {
                if(!bits)
                        tms_buffer[tap_index] = 0;

                tms_buffer[tap_index] |= (_tdi << bits) | (_tms << (bits + 1));
                tap_length++;
        } else
                LOG_ERROR("t3jtag_tap_append_step, overflow");
}

void t3jtag_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command) {
        DEBUG_JTAG_IO("append scan, length = %d", length);

        struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
        int i;

        pending_scan_result->first = tap_length;
        pending_scan_result->length = length;
        pending_scan_result->command = command;
        pending_scan_result->buffer = buffer;

        for(i = 0; i < length; i++)
                t3jtag_tap_append_step((i < length - 1 ? 0 : 1), (buffer[i / 8] >> (i % 8)) & 1);
        pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
int t3jtag_tap_execute(void) {
        int byte_length;
        int i, j;
        int result;

#ifdef _DEBUG_USB_COMMS_
        int byte_length_out;
#endif
        if(tap_length > 0) {

                /* memset(tdo_buffer,0,T3JTAG_TAP_BUFFER_SIZE); */
                /* LOG_INFO("T3JTAG tap execute %d",tap_length); */
                byte_length = (tap_length + 3) / 4;

#ifdef _DEBUG_USB_COMMS_
                byte_length_out = (tap_length + 7) / 8;
                LOG_DEBUG("t3jtag is sending %d bytes", byte_length);
#endif

                for(j = 0, i = 0; j < byte_length;) {

                        int receive;
                        int transmit = byte_length - j;
                        if(transmit > T3JTAG_MAX_TAP_TRANSMIT) {
                                transmit = T3JTAG_MAX_TAP_TRANSMIT;
                                receive = (T3JTAG_MAX_TAP_TRANSMIT) / 2;
                                usb_out_buffer[2] = JTAG_CMD_TAP_OUTPUT;
                        } else {
                                usb_out_buffer[2] = JTAG_CMD_TAP_OUTPUT | ((tap_length % 4) << 4);
                                receive = (transmit + 1) / 2;
                        }
                        usb_out_buffer[0] = (transmit + 1) & 0xff;
                        usb_out_buffer[1] = ((transmit + 1) >> 8) & 0xff;

                        memmove(usb_out_buffer + 3, tms_buffer + j, transmit);
                        result = t3jtag_usb_message(t3jtag_jtag_handle, 3 + transmit, receive);
                        if(result != receive) {
                                LOG_ERROR("t3jtag_tap_execute, wrong result %d, expected %d", result, receive);
                                return ERROR_JTAG_QUEUE_FAILED;
                        }

                        memmove(tdo_buffer + i, usb_in_buffer, receive);
                        i += receive;
                        j += transmit;
                }

#ifdef _DEBUG_USB_COMMS_
                LOG_DEBUG("t3jtag tap result %d", byte_length_out);
                t3jtag_debug_buffer(tdo_buffer, byte_length_out);
#endif

                /* LOG_INFO("eStick tap execute %d",tap_length); */
                for(i = 0; i < pending_scan_results_length; i++) {

                        struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[i];
                        uint8_t *buffer = pending_scan_result->buffer;
                        int length = pending_scan_result->length;
                        int first = pending_scan_result->first - 1; // minus 1, because it starts counting at zero!
                        struct scan_command *command = pending_scan_result->command;
                        assert(0 < first);
                        DEBUG_JTAG_IO("first bit to copy %d", first);
                        /* Copy to buffer */
                        buf_set_buf(tdo_buffer, first, buffer, 0, length);

                        DEBUG_JTAG_IO("pending scan result, length = %d", length);

#ifdef _DEBUG_USB_COMMS_
                        t3jtag_debug_buffer(buffer, byte_length_out);
#endif

                        if(jtag_read_buffer(buffer, command) != ERROR_OK) {
                                t3jtag_tap_init();
                                return ERROR_JTAG_QUEUE_FAILED;
                        }

                        if(pending_scan_result->buffer != NULL)
                                free(pending_scan_result->buffer);
                }

                t3jtag_tap_init();
        }

        return ERROR_OK;
}

/*****************************************************************************/

/* Estick USB low-level functions */

struct t3jtag_jtag *t3jtag_usb_open(void) {
        struct t3jtag_jtag *result;

        struct jtag_libusb_device_handle *devh;
        if(jtag_libusb_open(t3jtag_probe->VID, t3jtag_probe->PID, NULL, &devh) != ERROR_OK)
                return NULL;

        jtag_libusb_set_configuration(devh, 0);
        jtag_libusb_claim_interface(devh, 0);

        //	result = (struct t3jtag_jtag *) malloc(sizeof(struct t3jtag_jtag));

        result = malloc(sizeof(*result));
        result->usb_handle = devh;
        return result;
}

void t3jtag_usb_close(struct t3jtag_jtag *t3jtag_jtag) {
        jtag_libusb_close(t3jtag_jtag->usb_handle);
        free(t3jtag_jtag);
}

/* Send a message and receive the reply. */
int t3jtag_usb_message(struct t3jtag_jtag *t3jtag_jtag, int out_length, int in_length) {
        int result;

        result = t3jtag_usb_write(t3jtag_jtag, out_length);
        if(result == out_length) {
                result = t3jtag_usb_read(t3jtag_jtag);
                if(result == in_length)
                        return result;
                else {
                        LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)", in_length, result);
                        return -1;
                }
        } else {
                LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)", out_length, result);
                return -1;
        }
}

/* Write data from out_buffer to USB. */
int t3jtag_usb_write(struct t3jtag_jtag *t3jtag_jtag, int out_length) {
        int result;

        if(out_length > T3JTAG_OUT_BUFFER_SIZE) {
                LOG_ERROR("t3jtag_jtag_write illegal out_length=%d (max=%d)", out_length, T3JTAG_OUT_BUFFER_SIZE);
                return -1;
        }

#ifdef _DEBUG_USB_COMMS_
        LOG_DEBUG("%s: USB write begin", t3jtag_get_time(time_str));
#endif
        if(t3jtag_probe->CONTROL_TRANSFER) {
                result = jtag_libusb_control_transfer(t3jtag_jtag->usb_handle,
                        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT,
                        FUNC_WRITE_DATA, 0, 0, (char *) usb_out_buffer, out_length, good ? T3JTAG_USB_TIMEOUT: 200);
        } else {
                result = jtag_libusb_bulk_write(t3jtag_jtag->usb_handle, T3JTAG_WRITE_ENDPOINT,
			(char *) usb_out_buffer, out_length, good ? T3JTAG_USB_TIMEOUT : 200);
        }
#ifdef _DEBUG_USB_COMMS_
        LOG_DEBUG("%s: USB write end: %d bytes", t3jtag_get_time(time_str), result);
#endif

        DEBUG_JTAG_IO("t3jtag_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
        t3jtag_debug_buffer(usb_out_buffer, out_length);
#endif
        return result;
}

/* Read data from USB into in_buffer. */
int t3jtag_usb_read(struct t3jtag_jtag *t3jtag_jtag) {
#ifdef _DEBUG_USB_COMMS_
        LOG_DEBUG("%s: USB read begin", t3jtag_get_time(time_str));
#endif
        int result;
        if(t3jtag_probe->CONTROL_TRANSFER) {
                result = jtag_libusb_control_transfer(t3jtag_jtag->usb_handle,
                        LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
                        FUNC_READ_DATA, 0, 0, (char *) usb_in_buffer, T3JTAG_IN_BUFFER_SIZE, good ? T3JTAG_USB_TIMEOUT : 200);
        } else {
                result = jtag_libusb_bulk_read(t3jtag_jtag->usb_handle, T3JTAG_READ_ENDPOINT,
                        (char *) usb_in_buffer, T3JTAG_IN_BUFFER_SIZE, good ? T3JTAG_USB_TIMEOUT : 200);
        }
#ifdef _DEBUG_USB_COMMS_
        LOG_DEBUG("%s: USB read end: %d bytes", t3jtag_get_time(time_str), result);
#endif
        DEBUG_JTAG_IO("t3jtag_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
        t3jtag_debug_buffer(usb_in_buffer, result);
#endif
        return result;
}

#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

void t3jtag_debug_buffer(uint8_t *buffer, int length) {
        char line[81];
        char s[4];
        int i;
        int j;

        for(i = 0; i < length; i += BYTES_PER_LINE) {
                snprintf(line, 5, "%04x", i);
                for(j = i; j < i + BYTES_PER_LINE && j < length; j++) {
                        snprintf(s, 4, " %02x", buffer[j]);
                        strcat(line, s);
                }
                LOG_DEBUG("%s", line);
        }
}

char *t3jtag_get_time(char *str) {
        struct timeb timebuffer;
        char *timeline;

        ftime(&timebuffer);
        timeline = ctime(&(timebuffer.time));
        snprintf(str, 49, "%.8s.%hu", &timeline[11], timebuffer.millitm);
        return str;
}
#endif
/***************************************************************************/

/* External interface implementation */


static int t3jtag_init(void) {
        //int check_cnt;
        struct t3jtag_probe *cur_t3jtag_probe;

        cur_t3jtag_probe = t3jtag_probes;
        good = 0;

        if(t3jtag_type == NULL) {
                t3jtag_type = strdup("t3jtag");
                LOG_WARNING("No t3jtag_type specified, using default 't3jtag'");
        }

        while(cur_t3jtag_probe->name) {
                if(strcmp(cur_t3jtag_probe->name, t3jtag_type) == 0) {
                        t3jtag_probe = cur_t3jtag_probe;
                        break;
                }
                cur_t3jtag_probe++;
        }

        if(!t3jtag_probe) {
                LOG_ERROR("No matching cable found for %s", t3jtag_type);
                return ERROR_JTAG_INIT_FAILED;
        }


        usb_in_buffer = malloc(t3jtag_probe->BUFFERSIZE);
        usb_out_buffer = malloc(t3jtag_probe->BUFFERSIZE);

        //pending_scan_results_buffer = (struct pending_scan_result *)
        //		malloc(MAX_PENDING_SCAN_RESULTS * sizeof(struct pending_scan_result));

        pending_scan_results_buffer = malloc(
                MAX_PENDING_SCAN_RESULTS * sizeof(*pending_scan_results_buffer));

        t3jtag_jtag_handle = t3jtag_usb_open();

        if(t3jtag_jtag_handle == 0) {
                LOG_ERROR("Cannot find t3jtag Interface! Please check connection and permissions.");
                return ERROR_JTAG_INIT_FAILED;
        }

        // Drain any data by doing reads a couple times, and ignore fails
        // We don't even care what is/was there.
        usleep(1000);
        t3jtag_reset(1, 1);
        t3jtag_reset(1, 1);
        t3jtag_reset(1, 1);
        t3jtag_reset(1, 1);
        t3jtag_reset(1, 1);
        LOG_INFO("t3jtag Draining JTAG Interface");
        int i;
        for(i = 0; i < 5; i++) {
                if(t3jtag_probe->CONTROL_TRANSFER) {
                        jtag_libusb_control_transfer(t3jtag_jtag_handle->usb_handle,
                                LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
                                FUNC_READ_DATA, 0, 0, (char *) usb_in_buffer, T3JTAG_IN_BUFFER_SIZE, 10);
                } else {
                        jtag_libusb_bulk_read(t3jtag_jtag_handle->usb_handle, T3JTAG_READ_ENDPOINT,
                                (char *) usb_in_buffer, T3JTAG_IN_BUFFER_SIZE, 10);
                }
        }

        //t3jtag_usb_read(t3jtag_jtag);
        // Force reset
        t3jtag_reset(1, 1);
        usleep(30);
        t3jtag_reset(0, 0);
        usleep(30);
        good = 1;

        t3jtag_tap_init();
        LOG_INFO("t3jtag JTAG Interface ready");

        return ERROR_OK;
}

static int t3jtag_quit(void) {
        t3jtag_usb_close(t3jtag_jtag_handle);

        if(usb_out_buffer) {
                free(usb_out_buffer);
                usb_out_buffer = NULL;
        }

        if(usb_in_buffer) {
                free(usb_in_buffer);
                usb_in_buffer = NULL;
        }

        if(pending_scan_results_buffer) {
                free(pending_scan_results_buffer);
                pending_scan_results_buffer = NULL;
        }

        if(t3jtag_type) {
                free(t3jtag_type);
                t3jtag_type = NULL;
        }

        return ERROR_OK;
}

static int t3jtag_execute_queue(void) {
        struct jtag_command *cmd = jtag_command_queue;
        int scan_size;
        enum scan_type type;
        uint8_t *buffer;

        while(cmd != NULL) {
                switch(cmd->type) {
                        case JTAG_RUNTEST:
                                DEBUG_JTAG_IO("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, \
					cmd->cmd.runtest->end_state);

                                if(cmd->cmd.runtest->end_state != -1)
                                        t3jtag_end_state(cmd->cmd.runtest->end_state);
                                t3jtag_runtest(cmd->cmd.runtest->num_cycles);
                                break;

                        case JTAG_TLR_RESET:
                                DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

                                if(cmd->cmd.statemove->end_state != -1)
                                        t3jtag_end_state(cmd->cmd.statemove->end_state);
                                t3jtag_state_move();
                                break;

                        case JTAG_PATHMOVE:
                                DEBUG_JTAG_IO("pathmove: %i states, end in %i", \
					cmd->cmd.pathmove->num_states, \
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

                                t3jtag_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
                                break;

                        case JTAG_SCAN:
                                DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

                                if(cmd->cmd.scan->end_state != -1)
                                        t3jtag_end_state(cmd->cmd.scan->end_state);

                                scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
                                DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
                                t3jtag_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
                                type = jtag_scan_type(cmd->cmd.scan);
                                t3jtag_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size, cmd->cmd.scan);
                                break;

                        case JTAG_RESET:
                                DEBUG_JTAG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

                                t3jtag_tap_execute();

                                if(cmd->cmd.reset->trst == 1)
                                        tap_set_state(TAP_RESET);
                                t3jtag_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
                                break;

                        case JTAG_SLEEP:
                                DEBUG_JTAG_IO("sleep %" PRIi32, cmd->cmd.sleep->us);
                                t3jtag_tap_execute();
                                jtag_sleep(cmd->cmd.sleep->us);
                                break;

                        default:
                                LOG_ERROR("BUG: unknown JTAG command type encountered");
                                exit(-1);
                }
                cmd = cmd->next;
        }
        return t3jtag_tap_execute();
}

COMMAND_HANDLER(t3jtag_handle_t3jtag_type_command) {
        if (CMD_ARGC > 1) return ERROR_COMMAND_SYNTAX_ERROR;
        if (CMD_ARGC == 0) return ERROR_OK;
        /* only if the cable name wasn't overwritten by cmdline */
        if (t3jtag_type == NULL) t3jtag_type = strdup(CMD_ARGV[0]);
        return ERROR_OK;
}

COMMAND_HANDLER(t3jtag_handle_unwedge_command){
        switch(CMD_ARGC) {
                case 0:
                        t3jtag_simple_command(JTAG_CMD_KINETIS_ERASE, 0x00);
                        break;
                default:
                        return ERROR_COMMAND_SYNTAX_ERROR;
        }
        return ERROR_OK;
}


//COMMAND_HANDLER(t3jtag_handle_t3jtag_info_command)
//{
//	if (t3jtag_get_version_info() == ERROR_OK) {
//		/* attempt to get status */
//		t3jtag_get_status();
//	}
//
//	return ERROR_OK;
//}

COMMAND_HANDLER(t3jtag_handle_t3jtag_set_microsec_command) {
        int speed = 0;
        switch(CMD_ARGC) {
                case 1:
                        speed = atoi(CMD_ARGV[0]);
                        if(speed < 0 || speed > 255) return ERROR_COMMAND_SYNTAX_ERROR;
                        t3jtag_simple_command(JTAG_CMD_SET_DELAY, speed & 0xff);
                        break;
                default:
                        return ERROR_COMMAND_SYNTAX_ERROR;
        }

        return ERROR_OK;
}

static const struct command_registration t3jtag_command_handlers[] = {
        {
                .name = "unwedge",
                .handler = &t3jtag_handle_unwedge_command,
                .mode = COMMAND_EXEC,
                .help = "Performs a mass erase on Freescale devices via SPI/EZport.",
                .usage = "",
        },
        {
                .name = "t3jtag_set_microsec",
                .handler = &t3jtag_handle_t3jtag_set_microsec_command,
                .mode = COMMAND_EXEC,
                .help = "t3jtag_set_microsec sets how many microseconds to delay after TCLK goes idle before reading the results.",
                .usage = "0-255",
        },
        {
                // currently does nothing ;-)
                .name = "t3jtag_type",
                .handler = &t3jtag_handle_t3jtag_type_command,
                .mode = COMMAND_CONFIG,
                .help = "set t3jtag type",
                .usage = "[t3jtag]",
        },
        COMMAND_REGISTRATION_DONE
};

struct jtag_interface t3jtag_interface = {
        .name = "t3jtag",
        .commands = t3jtag_command_handlers,
        .transports = jtag_only,
        //.transports = t3jtag_transports,
        .execute_queue = t3jtag_execute_queue,
        .init = t3jtag_init,
        .quit = t3jtag_quit,
        .supported = 0,  //DEBUG_CAP_TMS_SEQ
        // .khz =
        // .speed =
        // .speed_div =
        // .swd = &bitbang_swd,
        // .swdio_read = t3jtag_swdio_read,
        // .swdio_drive = t3jtag_swdio_drive,
        // .blink = 0
};

