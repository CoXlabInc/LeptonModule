/*
Copyright (c) 2014, Pure Engineering LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, 
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>
#include <string.h>
#include "LEPTON_SDK.h"
#include "LEPTON_SYS.h"
#include "LEPTON_RAD.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 16000000;


#define VOSPI_FRAME_SIZE_BYTES (164)
#define VOSPI_FRAME_SIZE_UINT16 (VOSPI_FRAME_SIZE_BYTES / 2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (VOSPI_FRAME_SIZE_UINT16 * PACKETS_PER_FRAME)

static uint8_t shelf[4][VOSPI_FRAME_SIZE_BYTES * PACKETS_PER_FRAME];
static int lepton_width = 160;
static int lepton_height = 120;
static int lepton_type = 3; // 2 for Lepton 2.x, 3 for Lepton 3.x



// Max size for lepton_image to accommodate Lepton 3.x (120x160)
static unsigned int lepton_image[120][160];

// Add function prototype for parsing arguments
static void parse_args(int argc, char *argv[]);

static void usage(const char *argv0)
{
	fprintf(stderr, "Usage: %s [-t <lepton_type>]\n", argv0);
	fprintf(stderr, "  -t <lepton_type>: Specify Lepton type (2 for 2.x, 3 for 3.x). Default is 2.\n");
	exit(EXIT_FAILURE);
}

static void parse_args(int argc, char *argv[])
{
	int opt;
	while ((opt = getopt(argc, argv, "t:")) != -1) {
		switch (opt) {
		case 't':
			lepton_type = atoi(optarg);
			if (lepton_type != 2 && lepton_type != 3) {
				fprintf(stderr, "Invalid Lepton type: %s. Must be 2 or 3.\n", optarg);
				usage(argv[0]);
			}
			break;
		default:
			usage(argv[0]);
			break;
		}
	}
}

static void save_pgm_file(void)
{
	int i;
	int j;
	unsigned int maxval = 0;
	unsigned int minval = UINT_MAX;
	char image_name[32];
	char csv_name[32];
	int image_index = 0;

	do {
		sprintf(image_name, "IMG_%.4d.pgm", image_index);
		image_index += 1;
		if (image_index > 9999) 
		{
			image_index = 0;
			break;
		}

	} while (access(image_name, F_OK) == 0);

	sprintf(csv_name, "IMG_%.4d.csv", image_index - 1);

	FILE *f = fopen(image_name, "w");
	if (f == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}

	FILE *f_csv = fopen(csv_name, "w");
	if (f_csv == NULL)
	{
		printf("Error opening CSV file!\n");
	}

	printf("Calculating min/max values for proper scaling...\n");
	for(i=0;i<lepton_height;i++)
	{
		for(j=0;j<lepton_width;j++)
		{
			if (lepton_image[i][j] > maxval) {
				maxval = lepton_image[i][j];
			}
			if (lepton_image[i][j] < minval) {
				minval = lepton_image[i][j];
			}
		}
	}
	printf("maxval = %u\n",maxval);
	printf("minval = %u\n",minval);
	
	fprintf(f,"P2\n%d %d\n%u\n", lepton_width, lepton_height, maxval-minval);
	for(i=0;i<lepton_height;i++)
	{
		for(j=0;j<lepton_width;j++)
		{
			fprintf(f,"%d ", lepton_image[i][j] - minval); 
			
			if (f_csv) {
				float temp = (lepton_image[i][j] - 27315) / 100.0f;
				fprintf(f_csv, "%.2f%s", temp, (j == lepton_width - 1) ? "" : ",");
			}
		}
		fprintf(f,"\n");
		if (f_csv) fprintf(f_csv, "\n");
	}
	fprintf(f,"\n\n");

	fclose(f);
	if (f_csv) fclose(f_csv);
}

// This function is now responsible for populating the lepton_image from the shelf buffer
void process_lepton_frame()
{
	int i;
	unsigned short value;
	int iSegmentStart = 0;
	int iSegmentStop = (lepton_type == 3) ? 3 : 0;

	for (int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
		for (i = 0; i < FRAME_SIZE_UINT16; i++) {
			if (i % VOSPI_FRAME_SIZE_UINT16 < 2) {
				continue;
			}
			value = (shelf[iSegment][i * 2] << 8) + shelf[iSegment][i * 2 + 1];
			if (value == 0) {
				continue;
			}

			if (lepton_type == 3) {
				int row = i / VOSPI_FRAME_SIZE_UINT16 / 2 + (30 * iSegment);
				int col = (i % VOSPI_FRAME_SIZE_UINT16) - 2 + (80 * ((i % (VOSPI_FRAME_SIZE_UINT16 * 2)) / VOSPI_FRAME_SIZE_UINT16));
				lepton_image[row][col] = value;
			} else {
				int row = i / VOSPI_FRAME_SIZE_UINT16;
				int col = (i % VOSPI_FRAME_SIZE_UINT16) - 2;
				lepton_image[row][col] = value;
			}
		}
	}
}


int capture_one_frame(int fd, int* segment_number)
{
	uint8_t local_frame_buffer[VOSPI_FRAME_SIZE_BYTES * PACKETS_PER_FRAME];
	int resets = 0;
	int packetNumber;

	for(int j=0; j<PACKETS_PER_FRAME; j++) {
		// Attempt to read one packet
		if (read(fd, local_frame_buffer + j * VOSPI_FRAME_SIZE_BYTES, VOSPI_FRAME_SIZE_BYTES) != VOSPI_FRAME_SIZE_BYTES) {
			// This could indicate a read error or short read, treat as a reset.
			j = -1; // Force restart frame capture
			resets++;
			usleep(1000);
			if (resets > 750) {
				printf("Camera reset: Too many read errors.\n");
				return -1; // Critical error
			}
			continue;
		}

		packetNumber = local_frame_buffer[j * VOSPI_FRAME_SIZE_BYTES + 1];

		// Check for discard packet (first nibble is 0x0F)
		if ((local_frame_buffer[j * VOSPI_FRAME_SIZE_BYTES + 0] & 0x0f) == 0x0f) {
			j = -1; // Discard packet, restart frame capture
			resets++;
			usleep(1000);
			if (resets > 750) {
				printf("Camera reset: Too many discard packets.\n");
				return -1; // Critical error
			}
			continue;
		}
		resets = 0; // Reset consecutive resets count if valid packet found

		// Check for packet number mismatch (sync loss)
		if (packetNumber != j) {
			j = -1; // Mismatch, restart frame capture
			usleep(1000);
			continue;
		}

		// If Lepton 3.x, extract segment number from packet 20
		if (lepton_type == 3 && packetNumber == 20) {
			*segment_number = (local_frame_buffer[j * VOSPI_FRAME_SIZE_BYTES + 0] >> 4) & 0x0f;
			if (*segment_number < 1 || *segment_number > 4) {
				printf("Invalid segment number (%d), restarting frame.\n", *segment_number);
				j = -1; // Invalid segment, restart frame capture
				continue;
			}
		}
	}

	// After successfully capturing all 60 packets for a frame
	// Copy the local_frame_buffer to the appropriate shelf segment
	if (lepton_type == 3) {
		if (*segment_number > 0) { // Should be set by packet 20
			memcpy(shelf[*segment_number - 1], local_frame_buffer, VOSPI_FRAME_SIZE_BYTES * PACKETS_PER_FRAME);
		} else {
			// Lepton 3.x frame completed without a segment number. This indicates an issue.
			printf("Lepton 3.x frame captured without segment number. Retrying.\n");
			return -1;
		}
	} else { // Lepton 2.x
		memcpy(shelf[0], local_frame_buffer, VOSPI_FRAME_SIZE_BYTES * PACKETS_PER_FRAME);
		*segment_number = 1; // Explicitly set for Lepton 2.x processing
	}
	
	return 0; // Success
}


int main(int argc, char *argv[])
{
	parse_args(argc, argv); // Parse command line arguments

	if (lepton_type == 2) { // Only change to 80x60 if -t 2 is specified
		lepton_width = 80;
		lepton_height = 60;
		printf("Lepton 2.x selected, resolution: %dx%d\n", lepton_width, lepton_height);
	} else { // Default to Lepton 3.x
		lepton_width = 160;
		lepton_height = 120;
		printf("Lepton 3.x selected, resolution: %dx%d\n", lepton_width, lepton_height);
	}

	int ret = 0;
	int fd;


	fd = open(device, O_RDWR);
	if (fd < 0)
	{
		pabort("can't open device");
	}

	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't set spi mode");
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
	{
		pabort("can't get spi mode");
	}

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't set bits per word");
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		pabort("can't get bits per word");
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't set max speed hz");
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't get max speed hz");
	}

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	LEP_CAMERA_PORT_DESC_T port_desc;
	LEP_RESULT lres = LEP_OpenPort(1, LEP_CCI_TWI, 400, &port_desc);
	if (lres != LEP_OK) {
		printf("I2C port open failed: %d\n", lres);
	} else {
		printf("Enabling T-Linear Radiometry mode...\n");
		lres = LEP_SetRadTLinearEnableState(&port_desc, LEP_RAD_ENABLE);
		if (lres != LEP_OK) {
			printf("Failed to enable T-Linear mode: %d. (This is expected for non-radiometric Leptons)\n", lres);
		} else {
			printf("T-Linear mode enabled.\n");
		}

		printf("Performing FFC...\n");
		lres = LEP_RunSysFFCNormalization(&port_desc);
		if (lres != LEP_OK) {
			printf("FFC command failed: %d\n", lres);
		} else {
			LEP_SYS_STATUS_E ffc_status;
			int timeout_ms = 2000; // 2 second timeout
			do {
				lres = LEP_GetSysFFCStatus(&port_desc, &ffc_status);
				if (lres == LEP_OK) {
					usleep(10000); // 10ms
					timeout_ms -= 10;
				} else {
					break; // Break on communication error
				}
			} while (lres == LEP_OK && ffc_status == LEP_SYS_STATUS_BUSY && timeout_ms > 0);

			if (timeout_ms <= 0) {
				printf("FFC status polling timed out.\n");
			} else if (lres == LEP_OK) {
				printf("FFC complete.\n");
			} else {
				printf("Error polling FFC status.\n");
			}
		}
	}

	if (lepton_type == 3) {
		int segments_captured[4] = {0};
		int segment_count = 0;
		int segment_number = 0;
		while (segment_count < 4) {
			if (capture_one_frame(fd, &segment_number) == 0) {
				if (segment_number > 0 && segments_captured[segment_number - 1] == 0) {
					segments_captured[segment_number - 1] = 1;
					segment_count++;
					printf("Captured segment %d\n", segment_number);
				} else {
					// Frame captured, but segment_number not valid for unique segment (e.g., segment_number == 0)
					// This means we need to retry the entire frame acquisition for a valid segment
					usleep(10000); // Wait a bit before retrying
					segment_number = 0; // Reset segment_number to indicate needing a new valid segment
				}
			} else {
				// capture_one_frame failed, possibly due to sync loss or invalid data
				usleep(10000); // Wait a bit before retrying
				segment_number = 0; // Reset segment_number
			}
		}
	} else {
		int segment_number = 1; // Lepton 2.x is just one segment
		capture_one_frame(fd, &segment_number);
	}

	if (lres == LEP_OK) {
		LEP_ClosePort(&port_desc);
	}
	close(fd);

	process_lepton_frame();

	save_pgm_file();

	return ret;
}