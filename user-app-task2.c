/*
 ============================================================================
 Name        : ESP-Lab3.c
 Author      : Rohit Khanna
 Version     :
 Copyright   : Your copyright notice
 Description : user-app for ESP Lab3

  	15 address space = 32KB, Page size = 64 B therefore we have 32KB/64B pages = 2^9 = 512 pages
  	 global pointer to maintain the current page position of EEPROM where it can write.
  	 Will vary from 0 to 511

 ============================================================================
 */


#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

#define I2C_SLAVE	0x0703	/* Use this slave address  for ioctl*/

int FILE_DESC = 0;
uint PAGE_SIZE = 64;



/*
 * read count*PAGE_SIZE bytes in buffer buf
 * returns no of bytes read
 */
int read_EEPROM(void *buf, int count){

	printf("APP :  read_EEPROM()\n");
	int res;
	unsigned char *buffer = malloc(count*PAGE_SIZE);

	if ((res=read(FILE_DESC, buffer, count)) != count*PAGE_SIZE) {
		fprintf(stderr,"APP : Error: Failed to read from the i2c bus: %s, res=%d\n",strerror(errno), res);
		return -1;
	}

	memcpy(buf, buffer, count*PAGE_SIZE);
	return res;
}



/*
 * int seek_EEPROM(int offset): to set the current page position in the EEPROM to offset which is
 * the page number in the range of 0 to 2k-1. Returns the page_number seeked to, or
 * –1 otherwise.
 */

int seek_EEPROM(int offset){				//offset is page number
	int page_number = offset;
	printf("APP: user seek_EEPROM()\n");
	int res;

	if ((res=lseek(FILE_DESC, page_number, SEEK_SET)) != page_number) {
		fprintf(stderr,"APP: Error: seek_EEPROM()- Failed to write to the i2c bus: %s, res=%d\n",strerror(errno), res);
		return -1;
	}
	printf("APP: seek successful at address=%d\n", res);
	return res;
}


/*
 * to write a sequence of count pages pointed by buffer buf to /dev/i2c-flash
 * Return no of bytes written, or –1 otherwise
 */
int write_EEPROM(const void *buf, int count){
	printf("user write_EEPROM()\n");
	int res=0;

	if ( (res=write(FILE_DESC,buf,count))  != count*PAGE_SIZE ) {
		fprintf(stderr,"APP: Error: Failed to write to the i2c bus: %s, res=%d\n",strerror(errno), res);
		return -1;
	}
	printf("APP : write_EEPROM successful, bytes written = res=%d\n",res);

	return res;
}



int main(){

	int i,k;
	int page_count=15;
	char wr_buffer[(page_count*PAGE_SIZE)];
	char rd_buffer[page_count*PAGE_SIZE];

	if ((FILE_DESC = open("/dev/i2c-flash",O_RDWR)) < 0) {			// not fopen so that writes to the bus are not buffered.
		fprintf(stderr,"APP: Error: Failed to open the bus: %s\n",strerror(errno));
		exit(1);
	}
	printf("APP: File opened successfully\n");

	for(i=0; i<(page_count*PAGE_SIZE); i++){						// Populate the buffer to write
		wr_buffer[i]='r';
	}

	/*	SEEK to page number	*/
	if(seek_EEPROM(500) ==-1)											//
		return -1;

	/*	WRITE	*/
	if(write_EEPROM(wr_buffer, page_count) == -1)
		return -1;

	/*	SEEK	*/
	if(seek_EEPROM(500) ==-1)											//
			return -1;

	/*	READ	*/
	if(read_EEPROM(rd_buffer, page_count) != -1){
		printf("APP : Data read from EEPROM -->\n");
		for(k=0; k<page_count*PAGE_SIZE; k++){
			printf("%c", rd_buffer[k]);
		}
		printf("\n");
	}


	return 0;
}
