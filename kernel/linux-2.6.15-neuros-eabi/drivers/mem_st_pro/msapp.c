/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <error.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

/* mknod /dev/ms -m 666 c 254 0 */

#define SDMMCDEVFS "/dev/mem_stkblk0"
#define CARD_TYPE 0xAA	
	
int main(){
	int status = 0;
        int sd_mmcdevfs;

        sd_mmcdevfs = open(SDMMCDEVFS,O_RDWR,0);

	if (sd_mmcdevfs < 0){
		printf("No Device");
		close (sd_mmcdevfs);
		return 0;
	}
	if (ioctl(sd_mmcdevfs,CARD_TYPE,(unsigned long)(&status)) == -1){
		printf("ioctl error\n");
	}
	if (status == 1)
		printf ("Memory stick Pro\n");
	else
	if (status == 2)
		printf ("Memory stick\n");
	else
		printf ("Unknown device\n");	
	close (sd_mmcdevfs);
	return 0;
}


