#include "../Header/timdebug.hpp"

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include "../Header/sysconfig.hpp"
#include "../Header/datatype.hpp"
#include "../Header/datastruct.hpp"
#include "../Header/lpr.hpp"

#ifdef ACTIVATE_OPENCV
#include "opencv/cv.h"
#include "opencv/highgui.h"
#endif

#ifdef ENVIRONMENT_BOARD
#include <memory.h>
#include <sys/mman.h>
void write_orig_to_shmem(volatile ushort* addr, IMAGE* pImage);
void write_lp_to_shmem(volatile uchar* addr, IMAGE* pImage);
void write_token_to_shmem(volatile uchar** table, IMAGE* pImage);
#endif

int read_img_bmp(char* filename, IMAGE* pImage);
void convert_img(IMAGE* pSrc, IMAGE* pDst);

uchar	_8bit_buff_1[52500];
uchar	_8bit_buff_2[52500];
uchar	_8bit_buff_3[52500];
int	_32bit_buff[52500];
int	_global_FrmOrigH;

int main(int argc, char** argv)
{
#ifdef ENVIRONMENT_BOARD
	volatile ushort		*orig_image_addr    = NULL;
   	volatile uchar		*plate_image_addr   = NULL;
   	volatile uchar		*text_image_addr[7] = {NULL};
   	void			*axi_virtual_base;
   	int			fd;
#endif
	DIR			*dir_info;
	struct dirent		*dir_entry;
	char			path[100];
	char			file[100];
	int			nframe = 0, nError = 0;
	time_t			start, end, start1, end1, start2, end2, mod1 = 0, mod2 = 0;

	LPRManager		lm;
	IMAGE			forig;
	IMAGE			fcvt;
	IMAGE			lp;
	IMAGE			tk;

#ifdef ENVIRONMENT_BOARD
	if((fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) < 0)
	{
		fprintf(stdout, "[error] main - can not open \"/dev/mem\"\n");
		return -1;
	}
	
	axi_virtual_base  = mmap(   NULL,
                                HW_FPGA_AXI_SPAN,
                                ( PROT_READ | PROT_WRITE ),
                                MAP_SHARED,
                                fd,
                                ALT_AXI_FPGASLVS_OFST
                            );

	if( axi_virtual_base == MAP_FAILED )
	{
		fprintf(stdout, "[error] main - axi mmap() failure.\n");
		close(fd);
		return -1;
	}
	close(fd);
	
	orig_image_addr = (ushort*)(axi_virtual_base + (ORIG_IMAGE_ADDR & HW_FPGA_AXI_MASK));
	plate_image_addr = (uchar*)(axi_virtual_base + (PLATE_IMAGE_ADDR & HW_FPGA_AXI_MASK));
	text_image_addr[0] = (uchar*)(axi_virtual_base + (TEXT_IMAGE0_ADDR & HW_FPGA_AXI_MASK));
	text_image_addr[1] = (uchar*)(axi_virtual_base + (TEXT_IMAGE1_ADDR & HW_FPGA_AXI_MASK));
	text_image_addr[2] = (uchar*)(axi_virtual_base + (TEXT_IMAGE2_ADDR & HW_FPGA_AXI_MASK));
	text_image_addr[3] = (uchar*)(axi_virtual_base + (TEXT_IMAGE3_ADDR & HW_FPGA_AXI_MASK));
	text_image_addr[4] = (uchar*)(axi_virtual_base + (TEXT_IMAGE4_ADDR & HW_FPGA_AXI_MASK));
	text_image_addr[5] = (uchar*)(axi_virtual_base + (TEXT_IMAGE5_ADDR & HW_FPGA_AXI_MASK));
	text_image_addr[6] = (uchar*)(axi_virtual_base + (TEXT_IMAGE6_ADDR & HW_FPGA_AXI_MASK));
#endif
	memset(path, 0, sizeof(path));
	strncpy(path, PATH_IMAGESET, strlen(PATH_IMAGESET));

	if((dir_info = opendir(path)) == NULL)
	{
        	fprintf(stdout, "[error] main - no such path >> %s\n", path);
		return -1;
	}

	_global_FrmOrigH = SYSCONFIG_FRAME_ORIGIN_HEIGHT;	
	
	forig.allocate(3,SYSCONFIG_FRAME_ORIGIN_WIDTH, SYSCONFIG_FRAME_ORIGIN_HEIGHT);
	
	fcvt.allocate(1,SYSCONFIG_FRAME_CONVERT_WIDTH, SYSCONFIG_FRAME_CONVERT_HEIGHT);

	lp.allocate(1,SYSCONFIG_LICENSEPLATE_WIDTH, SYSCONFIG_LICENSEPLATE_HEIGHT);

	tk.allocate(1,SYSCONFIG_TOKEN_WIDTH * 7, SYSCONFIG_TOKEN_HEIGHT);

	lm.allocate(
			fcvt.size, _32bit_buff,
			DATATYPE_VPM_DATA_MAXSIZE,
			BLOB_DATA_MAXSIZE,
			LPR_CM_DATA_MAXSIZE,
			LPR_TM_DATA_MAXSIZE,
			LPR_NE_DATA_MAXSIZE,
			LPR_EX_DATA_MAXSIZE,
			LPR_AN_CBUFF1_SIZE,
			LPR_AN_CBUFF2_SIZE
						);

	start = clock();

	while((dir_entry = readdir(dir_info)) != NULL)
	{
		if(!strstr(dir_entry->d_name, ".bmp")) continue;

		memset(file, 0x00, sizeof(file));
		strncpy(file, path, strlen(path));
		strncat(file, dir_entry->d_name, strlen(dir_entry->d_name));

		start1 = clock();
		if(read_img_bmp(file, &forig) < 0) continue;
		convert_img(&forig, &fcvt);
		end1 = clock();
		mod1 += (end1 - start1);
		
		nframe++;
	
#ifndef ENVIRONMENT_BOARD
		start2 = clock();
		if(lm.recognize(&fcvt, &lp, &tk) < 0)
		{
			nError++;
			continue;
		}
		end2 = clock();
		mod2 += (end2 - start2);

#ifdef ACTIVATE_OPENCV
		cv::Mat frame = cv::Mat(forig.size.h, forig.size.w, CV_8UC3);
		memcpy(frame.data, forig.data, sizeof(uchar) * 3 * forig.size.h * forig.size.w);
		cv::imshow("original", frame);
		cv::moveWindow("original", 1300, 400);
		cv::waitKey();
#endif
#endif

#ifdef ENVIRONMENT_BOARD
		write_orig_to_shmem(orig_image_addr, &forig);

		start2 = clock();
		if(lm.recognize(&fcvt, &lp, &tk) < 0)
		{
			nError++;
			continue;
		}
		end2 = clock();
		mod2 += (end2 - start2);

		write_lp_to_shmem(plate_image_addr, &lp);
		write_token_to_shmem(text_image_addr, &tk);

#ifndef	DELAY_ZERO
		getchar();
#endif
#endif
	}

	end = clock();
	fprintf(stdout, "module1 processing time : %f [%f per]\n", (float)mod1 / (float) CLOCKS_PER_SEC, (float)mod1 / (float)(end-start) * 100.0f );
	fprintf(stdout, "module2 processing time : %f [%f per]\n", (float)mod2 / (float) CLOCKS_PER_SEC, (float)mod2 / (float)(end-start) * 100.0f );
	fprintf(stdout, "total frame [%d] error [%d] => success rate [%f per]\n", nframe, nError, (float)(nframe - nError) / (float)nframe * 100.0f);
	
	forig.clear();
	fcvt.clear();
	lp.clear();
	tk.clear();
	lm.clear();

	return 0;
}

int read_img_bmp(char* filename, IMAGE* pImage)
{
    const int offset_bitmap_data = 10;
    const int offset_bitmap_width = 18;
    const int offset_bitmap_height = 22;

    int r;
    int fd;
    int offset;
    int width;
    int height;

    //get file discriptor
    if((fd = open(filename, O_RDONLY)) < 0)
    {
	    fprintf(stdout, "[%s] file upload failure.\n", filename);
	    return -1;
    }
    fprintf(stdout, "[%s] file upload complete.\n", filename);

    //Parse Header to check whether width is valid or not.
    lseek(fd, offset_bitmap_width, SEEK_SET);
    read(fd, &width, sizeof(int));
    assert(pImage->size.w == width);

    //Parse Header to check whether height is valid or not.
    lseek(fd, offset_bitmap_height, SEEK_SET);
    read(fd, &height, sizeof(int));
    assert(pImage->size.h == height);

    //Parse Header to Get Data Offset.
    lseek(fd, offset_bitmap_data, SEEK_SET);
    read(fd, &offset, sizeof(int));

    //Bring Bitmap Data from Data offset.
    for(r = height - 1; r >= 0 ; r--)
    {
        lseek(fd, offset, SEEK_SET);
        read(fd, &pImage->data[3*(r*width)], 3 * sizeof(uchar) * width);
        offset += 3*sizeof(uchar)*width;
    }

    close(fd);
    return 0;
}

void convert_img(IMAGE* pSrc, IMAGE* pDst)
{
    int r, c;
    int red, green , blue;
    int offset = pSrc->size.h - pDst->size.h;
    assert(offset >= 0);

    memset(pDst->data, 0, sizeof(uchar) * pDst->size.w * pDst->size.h);

    for(r = 0; r < pDst->size.h ; r++)
    {
        for(c = 0; c < pDst->size.w ; c++)
        {
            blue  = pSrc->data[3 * (c+ (r+offset) * pSrc->size.w) + 0];
            green = pSrc->data[3 * (c+ (r+offset) * pSrc->size.w) + 1];
            red   = pSrc->data[3 * (c+ (r+offset) * pSrc->size.w) + 2];

            pDst->data[c + r * pDst->size.w] = (blue + green + red) / 3;
        }
    }
}


#ifdef ENVIRONMENT_BOARD
int pixel_processing_4b( int Red, int Green, int Blue)
{
	int Red_temp, Green_temp, Blue_temp;

	Red_temp = Red >> 4;
	Green_temp = Green >> 4;
	Blue_temp = Blue >> 4;

    return (Red_temp + Green_temp + Blue_temp)/3;
};

int pixel_processing_16b( int Red, int Green, int Blue)
{
    int Red_temp, Green_temp, Blue_temp;
    Red_temp = Red >> 3;
    Green_temp = Green >> 2;
    Blue_temp = Blue >> 3;

    return ((Red_temp<<11) | (Green_temp<<5) | (Blue_temp));
};

void write_orig_to_shmem(volatile ushort* addr, IMAGE* pImage)
{
    int r, c;
    long red, green, blue;

    AREA size = pImage->size;

    for(r = 0; r < size.h; r++)
    {
        for(c = 0; c < size.w; c++)
        {
            blue  = pImage->data[3 * (c + r * size.w) + 0];
            green = pImage->data[3 * (c + r * size.w) + 1];
            red   = pImage->data[3 * (c + r * size.w) + 2];
        }
        *addr = (ushort)pixel_processing_16b(red, green, blue);
        addr += 1;
    }
}

void write_lp_to_shmem(volatile uchar* addr, IMAGE* pImage)
{
    int r, c;
    long pixel0, pixel1, red, green, blue;

    AREA size = pImage->size;

    for(r = 0; r < size.h; r++)
    {
        for(c = 0; c < size.w; c++)
        {
            if(c%2 == 0)
            {
                blue  = pImage->data[c + r * size.w];
                green = pImage->data[c + r * size.w];
                red   = pImage->data[c + r * size.w];

                pixel0 = pixel_processing_4b(red, green, blue);
            }
            else
            {
                blue  = pImage->data[c + r * size.w];
                green = pImage->data[c + r * size.w];
                red   = pImage->data[c + r * size.w];

                pixel1 = pixel_processing_4b(red, green, blue);

                *addr = (pixel1<<4) | pixel0;
                addr +=1;
            }
        }
    }
}

void write_token_to_shmem(volatile uchar** table, IMAGE* pImage)
{
    volatile uchar* addr;
    AREA size = pImage->size;

    int r, c, i;
    long pixel0, pixel1, red, green, blue;

    for(i = 0 ; i < 7 ; i++)
    {
        addr = table[i];
        for(r = 0; r < 32 ; r++)
        {
            for(c = 0; c < 32; c++)
            {
                if(c%2 == 0)
                {
                    blue  = pImage->data[c+32*i + r * size.w];
                    green = pImage->data[c+32*i + r * size.w];
                    red   = pImage->data[c+32*i + r * size.w];
                    pixel0 = pixel_processing_4b(red, green, blue);
                }
                else
                {
                    blue  = pImage->data[c+32*i + r * size.w];
                    green = pImage->data[c+32*i + r * size.w];
                    red   = pImage->data[c+32*i + r * size.w];
                    pixel1 = pixel_processing_4b(red, green, blue);

                    *addr = (pixel1<<4) | pixel0;
                    addr +=1;
                }
            }
        }
    }
}
#endif
