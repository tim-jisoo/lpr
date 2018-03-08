#include "../Header/timdebug.hpp"

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory.h>
#include <sys/mman.h>
#include "../Header/sysconfig.hpp"
#include "../Header/datatype.hpp"
#include "../Header/datastruct.hpp"
#include "../Header/lpr.hpp"
#include "../Tracker/Header/historybuff.hpp"
#include "../Tracker/Header/objtracker.hpp"

void read_frame_from_shmem(volatile ushort* addr, IMAGE*pImage);
void crop_img(IMAGE* src, IMAGE* dst);
void write_lp_to_shmem(volatile uchar* addr, IMAGE* pImage);
void write_token_to_shmem(volatile uchar** table, IMAGE* pImage);

uchar	_8bit_buff_1[52500];
uchar	_8bit_buff_2[52500];
uchar	_8bit_buff_3[52500];
int	_32bit_buff[52500];
int	_global_FrmOrigH;

int main(int argc, char** argv)
{
	volatile ushort		*orig_image_addr    = NULL;
   	volatile uchar		*plate_image_addr   = NULL;
   	volatile uchar		*text_image_addr[7] = {NULL};
   	void			*axi_virtual_base;
   	int			fd;

	RECT			position_license_plate;
	LPRManager		lm;
	ObjTracker		tracker;
	Recog_Hist_Manager	rhm;
	IMAGE			forig;
	IMAGE			fcvt;
	IMAGE			lp;
	IMAGE			tk;

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

	_global_FrmOrigH = SYSCONFIG_FRAME_ORIGIN_HEIGHT;	
	forig.allocate(1, SYSCONFIG_FRAME_ORIGIN_WIDTH, SYSCONFIG_FRAME_ORIGIN_HEIGHT);
	fcvt.allocate(1,SYSCONFIG_FRAME_CONVERT_WIDTH, SYSCONFIG_FRAME_CONVERT_HEIGHT);
	lp.allocate(1,SYSCONFIG_LICENSEPLATE_WIDTH, SYSCONFIG_LICENSEPLATE_HEIGHT);
	tk.allocate(1,SYSCONFIG_TOKEN_WIDTH * 7, SYSCONFIG_TOKEN_HEIGHT);
	rhm.allocate(HISTORYBUFF_MAXSIZE);
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
	rhm.init();

	while(1)
	{
		read_frame_from_shmem(orig_image_addr, &forig);

		//detection mode.
		if(!tracker.is_active())
		{
			crop_img(&forig, &fcvt);
			if(lm.recognize(&fcvt, &lp, &tk) < 0) continue;
			position_license_plate = lm.read_pos();
			position_license_plate.orig.y += (forig.size.h - fcvt.size.h);
			rhm.write_recog(position_license_plate, lm.read_param());
			if(rhm.observe_history_buff()) tracker.activate(&rhm);
			write_lp_to_shmem(plate_image_addr, &lp);
			write_token_to_shmem(text_image_addr, &tk);
		}

		//tracking mode.
		else
		{
			if(lm._Tracker_recognize(&forig, &lp, &tk, &tracker) < 0)
			{
				tracker.deactivate();
				rhm.init();
				continue;
			}
			
			write_lp_to_shmem(plate_image_addr, &lp);
			write_token_to_shmem(text_image_addr, &tk);
		}
	}
	
	forig.clear();
	fcvt.clear();
	lp.clear();
	tk.clear();
	lm.clear();
	rhm.clear();

	return 0;
}


void read_frame_from_shmem(volatile ushort* addr, IMAGE*pImage)
{
	int r, c;
	
	for(r = 0; r < pImage->size.h; r++)
	{
		for(c = 0; c < pImage->size.w; c++)
		{
			pImage->data[c + r * pImage->size.w] = (uchar)(*addr);
			addr+=1;
		}
	}
}

void crop_img(IMAGE* pSrc, IMAGE* pDst)
{
    int r;
    int offset = pSrc->size.h - pDst->size.h;

    assert(offset >= 0);

    for(r = 0; r < pDst->size.h; r++)
    {
        memcpy(
            &pDst->data[r * pDst->size.w],
            &pSrc->data[(offset + r) * pSrc->size.w],
            sizeof(uchar) * pSrc->size.w
        );
    }
}

int pixel_processing_4b( int Red, int Green, int Blue)
{
	int Red_temp, Green_temp, Blue_temp;

	Red_temp = Red >> 4;
	Green_temp = Green >> 4;
	Blue_temp = Blue >> 4;

    return (Red_temp + Green_temp + Blue_temp)/3;
};

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
