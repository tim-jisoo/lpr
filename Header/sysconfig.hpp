#ifndef SYSCONFIG_H_INCLUDED
#define SYSCONFIG_H_INCLUDED


//============= PARAMETER FOR ARM ==============================================================================================//
#define SYSCONFIG_FRAME_ORIGIN_WIDTH                300
#define SYSCONFIG_FRAME_ORIGIN_HEIGHT               250

#define SYSCONFIG_FRAME_CONVERT_HRATIO              0.7f
#define SYSCONFIG_FRAME_CONVERT_WIDTH               SYSCONFIG_FRAME_ORIGIN_WIDTH
#define SYSCONFIG_FRAME_CONVERT_HEIGHT              (int)((((float)(SYSCONFIG_FRAME_ORIGIN_HEIGHT))*(SYSCONFIG_FRAME_CONVERT_HRATIO)))

#define SYSCONFIG_LICENSEPLATE_WIDTH                250
#define	SYSCONFIG_LICENSEPLATE_HEIGHT               70

#define SYSCONFIG_TOKEN_WIDTH                       32
#define SYSCONFIG_TOKEN_HEIGHT                      32

//============= PARAMETER FOR FPGA ==============================================================================================//
#define SDRAM_BASE_ADDR                             0x00000000
#define ALT_VIP_SOFTWARE_RESET_N_BASE               0x00000200
#define ALT_AXI_FPGASLVS_OFST                       0xC0000000  // axi_master
#define HW_FPGA_AXI_SPAN                            0x40000000  // Bridge span
#define HW_FPGA_AXI_MASK                            ((HW_FPGA_AXI_SPAN) - (1))
#define ORIG_IMAGE_ADDR                             0x00000000
#define PLATE_IMAGE_ADDR                            0x00040000
#define TEXT_IMAGE0_ADDR                            0x00044c00
#define TEXT_IMAGE1_ADDR                            0x00044a00
#define TEXT_IMAGE2_ADDR                            0x00044800
#define TEXT_IMAGE3_ADDR                            0x00044600
#define TEXT_IMAGE4_ADDR                            0x00044400
#define TEXT_IMAGE5_ADDR                            0x00044200
#define TEXT_IMAGE6_ADDR                            0x00044000

#endif // SYSCONFIG_H_INCLUDED
