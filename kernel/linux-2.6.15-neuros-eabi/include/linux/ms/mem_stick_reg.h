
/*ADDRESSING MODE*/
	
#define USER_DATA 	1


/*MS MODE */

#define MSMDE 		(1<<0)
#define NO_MSMDE	~(1<<0)

/*MS SYSTEM */

#define MSRST		(1 << 15)
#define MSSRAC		(1 << 14)
#define MSINTEN		(1 << 13)
#define MSNOCRC		(1 << 12)
#define MSINTCLR	(1 << 11)
#define MSIEN		(1 << 10)
#define MSFCLR		(1 << 9)
#define MSFDIR		(1 << 8)
#define MSDAM		(1 << 7)
#define	MSDRQSL		(1 << 5)
#define MSREI		(1 << 4)

/*MS ENDIAN*/

#define MSEND		(1 << 0)

/*MS DMA TRIGGER */

#define	MSBTRG		(1 << 1)
#define MSSTRG		(1 << 0)

/*MS DMA MODE*/

#define MSDIEN		(1 << 13)
#define MSDIR		(1 << 12)
#define MSPRM_LE	~(0x3 << 10)
#define MSPRM_BE	~(0x2  << 10)
#define MSPRM_WS	~(0x1 << 10)
#define MSPRM_BS	(0x3  << 10)

/*DMA STATUS*/

#define MSRUN           (1 << 15)


/*MS STATUS VALUES*/

#define MS_CRC_ERROR	(1<<9)
#define MS_TIME_OUT	(1<<8)
#define MS_ERROR	(1<<2)
#define MS_FIFO_RW	(1<<14)


#define MS_DATA_REQ	(1<<1)

#define MEM_STICK_CRC_ERROR	1
#define MEM_STICK_TIME_OUT	2
#define MEM_STICK_NO_ERROR	3


#define REG_RW_CMD	0xA
#define DATA_RW_CMD	0xB


/*MODE OF TRANSFER*/
#define SERIAL		0x80
#define PARALLEL	0x00

#define SUCCESS		0
#define FAILURE		1
#define NO_CARD		2
#define NOT_DATA_STATE  3	
#define READ_WRITE	4
#define READ_ONLY	5

/*COMMANDS*/
/*FOR MEMORY STICK*/
#define READ_PAGE_DATA		(0x2000)
#define READ_SHORT_DATA		(0x3000)
#define READ_REG		(0x4000)
#define GET_INT			(0x7000)
#define WRITE_PAGE_DATA		(0xD000)
#define WRITE_REG		(0xB000)
#define SET_RW_REG_ADRS		(0x8000)
#define SET_CMD			(0xE000)

/*FOR MEMORY STICK PRO ONLY*/

#define EX_SET_CMD		(0x9000)

/*Yet to be filled*/

/*MEMORY STICK REGISTERS*/

#define INT_REG			(0x01)
#define STATUS_REG0		(0x02)
#define STATUS_REG1		(0x03)
#define TYPE_REG		(0x04)
#define CATEGORY_REG		(0x06)
#define CLASS_REG		(0x07)
#define SYS_PARAM_REG		(0x10)
#define BLOCK_ADD_REG2		(0x11)
#define BLOCK_ADD_REG1		(0x12)
#define BLOCK_ADD_REG0		(0x13)
#define CMD_PARAM_REG		(0x14)
#define PAGE_ADD_REG		(0x15)
#define OVER_WR_FLAG_REG	(0x16)
#define MANAGE_FLAG_REG		(0x17)
#define LOG_ADD_REG1		(0x18)		
#define LOG_ADD_REG0		(0x19)		

/*MEMORY STICK COMMANDS*/

/*MEMORY ACCESS COMMAND*/
#define BLOCK_READ		(0xAA00)
#define BLOCK_WRITE		(0x5500)
#define BLOCK_END		(0x3300)
#define BLOCK_ERASE		(0x9900)
#define FLASH_STOP		(0xCC00)

/*MEMORY STICK PRO COMMANDS*/

#define READ_DATA		(0x2000)
#define WRITE_DATA		(0x2100)
#define READ_ATRB		(0x2400)
#define STOP			(0x2500)
#define ERASE			(0x2600)
#define SET_IBD			(0x4600)
#define GET_IBD			(0x4700)

/*FUNCTION COMMAND*/

#define SLEEP			(0x1100) 
#define CLEAR_BUF		(0xC300)
#define RESET			(0x3C00)

/*Memory stick INT register status*/
	
#define NORM_COMP		0
#define CMD_ERR_TER		1
#define NORM_DATA_TRANS		2
#define DATA_REQ_ERR		3
#define CMD_EXE			4
#define CMD_NOT_EXE		5

struct attribute_info{
	unsigned short sig_code;
	unsigned short version;
	unsigned short no_of_entity;
	unsigned short done;
};

#define NO_OF_ENTITY 12
#define ENTRY_SIZE 12

struct entry_table{
	unsigned char *add;
	unsigned int size;
	unsigned char info_id;	
};
/*Device in formation ID*/


struct assembly_date{
	u8 time_diff;
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 minute;
	u8 second;
}__attribute__ ((packed));
	
/*Device information ID 10h*/
struct system_information{
	u8 ms_class;
	u8 for_uniq_valu1;
	u16 block_size;
	u16 total_block;
	u16 user_area_block;
	u16 page_size;
	//u8 res2[2];
	u8 extra_data_size;
	u8 for_uniq_valu2;
	struct assembly_date date;
        u8 serial_no[4];//MS for_uniq_valu1 1 bytei,serial number 3 bytes.MS-PRO 4 bytes for serial number.
	u8 ass_maker_code;
	u8 ass_model_code[3];
	u8 mem_maker_code[2];
	u8 mem_model_code[2];
	u16 implem_capacity;
	u8 for_uniq_valu3[2];
	u8 vcc;
	u8 vpp;
	u16 controller_no;
	u16 controller_func;
	u16 start_sector;
	u16 unit_size;
	u8 ms_sub_class;
	u8 res4[4];
	u8 interface_type;
	u16 controller_code;
	u8 format_type;
	u8 ms_application;
	u8 device_type;
	u8 res6[7];
	u8 ms_pro_ID[16];
	u8 res7[16];
}__attribute__ ((packed));
//};

