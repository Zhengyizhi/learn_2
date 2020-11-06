#ifndef __JUDGE_H
#define __JUDGE_H



#include "system.h"


/***************ÃüÁîÂëID********************/

/* 

	ID: 0x0001  Byte:  3    ±ÈÈü×´Ì¬Êı¾İ       			·¢ËÍÆµÂÊ 1Hz      
	ID: 0x0002  Byte:  1    ±ÈÈü½á¹ûÊı¾İ         		±ÈÈü½áÊøºó·¢ËÍ      
	ID: 0x0003  Byte:  32   ±ÈÈü»úÆ÷ÈËÑªÁ¿Êı¾İ   		1Hz·¢ËÍ       **
	ID: 0x0004  Byte:  3   	·ÉïÚ·¢Éä×´Ì¬  		·			·ÉïÚ·¢ÉäÊ±·¢ËÍ**
	ID: 0x0005  Byte:  3   	ÈË¹¤ÖÇÄÜÌôÕ½Èü¼Ó³ÉÓë³Í·£Êı¾İ   **
	
	ID: 0x0101  Byte:  4    ³¡µØÊÂ¼şÊı¾İ   				ÊÂ¼ş¸Ä±äºó·¢ËÍ
	ID: 0x0102  Byte:  3    ³¡µØ²¹¸øÕ¾¶¯×÷±êÊ¶Êı¾İ    	¶¯×÷¸Ä±äºó·¢ËÍ 
	ID: 0X0104  Byte:  2    ²ÃÅĞ¾¯¸æÊı¾İ
	ID: 0x0105  Byte:  1    ·ÉïÚ·¢Éä¿Úµ¹¼ÆÊ±
	
	ID: 0X0201  Byte: 15    »úÆ÷ÈË×´Ì¬Êı¾İ        		10Hz
	ID: 0X0202  Byte: 14    ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ   			50Hz       
	ID: 0x0203  Byte: 16    »úÆ÷ÈËÎ»ÖÃÊı¾İ           	10Hz
	ID: 0x0204  Byte:  1    »úÆ÷ÈËÔöÒæÊı¾İ           	ÔöÒæ×´Ì¬¸Ä±äºó·¢ËÍ
	ID: 0x0205  Byte:  3    ¿ÕÖĞ»úÆ÷ÈËÄÜÁ¿×´Ì¬Êı¾İ      10Hz
	ID: 0x0206  Byte:  1    ÉËº¦×´Ì¬Êı¾İ           		ÉËº¦·¢Éúºó·¢ËÍ
	ID: 0x0207  Byte:  6    ÊµÊ±Éä»÷Êı¾İ           		×Óµ¯·¢Éäºó·¢ËÍ
	ID: 0x0208  Byte:  2    µ¯ÍèÊ£ÓàÊıÁ¿  ½ö¿ÕÖĞ»úÆ÷ÈË ÉÚ±ø
	ID: 0x0209  Byte:  4    »úÆ÷ÈËRFID×´Ì¬
	
	ID: 0x0301  Byte:  n    »úÆ÷ÈË¼ä½»»¥Êı¾İ           	·¢ËÍ·½´¥·¢·¢ËÍ,10Hz
	
*/

#define JUDGE_BUFFER_LEN 200              //»º´æÇø³¤¶È

#define JUDGE_FRAME_HEADER 0xA5           //Ö¡Í·

#define    JUDGE_DATA_ERROR      0
#define    JUDGE_DATA_CORRECT    1     //²ÃÅĞÏµÍ³¿ÉÒÔ²»¿ÉÒÔÓÃ

#define shoot_speed_limit 20;

#define TRUE 1
#define FALSE 0

#define BLUE 0
#define RED 1

//³¤¶È¸ù¾İĞ­Òé¶¨Òå,Êı¾İ¶Î³¤¶ÈÎªnĞèÒª¸ù¾İÖ¡Í·µÚ¶ş×Ö½ÚÀ´»ñÈ¡
#define    LEN_HEADER    5        //Ö¡Í·³¤
#define    LEN_CMDID     2        //ÃüÁîÂë³¤¶È
#define    LEN_TAIL      2	      //Ö¡Î²CRC16

/* RFID¿¨ÀàĞÍ */
#define    CARD_ATTACK        ((uint8_t)0x00)
#define    CARD_PROTECT       ((uint8_t)0x01)
#define    CARD_BLOOD_RED     ((uint8_t)0x02)
#define    CARD_BLOOD_BLUE    ((uint8_t)0x03)
#define    CARD_HEAL_RED      ((uint8_t)0x04)
#define    CARD_HEAL_BLUE     ((uint8_t)0x05)
#define    CARD_COLD_RED      ((uint8_t)0x06)
#define    CARD_COLD_BLUE     ((uint8_t)0x07)
#define    CARD_FORT          ((uint8_t)0x08)


#define    LEN_HEADER    5        //Ö¡Í·³¤
#define    LEN_CMDID     2        //ÃüÁîÂë³¤¶È
#define    LEN_TAIL      2	      //Ö¡Î²CRC16

//Í¨ĞÅĞ­Òé¸ñÊ½
typedef enum  
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

// frame_header ¸ñÊ½
typedef enum
{
	SOF          = 0,//ÆğÊ¼Î»
	DATA_LENGTH  = 1,//Ö¡ÄÚÊı¾İ³¤¶È,¸ù¾İÕâ¸öÀ´»ñÈ¡Êı¾İ³¤¶È
	SEQ          = 3,//°üĞòºÅ
	CRC8         = 4 //CRC8	
}	FrameHeaderOffset;


typedef enum
{
	ID_game_state       = 0x0001,
	ID_game_result      = 0x0002,
	ID_game_robot_survivors       	= 0x0003,//±ÈÈü»úÆ÷ÈË´æ»îÊı¾İ
	ID_game_missile_state = 0x0004, //·ÉïÚ·¢Éä×´Ì¬
	ID_game_buff         =0x0005,//buff
	
	ID_event_data  					= 0x0101,//³¡µØÊÂ¼şÊı¾İ 
	ID_supply_projectile_action   	= 0x0102,//³¡µØ²¹¸øÕ¾¶¯×÷±êÊ¶Êı¾İ
	ID_supply_warm 	= 0x0104,//²ÃÅĞÏµÍ³¾¯¸æÊı¾İ
	ID_missile_shoot_time =0x0105  , //·ÉïÚ·¢Éä¿Úµ¹¼ÆÊ±
	
	ID_game_robot_state    			= 0x0201,//»úÆ÷ÈË×´Ì¬Êı¾İ
	ID_power_heat_data    			= 0x0202,//ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ
	ID_game_robot_pos        		= 0x0203,//»úÆ÷ÈËÎ»ÖÃÊı¾İ
	ID_buff_musk					= 0x0204,//»úÆ÷ÈËÔöÒæÊı¾İ
	ID_aerial_robot_energy			= 0x0205,//¿ÕÖĞ»úÆ÷ÈËÄÜÁ¿×´Ì¬Êı¾İ
	ID_robot_hurt					= 0x0206,//ÉËº¦×´Ì¬Êı¾İ
	ID_shoot_data					= 0x0207,//ÊµÊ±Éä»÷Êı¾İ
	ID_shoot_num          = 0x0208,//Ê£Óà·¢ÉäÊı

}CmdID;

typedef enum
{
	LEN_game_state       				=  3,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_survivors       		=  32,	//0x0003  ±ÈÈü»úÆ÷ÈËÑªÁ¿Êı¾İ
	LED_game_missile_state      =3  , //0X0004·ÉïÚ·¢Éä×´Ì¬
	LED_game_buff               =3 , //0X0005
	
	LEN_event_data  					=  4,	//0x0101  ³¡µØÊÂ¼şÊı¾İ 
	LEN_supply_projectile_action        =  3,	//0x0102³¡µØ²¹¸øÕ¾¶¯×÷±êÊ¶Êı¾İ
	LEN_supply_warm        =2, //²ÃÅĞÏµÍ³¾¯¸æ 0x0104
	LEN_missile_shoot_time =1  , //·ÉïÚ·¢Éä¿Úµ¹¼ÆÊ±
	
	LEN_game_robot_state    			= 18,	//0x0201»úÆ÷ÈË×´Ì¬Êı¾İ
	LEN_power_heat_data   				= 16,	//0x0202ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ
	LEN_game_robot_pos        			= 16,	//0x0203»úÆ÷ÈËÎ»ÖÃÊı¾İ
	LEN_buff_musk        				=  1,	//0x0204»úÆ÷ÈËÔöÒæÊı¾İ
	LEN_aerial_robot_energy        		=  3,	//0x0205¿ÕÖĞ»úÆ÷ÈËÄÜÁ¿×´Ì¬Êı¾İ
	LEN_robot_hurt        				=  1,	//0x0206ÉËº¦×´Ì¬Êı¾İ
	LEN_shoot_data       				=  6,	//0x0207	ÊµÊ±Éä»÷Êı¾İ
	LEN_shoot_num          = 2,//Ê£Óà·¢ÉäÊı

}JudgeDataLength;

/* ×Ô¶¨ÒåÖ¡Í· */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
	
} xFrameHeader;

/* ID: 0x0001  Byte:  3    ±ÈÈü×´Ì¬Êı¾İ */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 

/* ID: 0x0002  Byte:  1    ±ÈÈü½á¹ûÊı¾İ */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 

/* ID: 0x0003  Byte:  32    ±ÈÈü»úÆ÷ÈËÑªÁ¿Êı¾İ */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;   
	uint16_t red_2_robot_HP;  
	uint16_t red_3_robot_HP;  
	uint16_t red_4_robot_HP;  
	uint16_t red_5_robot_HP;  
	uint16_t red_6_robot_HP;  
	uint16_t red_7_robot_HP;  
	uint16_t red_outpost_HP; 
  uint16_t red_base_HP; 
	
	uint16_t blue_1_robot_HP;   
	uint16_t blue_2_robot_HP;   
	uint16_t blue_3_robot_HP;   
	uint16_t blue_4_robot_HP;   
	uint16_t blue_5_robot_HP;   
	uint16_t blue_6_robot_HP;   
	uint16_t blue_7_robot_HP; 

	uint16_t blue_outpost_HP; 
  uint16_t blue_base_HP;   
}  ext_game_robot_HP_t; 

/* ID: 0x0004  Byte:  3    ·ÉïÚ·¢Éä×´Ì¬ */
typedef __packed struct 
{  
	uint8_t dart_belong; 
	uint16_t stage_remaining_time; 
} ext_dart_status_t;

/* ID: 0x0005  Byte:  3    buff */
typedef __packed struct
{ 
	uint8_t F1_zone_status:1;  
	uint8_t F1_zone_buff_debuff_status:3;   
	
	uint8_t F2_zone_status:1;  
	uint8_t F2_zone_buff_debuff_status:3;   
	
	uint8_t F3_zone_status:1;  
	uint8_t F3_zone_buff_debuff_status:3;  
	
	uint8_t F4_zone_status:1;  
	uint8_t F4_zone_buff_debuff_status:3;  
	
	uint8_t F5_zone_status:1;  
	uint8_t F5_zone_buff_debuff_status:3;  
	
	uint8_t F6_zone_status:1;  
	uint8_t F6_zone_buff_debuff_status:3;  
} ext_ICRA_buff_debuff_zone_status_t; 

/* ID: 0x0101  Byte:  4    ³¡µØÊÂ¼şÊı¾İ */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    ³¡µØ²¹¸øÕ¾¶¯×÷±êÊ¶Êı¾İ */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 

/* ID: 0x0104  Byte: 2   ²ÃÅĞÏµÍ³¾¯¸æĞÅÏ¢ */
typedef __packed struct 
{ 
  uint8_t level; 
	uint8_t foul_robot_id;
}  ext_referee_warning_t;  

/* ID: 0x0105  Byte:1  ·ÉïÚ·¢Éä¿Úµ¹¼ÆÊ± */
typedef __packed struct 
{ 
	uint8_t dart_remaining_time; 
}  ext_dart_remaining_time_t; ;  

/* ID: 0X0201  Byte: 18    »úÆ÷ÈË×´Ì¬Êı¾İ */
typedef __packed struct 
{ 
	uint8_t robot_id;   //»úÆ÷ÈËID£¬¿ÉÓÃÀ´Ğ£Ñé·¢ËÍ
	uint8_t robot_level;  //1Ò»¼¶£¬2¶ş¼¶£¬3Èı¼¶
	uint16_t remain_HP;  //»úÆ÷ÈËÊ£ÓàÑªÁ¿
	uint16_t max_HP; //»úÆ÷ÈËÂúÑªÁ¿
	uint16_t shooter_heat0_cooling_rate;  //»úÆ÷ÈË 17mm ×Óµ¯ÈÈÁ¿ÀäÈ´ËÙ¶È µ¥Î» /s
	uint16_t shooter_heat0_cooling_limit;   // »úÆ÷ÈË 17mm ×Óµ¯ÈÈÁ¿ÉÏÏŞ
	uint16_t shooter_heat1_cooling_rate;   
	uint16_t shooter_heat1_cooling_limit;  
	uint8_t shooter_heat0_speed_limit;
	uint8_t shooter_heat1_speed_limit;	
	uint8_t max_chassis_power; 
	uint8_t mains_power_gimbal_output : 1;  
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_state_t;   

/* ID: 0X0202  Byte: 16    ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   
	uint16_t chassis_current;    
	float chassis_power;   //Ë²Ê±¹¦ÂÊ 
	uint16_t chassis_power_buffer;//60½¹¶ú»º³åÄÜÁ¿
	uint16_t shooter_heat0;//17mm
	uint16_t shooter_heat1;  
	uint16_t mobile_shooter_heat2; 
} ext_power_heat_data_t; 

/* ID: 0x0203  Byte: 16    »úÆ÷ÈËÎ»ÖÃÊı¾İ */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 

/* ID: 0x0204  Byte:  1    »úÆ÷ÈËÔöÒæÊı¾İ */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 

/* ID: 0x0205  Byte:  3    ¿ÕÖĞ»úÆ÷ÈËÄÜÁ¿×´Ì¬Êı¾İ */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 

/* ID: 0x0206  Byte:  1    ÉËº¦×´Ì¬Êı¾İ */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 

/* ID: 0x0207  Byte:  6    ÊµÊ±Éä»÷Êı¾İ */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;   
	float bullet_speed;  
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  2    ×Óµ¯Ê£ÓàÊıÁ¿ */
typedef __packed struct 
{ 
	uint16_t bullet_remaining_num;   
}  ext_bullet_remaining_t; 

/* ID: 0x0209  Byte:  2    FRID×´Ì¬ */
typedef __packed struct 
{ 
	uint32_t rfid_status ;
}  ext_rfid_status_t; 

/* 
	
	½»»¥Êı¾İ£¬°üÀ¨Ò»¸öÍ³Ò»µÄÊı¾İ¶ÎÍ·½á¹¹£¬
	°üº¬ÁËÄÚÈİ ID£¬·¢ËÍÕßÒÔ¼°½ÓÊÜÕßµÄ ID ºÍÄÚÈİÊı¾İ¶Î£¬
	Õû¸ö½»»¥Êı¾İµÄ°ü×Ü¹²³¤×î´óÎª 128 ¸ö×Ö½Ú£¬
	¼õÈ¥ frame_header,cmd_id,frame_tail ÒÔ¼°Êı¾İ¶ÎÍ·½á¹¹µÄ 6 ¸ö×Ö½Ú£¬
	¹Ê¶ø·¢ËÍµÄÄÚÈİÊı¾İ¶Î×î´óÎª 113¡£
	Õû¸ö½»»¥Êı¾İ 0x0301 µÄ°üÉÏĞĞÆµÂÊÎª 10Hz¡£

	»úÆ÷ÈË ID£º
	1£¬Ó¢ĞÛ(ºì)£»
	2£¬¹¤³Ì(ºì)£»
	3/4/5£¬²½±ø(ºì)£»
	6£¬¿ÕÖĞ(ºì)£»
	7£¬ÉÚ±ø(ºì)£»
	11£¬Ó¢ĞÛ(À¶)£»
	12£¬¹¤³Ì(À¶)£»
	13/14/15£¬²½±ø(À¶)£»
	16£¬¿ÕÖĞ(À¶)£»
	17£¬ÉÚ±ø(À¶)¡£ 
	¿Í»§¶Ë ID£º 
	0x0101 ÎªÓ¢ĞÛ²Ù×÷ÊÖ¿Í»§¶Ë( ºì) £»
	0x0102 £¬¹¤³Ì²Ù×÷ÊÖ¿Í»§¶Ë ((ºì )£»
	0x0103/0x0104/0x0105£¬²½±ø²Ù×÷ÊÖ¿Í»§¶Ë(ºì)£»
	0x0106£¬¿ÕÖĞ²Ù×÷ÊÖ¿Í»§¶Ë((ºì)£» 
	0x0111£¬Ó¢ĞÛ²Ù×÷ÊÖ¿Í»§¶Ë(À¶)£»
	0x0112£¬¹¤³Ì²Ù×÷ÊÖ¿Í»§¶Ë(À¶)£»
	0x0113/0x0114/0x0115£¬²Ù×÷ÊÖ¿Í»§¶Ë²½±ø(À¶)£»
	0x0116£¬¿ÕÖĞ²Ù×÷ÊÖ¿Í»§¶Ë(À¶)¡£ 
*/
/* ½»»¥Êı¾İ½ÓÊÕĞÅÏ¢£º0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	Ñ§Éú»úÆ÷ÈË¼äÍ¨ĞÅ cmd_id 0x0301£¬ÄÚÈİ ID:0x0200~0x02FF
	½»»¥Êı¾İ »úÆ÷ÈË¼äÍ¨ĞÅ£º0x0301¡£
	·¢ËÍÆµÂÊ£ºÉÏÏŞ 10Hz  

	×Ö½ÚÆ«ÒÆÁ¿ 	´óĞ¡ 	ËµÃ÷ 			±¸×¢ 
	0 			2 		Êı¾İµÄÄÚÈİ ID 	0x0200~0x02FF 
										¿ÉÒÔÔÚÒÔÉÏ ID ¶ÎÑ¡È¡£¬¾ßÌå ID º¬ÒåÓÉ²ÎÈü¶Ó×Ô¶¨Òå 
	
	2 			2 		·¢ËÍÕßµÄ ID 	ĞèÒªĞ£Ñé·¢ËÍÕßµÄ ID ÕıÈ·ĞÔ£¬ 
	
	4 			2 		½ÓÊÕÕßµÄ ID 	ĞèÒªĞ£Ñé½ÓÊÕÕßµÄ ID ÕıÈ·ĞÔ£¬
										ÀıÈç²»ÄÜ·¢ËÍµ½µĞ¶Ô»úÆ÷ÈËµÄID 
	
	6 			n 		Êı¾İ¶Î 			n ĞèÒªĞ¡ÓÚ 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //Êı¾İ¶Î,nĞèÒªĞ¡ÓÚ113
} robot_interactive_data_t;




/**
  * @brief  ¶ÁÈ¡²ÃÅĞÊı¾İ,loopÖĞÑ­»·µ÷ÓÃ´Ëº¯ÊıÀ´¶ÁÈ¡Êı¾İ
  * @param  »º´æÊı¾İ
  * @retval ÊÇ·ñ¶ÔÕıÎóÅĞ¶Ï×ö´¦Àí
  * @attention  ÔÚ´ËÅĞ¶ÏÖ¡Í·ºÍCRCĞ£Ñé,ÎŞÎóÔÙĞ´ÈëÊı¾İ
  */
bool Judege_read_data(u8 *ReadFromUsart );










/***************************²ÃÅĞÏµÍ³´®¿Ú³õÊ¼»¯***********************************/
void UART5_Init( void );



//DMA³õÊ¼»¯
void UART5_DMA_Init( void );

/**
  * @brief  ´®¿ÚÒ»´Î·¢ËÍÒ»¸ö×Ö½ÚÊı¾İ
  * @param  ×Ô¼º´ò°üºÃµÄÒª·¢¸ø²ÃÅĞµÄÊı¾İ
  * @retval void
  * @attention  ´®¿ÚÒÆÎ»·¢ËÍ
  */
void UART5_SendChar(uint8_t cData);










#endif
