#ifndef __RM_CILENT_UI__
#define __RM_CILENT_UI__

#define Robot_ID UI_Data_RobotID_RHero
#define Cilent_ID UI_Data_CilentID_RHero        //?¨²?¡Â¨¨???¨¦?¨¦¨¨??

#include "stm32h7xx.h"
#include "stdarg.h"
#include "usart.h"
#include "math.h"

#define NULL 0
#define __FALSE 100

/****************************?a¨º?¡À¨º??*********************/
#define UI_SOF 0xA5
/****************************CMD_ID¨ºy?Y********************/
#define UI_CMD_Robo_Exchange 0x0301    
/****************************?¨²¨¨YID¨ºy?Y********************/
#define UI_Data_ID_Del 0x100 
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************o¨¬¡¤??¨²?¡Â¨¨?ID********************/
#define UI_Data_RobotID_RHero 1         
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************¨¤?¡¤??¨²?¡Â¨¨?ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************o¨¬¡¤?2¨´¡Á¡Â¨º?ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************¨¤?¡¤?2¨´¡Á¡Â¨º?ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************¨¦?3y2¨´¡Á¡Â***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************¨ª?D?????2?¨ºy__¨ª?D?2¨´¡Á¡Â********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************¨ª?D?????2?¨ºy__¨ª?D?¨¤¨¤D¨ª********************/
#define UI_Graph_Line 0         //?¡À??
#define UI_Graph_Rectangle 1    //??D?
#define UI_Graph_Circle 2       //???2
#define UI_Graph_Ellipse 3      //¨ª??2
#define UI_Graph_Arc 4          //?2??
#define UI_Graph_Float 5        //??¦Ì?D¨ª
#define UI_Graph_Int 6          //??D?
#define UI_Graph_Char 7         //¡Á?¡¤?D¨ª
/***************************¨ª?D?????2?¨ºy__¨ª?D???¨¦?********************/
#define UI_Color_Main 0         //o¨¬¨¤??¡Â¨¦?
#define UI_Color_Yellow 1
#define UI_Color_Green 2
#define UI_Color_Orange 3
#define UI_Color_Purplish_red 4 //¡Á?o¨¬¨¦?
#define UI_Color_Pink 5
#define UI_Color_Cyan 6         //?¨¤¨¦?
#define UI_Color_Black 7
#define UI_Color_White 8


#pragma pack(1)                           //¡ã¡ä1¡Á??¨²????¡ê??D??2??¨¦DT??


typedef unsigned char Uint8_t;
typedef unsigned char u8;
typedef uint16_t u16;
typedef uint32_t u32;




typedef struct
{
   u8 SOF;                    //?e¨º?¡Á??¨²,1¨¬?¡§0xA5
   u16 Data_Length;           //??¨ºy?Y3¡è?¨¨
   u8 Seq;                    //¡ã¨¹D¨°o?
   u8 CRC8;                   //CRC8D¡ê?¨¦?¦Ì
   u16 CMD_ID;                //?¨¹¨¢?ID
} UI_Packhead;             //??¨ª¡¤

typedef struct
{
   u16 Data_ID;               //?¨²¨¨YID
   u16 Sender_ID;             //¡¤¡é?¨ª??ID
   u16 Receiver_ID;           //?¨®¨º???ID
} UI_Data_Operate;         //2¨´¡Á¡Â?¡§¨°???

typedef struct
{
   u8 Delete_Operate;         //¨¦?3y2¨´¡Á¡Â
   u8 Layer;                  //¨¦?3y¨ª?2?
} UI_Data_Delete;          //¨¦?3y¨ª?2???


typedef struct
{ 
   uint8_t graphic_name[3]; 
   uint32_t operate_tpye:3; 
   uint32_t graphic_tpye:3; 
   uint32_t layer:4; 
   uint32_t color:4; 
   uint32_t start_angle:9;
   uint32_t end_angle:9;
   uint32_t width:10; 
   uint32_t start_x:11; 
   uint32_t start_y:11;
   int32_t graph_Float;              //??¦Ì?¨ºy?Y
} Float_Data;


typedef struct
{ 
uint8_t graphic_name[3]; 
uint32_t operate_tpye:3; 
uint32_t graphic_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11;
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11;              //¨ª?D?¨ºy?Y
} Graph_Data;


typedef struct
{
   Graph_Data Graph_Control;
   uint8_t show_Data[30];
} String_Data;                  //¡ä¨°¨®?¡Á?¡¤?¡ä?¨ºy?Y

#pragma pack()

void UI_Delete(u8 Del_Operate,u8 Del_Layer);
void Line_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
int UI_ReFresh(int cnt,...);
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
void Circle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 Graph_Radius);
void Rectangle_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 End_x,u32 End_y);
void Float_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float);
void Char_Draw(String_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,char *Char_Data);
int Char_ReFresh(String_Data string_Data);
void Arc_Draw(Graph_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_StartAngle,u32 Graph_EndAngle,u32 Graph_Width,u32 Start_x,u32 Start_y,u32 x_Length,u32 y_Length);
int Cal_rotation_x(float x, float y, float angle);
int Cal_rotation_y(float x, float y, float angle);
void Int_Draw(Float_Data *image,char imagename[3],u32 Graph_Operate,u32 Graph_Layer,u32 Graph_Color,u32 Graph_Size,u32 Graph_Digit,u32 Graph_Width,u32 Start_x,u32 Start_y,float Graph_Float);
extern uint8_t UIsend_buffer[512],top;
extern uint8_t head[128],num,locked;
#endif