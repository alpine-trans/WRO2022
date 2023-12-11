#include "ev3api.h"
#include "app.h"
#include "time.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define UNKNOWN 0
#define BLACK 1
#define BLUE 2
#define GREEN 3
#define YELLOW 4
#define RED 5
#define WHITE 6
#define BROWN 7

#define PI 3.14159 //円周率
#define Wdis 17    //車輪間の距離
#define Wdia 8.6   //車輪の直径
#define rf   10    //基本的な止める反射光の値
#define ad   65    //基本的な追加で進む距離
#define dly  100   //基本的な待機時間
#define pw   30    //基本的なスピード
#define right 0    
#define left 1
#define rightleft 2

//#define HT_COLOR 1
//#define GYRO_SENS 1
//===========変数の宣言リスト============

uint8_t color=0;
uint8_t color_R=0;
uint8_t color_L=0;

ulong_t tim;

rgb_raw_t rgb_val;
static FILE *bt = NULL; 


float angle=0;                                  //モーターが動いた角度
float rotation=0;                               //モーターを動かしたい角度
float Kp=0;                                     //Ｐ値
float Kd=0;                                     //Ｄ値
float error=0;                                  //値の差
float lasterror=0;                              //前回の値の差
float delivetive=0;                             //前回と今回の値の差の差
float steer=0;                                  //補正値
int rval=0;                                     //取得したR値を入れる
int gval=0;                                     //取得したG値を入れる
int bval=0;                                     //取得したB値を入れる
int rsum=0;                                     //R値の合計
int gsum=0;                                     //G値の合計
int bsum=0;                                     //B値の合計
int gw_col[4]={10,10,10,10};                 //病室のマーキングブロックの色の判別　インデックス0空1緑2赤3黄 0無1緑2白
int Lan_col[4]={10,10,10,10};                   //ランドリーブロックの色の格納  0無1赤2黄3黒
int fm_col[3]={10,10,10};
int Lan_pos[3]={2,2,2};                         //赤の位置，黄の位置，黒の位置
int rm_cnt=0;          
int val=0;                         

//===========関数の宣言リスト============
void config();                                                              //モーターセンサーのコンフィグ
void cfg_bluetooth();                                                       //Bluetoothのコンフィグ
void forward(float power,float dis,int brake,int change);                          //前進(スピード，距離，ブレーキの有無01，変速の有無01)
void colorforward(float power,float add,int brake);
void back(float power,float dis,int brake);                                    //後退(スピード，距離，ブレーキの有無01)
void rs(float deg);                                                         //右回転スピン(角度)
void ls(float deg);                                                         //左回転スピン(角度)
void spin(int rl,float deg);                                                          //rl 01
void Line(int power,float dis,int brake,int pd,int change);                  //二つのカラーセンサーによるライントレース(スピード，距離，ブレーキの有無01，pd値の変化の有無01，変速の有無01)
void crossLine(float power,float dis,float ref,int brake,float add,int pd);     //二つのカラーセンサーで反射光が低い値になったら止まるライントレース(スピード，ライントレースで進む距離，止めるときの反射光の値，ブレーキの有無01，停止後に追加で進む距離)
void color_rgb(int num);                                                    //カラーセンサーでRGBでの値を取得(カラーセンサーのポート番号)
void dp_rgb(int num);                                                       //取得したRGBの値をTeraTermで表示する(カラーセンサーのポート番号)
int r_rgb(int num);                                                         //カラーセンサーでRGBの値を取得しRの値を返す(カラーセンサーのポート番号)
int g_rgb(int num);                                                         //カラーセンサーでRGBの値を取得しGの値を返す(カラーセンサーのポート番号)
int b_rgb(int num);                                                         //カラーセンサーでRGBの値を取得しBの値を返す(カラーセンサーのポート番号)
void wall(int time);
void arm(int power,int angle);

void singleLine(int power,int distance);
void singlecrossLine(int power,int distance);
void backLine(int power,int distance);

float sign(float n);


void SGtoly2();           //StartGoalからly2地点までのルート

void ly2tobt2();          //ly2地点からbt2地点までのルート

void bt2togr2();          //bt2地点からgr2地点までのルート
void bt2togr3();
void bt2toby3();

void gr2toby3();

void gr3toSG();
void gr3toby2();
void gr3toby3();          //gr3地点からby3地点までのルート

void by2togr3();

void by3toSG();
void by3togr3();

void room(int c,int gw);              //病室内の動き
void rooms(int gw);             //病室内及び病室間の動き
void read_frame();       //ランドリーエリアの枠を押す
void read_landry(int c);
void pick_landry(int c);
void put_landry();        //ランドリーブロックを置く
void read_marking();
void pick_bottle();       //ボトルを回収する
void put_bottle(int rl, int fs);
void move_ball(int c);

void L(int dis, int change, int brake);
void turn(int brake, int port);

//=========主プログラム＝＝＝＝＝＝＝
void main_task(intptr_t unused) {//cd c:ev3rt/hrp2/sdk/workspace  export LANG=ja_JP.UTF-8
config();
cfg_bluetooth();
fprintf(bt,"*****************************************************\r\n");
fprintf(bt,"%d mV\r\n",ev3_battery_voltage_mV());
// if(ev3_battery_voltage_mV() < 7800)ev3_speaker_play_tone(2000,1000);
// ev3_color_sensor_get_reflect(0);  ev3_color_sensor_get_reflect(1);
// r_rgb(2);  r_rgb(3);
// ev3_lcd_set_font(EV3_FONT_MEDIUM);
// ev3_motor_reset_counts(3);  while(abs(ev3_motor_get_counts(3)) < 140){  ev3_motor_set_power(3,-15);  }  ev3_motor_stop(3,true);


    // int i=100;

    // FILE *file;//ファイルポインタを宣言
    // file=fopen("/test.txt","a");//ファイルをオープン(名前の指定)
    // fprintf(file,"Hello %d\n",i);//書き込み
    // fclose(file);//ファイルを閉じる


    // memfile_t memfile;//ファイル構造体を作成
    // ev3_memfile_load("/abc.wav", &memfile);//SDカード内の"test.wav"を紐づけ

    // ev3_speaker_set_volume(100);//ボリュームを100に設定
    // ev3_speaker_play_file(&memfile,SOUND_MANUAL_STOP);//音声を再生


  // r_rgb(0);  r_rgb(1);
  // L(100,1,1);
  // for(int i=0;i<4;i++){
  // L(520,1,1);
  // turn(1,1);
  // L(480,1,1);
  // turn(1,1);}
  // L(150,0,0);


// int a=0;int sum=0;int n=0;
// for(int i=0; a<10000; i++){
//   if(i%1000000==0){
//     a++;sum+=r_rgb(0);n=sum/a;
//     fprintf(bt,"%d %d\r\n",r_rgb(0),n);
//     char str[20];
//     sprintf(str,"%d",n);
//     ev3_lcd_draw_string(str,0,0);
//   }
// }

  // crossLine(20,130,rf,0,0,0);
  // back(30,470,0);
  // rs(80);
  // forward(70,400,1,1);
  // crossLine(70,300,rf,0,50,1);

// forward(30,100,0,0);
// for(int i=0;i<40;i++){
//   singlecrossLine(90,500);
  
//   ev3_motor_reset_counts(2);
//   while(ev3_motor_get_counts(2)>-70){
//       ev3_motor_set_power(1,-90);
//       ev3_motor_set_power(2,-90);
//   }	
//   ev3_motor_stop(1,true);  ev3_motor_stop(2,true);
//   tslp_tsk(dly);
// }


  // ev3_motor_reset_counts(2);
  // rotation = 0;
  // while(rotation < 1000*360.0/(Wdia*10*PI)){
  //   error=ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);//r_rgb(0) - r_rgb(1);
  //   steer=0.7*error+14*(error - lasterror);
  //   ev3_motor_set_power(1,-80 - steer);
  //   ev3_motor_set_power(2,80 -  steer);
  //   lasterror=error;
  //   rotation=ev3_motor_get_counts(2);
  //   fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //   tslp_tsk(1);
  // }
  // ev3_motor_stop(1,true);
  // ev3_motor_stop(2,true);


  // ev3_motor_reset_counts(2);
  // rotation=1000*360.0/(Wdia*10*PI);
  // Kp=0.3;  Kd=13;
  // angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0; 

  // while(abs(angle) < rotation){
  //   error=ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
  //   delivetive=error-lasterror;
  //   steer=Kp*error+Kd*delivetive;

  //   ev3_motor_set_power(1,-1*(200 + steer*1.3));
  //   ev3_motor_set_power(2,200 - steer*1.3);
    
  //   angle=ev3_motor_get_counts(2);
  //   lasterror=error;
  //   fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //   tslp_tsk(1);
  
  // }
  // while(ev3_motor_get_power(2) > 0){
  //   ev3_motor_stop(1,true);  
  //   ev3_motor_stop(2,true);
  //   fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  // }

/*singleline
  // ev3_motor_reset_counts(2);
  
  // while(ev3_motor_get_counts(2) < 100000000000){

  // fprintf(bt,"%d\r\n",(int)ev3_color_sensor_get_reflect(0));
  //   if(ev3_color_sensor_get_reflect(0) > 17){

  //     ev3_motor_set_power(2,30);
  //     ev3_motor_stop(1, true);
  //   }
  //   else{

  //     ev3_motor_set_power(1,-50);
  //     ev3_motor_stop(2, true);
  //   }
    
  // }
  // ev3_motor_stop(1,true);
  // ev3_motor_stop(2,true);
  */

/*//quick turn
  Kp=0.2;  Kd=2;

  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  rotation=300*360.0/(Wdia*10*PI);  
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;  

  while((abs(angle) < rotation) || (b_rgb(1)>10 && b_rgb(0)>10)){

    error=r_rgb(0) - r_rgb(1);
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

    ev3_motor_set_power(1,-1*(70 + steer));
    ev3_motor_set_power(2,70 - steer);
    
    angle=ev3_motor_get_counts(2);
    lasterror=error;
    tslp_tsk(1);
  }


  ev3_motor_reset_counts(1);
  fprintf(bt,"%d\r\n",(int)b_rgb(0));
  while((abs(ev3_motor_get_counts(1)) < 100) || (ev3_color_sensor_get_reflect(1)>rf)){
    ev3_motor_set_power(1,-70);
    ev3_motor_stop(2,true);
  }	


  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  rotation=500*360.0/(Wdia*10*PI);
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;  
  while(b_rgb(1)>40 && b_rgb(0)>40){

    error=r_rgb(0) - r_rgb(1);
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

      ev3_motor_set_power(1,-1*(70 + steer));
      ev3_motor_set_power(2,70 - steer);
  
    
    angle=ev3_motor_get_counts(2);
    lasterror=error;
    tslp_tsk(1);
  }



    ev3_motor_stop(1,true);
    ev3_motor_stop(2,true);
  tslp_tsk(dly);


*/

// while(1){
//     
// }
//   ev3_motor_reset_counts(2);
//   Kp=0.3;
//   Kd=10;int i =0 ;
//   while(ev3_motor_get_counts(2) < 5000){
// i++;if(i%5000 == 0)fprintf(bt,"%f \r\n",Kd*delivetive);
//     error = ev3_color_sensor_get_reflect(0) -ev3_color_sensor_get_reflect(1);
//     delivetive = error-lasterror;
//     steer = Kp*error + Kd*delivetive;

//     //出力
//     ev3_motor_set_power(1, -1*(80 + steer));
//     ev3_motor_set_power(2, 80 - steer);
    
//     lasterror = error;

//   }
// ev3_motor_stop(1,true);
// ev3_motor_stop(2,true);


/*line noless
  ev3_motor_reset_counts(2);
  rotation=1000000*360.0/(Wdia*10*PI);
  Kp=0.1;
  Kd=1;
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;  

  while(abs(angle) < rotation){
  fprintf(bt,"%d  ",(int)ev3_motor_get_power(1));
  fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  fprintf(bt,"%f\r\n",steer);
    error = b_rgb(0)-b_rgb(1);
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

    if (steer < -1){
      ev3_motor_set_power(1,-20);
      ev3_motor_set_power(2,20+steer);
    }
    else if(steer > 1){
      ev3_motor_set_power(1,-20-steer);
      ev3_motor_set_power(2,20);
    }
    else{
      ev3_motor_set_power(1,-20);
      ev3_motor_set_power(2,20);
      }

      // ev3_motor_set_power(1,-20-steer);
      // ev3_motor_set_power(2,20+steer);

    angle=ev3_motor_get_counts(2);
    lasterror=error;
 
  }
  ev3_motor_stop(1,true);
  ev3_motor_stop(2,true);
  */

  // ev3_motor_reset_counts(2);
  // rotation=700*360.0/(Wdia*10*PI);
  // angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;
  // int p=0;

  // Kp=0.2;  Kd=10;
  // while(ev3_motor_get_power(2) < 55){
  //   error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
  //   delivetive = error - lasterror;
  //   steer = Kp*error + Kd*delivetive;

  //   if(p<=100) p+=20;
  //   ev3_motor_set_power(1,-1*(p + steer*1.2));
  //   ev3_motor_set_power(2,p - steer*1.2);
    
  //   angle = ev3_motor_get_counts(2);
  //   lasterror = error;
  //   if(abs(angle) >= rotation) {break; ev3_speaker_play_tone(2000,1000);}
  //   fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //   tslp_tsk(1);
  // }
  
  // fprintf(bt,"100\r\n");
  // fprintf(bt,"0\r\n");

  // Kp=0.1;  Kd=8;
  // while(abs(angle) < rotation-70){
  //   error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
  //   delivetive = error - lasterror;
  //   steer = Kp*error + Kd*delivetive;

  //   ev3_motor_set_power(1,-1*(80 + steer));
  //   ev3_motor_set_power(2,80 - steer);
    
  //   angle = ev3_motor_get_counts(2);
  //   lasterror = error;
  //   fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //   tslp_tsk(1);
  // }

  // fprintf(bt,"100\r\n");
  // fprintf(bt,"0\r\n");
  
  // Kp=0.1;  Kd=7;
  // while(abs(angle) < rotation){
  //   error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
  //   delivetive = error - lasterror;
  //   steer = Kp*error + Kd*delivetive;

  //   ev3_motor_set_power(1,-1*(-5 + steer));
  //   ev3_motor_set_power(2,-5 - steer);
    
  //   angle = ev3_motor_get_counts(2);
  //   lasterror = error;
  //   fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //   tslp_tsk(1);
  // }
  
  // ev3_motor_stop(1,true);  
  // ev3_motor_stop(2,true);


/*
void drop(){
  int landry[3]=0;
  int landry_number=0;

//洗濯機を押す
  arm(3,0,1);
  akkun_stop(14,1);
  wall(700,15);

//移動　横向き
  back(150,20,1);
  arm(0,265,1);//前アームを上げる
  forward(25,15,1);
  L_spin(30,30);//元々40
  tslp_tsk(300);
  rgb_val.b=200;
  while(rgb_val.b>120){//線を読むまで前進
    ev3_color_sensor_get_rgb_raw(0,&rgb_val);
    ev3_motor_set_power(1,-8);
    ev3_motor_set_power(2,8);
  }	
  forward(55,8,1);//もともと24
  voice();
  ev3_motor_stop(2,true);
  L_spin(46,15);
  line(180,16,1);//元々200mm
  tslp_tsk(400);

//枠の色読み
  back(120,8,1);//もともと100
  for(int i=0; i<2; i++){
    yomi();//1個目
    voice();
    if(red<85)
      landry[i]=3;//黒
    else if(red/green>=1.8)
      landry[i]=1;//赤
    else
      landry[i]=2;//黄
    if(i==0){ 
      back(162,10,1);//もともと175
      akkun_stop(10,1);
      tslp_tsk(300);
    }
  }
  //3個目
  if((landry[0]==1 && landry[1]==2)  ||  (landry[0]==2 && landry[1]==1))//1と2が赤と黄色
    landry[2]=3;//黒
  else if((landry[0]==2 && landry[1]==3  ||  (landry[0]==3 && landry[1]==2))//1と2が黄色と黒
    landry[2]=1;//赤
  else//1と2が赤と黒
    landry[2]=2;//黄

//ブロックを排出
  ev3_motor_reset_counts(2);
  for(int i=0;i<=2;i++){
    tslp_tsk(200);
    yomi_kago();//かごの色読み
    imano_kyori=ev3_motor_get_counts(2)*270/360;//今の距離(mm)
    if(red<50){//黒だったら
      if(landry[0]==3)//黒置く場所が1だったら
        landry_number=1;
      else if(landry[1]==3)//黒置く場所が2だったら
        landry_number=2;
      else//黒置く場所が3だったら
        landry_number=3;
    }
    else if(red/green>2){//赤だったら
      if(landry[0]==1)//赤置く場所が1だったら
        landry_number=1;
      else if(landry[1]==1)//赤置く場所が2だったら
        landry_number=2;
      else//赤置く場所が3だったら
        landry_number=3;
    }
    else{//黄色だったら
      if(landry[0]==2)//黄色置く場所が1だったら
        landry_number=1;
      else if(landry[1]==2)//黄色置く場所が2だったら
        landry_number=2;
      else//黄色置く場所が3だったら
        landry_number=3;
    }
    kyori=-imano_kyori+(3-landry_number)*110+5;
    measure();
    drop_arm(landry_number);
  }
  imano_kyori=ev3_motor_get_counts(2)*270/360;//今の距離(mm)
  kyori=-imano_kyori-65;
  measure();
}

void measure(){//洗濯物を置きに行くときの前進後進
  if(abs(kyori)<10){//違いが1cm以下の時はなんもしない
  }
  else if(kyori>0){//前に進む時
    if(abs(imano_kyori)<50){//横の黒線をまたぐ時
      forward2(25,8,0);
      line2(kyori-25,20,1);
    }
    else{//またがない時
      line2(kyori,20,1);
    }
  }
  else{//後ろに下がる時
    //R_pd(-3,15);
    back2(abs(kyori),15,1);
  }
}
*/



/*


  Kp=0.3;
  while(abs(angle) < rotation){
    error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
    steer = Kp*error;

    ev3_motor_set_power(1,-1*(30 + steer));
    ev3_motor_set_power(2,30 - steer);
    
    angle = ev3_motor_get_counts(2);
    lasterror = error;
    fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
    tslp_tsk(1);
  }
  
  ev3_motor_stop(1,true);  
  ev3_motor_stop(2,true);
*/

/*
  // ev3_motor_reset_counts(2);
  // rotation=1000*360.0/(Wdia*10*PI);
  // Kp=0.3;  Kd=13;
  // angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;
  // float n[2][2] = {{80,1.3},{60,1.0}};

  // for(int i=0; i<2; i++){
  //   while((ev3_motor_get_power(2) < 60 && i == 0)  ||  ((abs(angle) < rotation*0.9 || abs(angle) < rotation-93) && i == 1)){
  //     error = ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
  //     delivetive = error - lasterror;
  //     steer = Kp*error + Kd*delivetive;

  //     ev3_motor_set_power(1,-1*(n[i][0] + steer*n[i][1]));
  //     ev3_motor_set_power(2,n[i][0] - steer*n[i][1]);
    
  //     angle = ev3_motor_get_counts(2);
  //     lasterror = error;
  //     if(abs(angle) >= rotation) break;
  //     fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //     tslp_tsk(1);
  //   }
  // }
  // for(int i=0; i<2; i++){
  //   while(ev3_motor_get_power(2) > 0){
  //     ev3_motor_stop(1,true);  
  //     ev3_motor_stop(2,true);
  //     fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //   }
  // }
  */

/*
  // int power=30;

  // while(power<70){

  //   power++;
  //   error=(ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1))+15;
  //   steer=0.3*error+13*(error - lasterror);
  //   ev3_motor_set_power(1,-1*(power + steer));
  //   ev3_motor_set_power(2,power -  steer);
  //   lasterror=error;
  //   fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  //   tslp_tsk(3.5);

  // }
*/

/*
  ev3_motor_reset_counts(2);
  rotation=0;
  lasterror=0;

  while(rotation < 1000*360.0/(Wdia*10*PI)){

  error=(ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1))+15;
  steer=0.3*error+13*(error - lasterror);
  ev3_motor_set_power(1,-(70 + steer));
  ev3_motor_set_power(2,70 -  steer);
  lasterror=error;
  rotation=ev3_motor_get_counts(2)/360;
  fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
  tslp_tsk(1);

  }*/

// /*
SGtoly2();   read_frame();
ly2tobt2();  pick_bottle();
bt2togr2();  read_marking();

if(gw_col[0] == gw_col[1]){   //左右のマーキングブロックが同じものかどうかの判断
  if(gw_col[0] == GREEN){     //GGWWのパターン
    gw_col[2]=WHITE;  gw_col[3]=WHITE;
    gr3toby3();  put_bottle(rightleft,2);  rooms(2);
    by3togr3();  rooms(0);
    gr3toSG();
  }
  else{                       //WWGGのパターン
    gw_col[2]=GREEN;  gw_col[3]=GREEN;
    put_bottle(rightleft,2);  rooms(0);                  //room:右折左折の選択、ボールの移動の有無  rooms:room(a,i)+移動+room(b,i+1)
    gr3toby3();  rooms(2);
    by3toSG();
  }
}

else{                           //WGWG GWWG WGGW GWGW
  if(gw_col[0] == GREEN)  put_bottle(left,0);//l
  else                    put_bottle(right,0);//r
  gr3toby2();  read_marking();
  if(gw_col[2] == GREEN)  put_bottle(left,1);//l
  else                    put_bottle(right,1);//r
  rooms(2);
  by3togr3();  rooms(0);
  gr3toSG();
}

put_landry();


// */
/**/
}  //main_task()

//=========関数定義============

void rooms(int gw){
  room(0,gw_col[gw]);
  Line(20,260,0,0,0);
  rs(180);
  room(1,gw_col[gw+1]);
}

void room(int c, int gw){
  read_landry(c);
  if(Lan_col[rm_cnt]==UNKNOWN&&gw==WHITE)ev3_speaker_play_tone(2000,500);
  else{
    pick_landry(c);
    if(gw==GREEN){
      move_ball(c);
      back(10,20,1);  tslp_tsk(200);  wall(500);
      forward(pw,330,0,0);
    }
    else if(gw==WHITE ){
      spin(c*-1+1,90);
      forward(20,110,0,1);
      spin(c*-1+1,90);
      ev3_motor_reset_counts(0);  ev3_motor_set_power(0,20);  tslp_tsk(200);  ev3_motor_stop(0,true);  tslp_tsk(200);
      arm(20,-150);
      back(20,50,0);
    }
  }
  colorforward(20,50,0);
  spin(c*-1+1,45);
  while(ev3_color_sensor_get_reflect(c)>rf){
    ev3_motor_set_power(1,(c*-2+1)*40);    ev3_motor_set_power(2,(c*-2+1)*40);
  } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);
  tslp_tsk(dly);
  rm_cnt++;
}

void read_frame(){
  back(15,125,0);  tslp_tsk(500);
  ev3_motor_reset_counts(2);  rotation=-180*Wdis/Wdia +3.2;
  while(ev3_motor_get_counts(2)> rotation){
    ev3_motor_set_power(1,-30+23*ev3_motor_get_counts(2)/rotation);
    ev3_motor_set_power(2,-30+23*ev3_motor_get_counts(2)/rotation);
  }	  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);

  back(15,120,0);  tslp_tsk(dly);
  Line(15,77,0,1,1);  tslp_tsk(dly);
  back(10,92,0);  tslp_tsk(dly);
  Line(15,77,0,1,1);  tslp_tsk(dly);
  ls(11);
  back(10,125,1); wall(800);

  forward(10,128,0,1);
  ev3_motor_reset_counts(2);
  while(abs(ev3_motor_get_counts(2))<Wdis/Wdia*170){
    ev3_motor_set_power(2,-20);
  }  ev3_motor_stop(2,true);
  // Line(10,150,0,1,1);
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  rotation=150*360.0/(Wdia*10*PI);  Kp=0.3;  Kd=6;
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;  
  while(abs(angle) < rotation){
    error=ev3_color_sensor_get_reflect(0)  -  ev3_color_sensor_get_reflect(1);
    delivetive=error-lasterror;  steer=Kp*error+Kd*delivetive;
    ev3_motor_set_power(1,-1*(10 + steer));  ev3_motor_set_power(2,10 - steer);
    angle=ev3_motor_get_counts(2);  lasterror=error;  tslp_tsk(1);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);
  tslp_tsk(dly);

  back(10,165,0);

  for(int i=0;i<2;i++){
    ev3_motor_reset_counts(2);   rsum=0; gsum=0; bsum=0; 

    while(ev3_motor_get_counts(2)<360.0/270*95){
      ev3_motor_set_power(1,-10);  ev3_motor_set_power(2,10);
      rsum+=r_rgb(2);  gsum+=g_rgb(2);  bsum+=b_rgb(2);
    }  
    ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  
    fprintf(bt,"Frame %d,%d,%d\r\n",rsum,gsum,bsum); 
    tslp_tsk(dly);

    if((rsum+gsum+bsum)>200000&&(rsum/bsum)>=5&&(rsum/gsum)>=3&&(gsum/bsum)<=2&&rsum>gsum&&rsum>bsum){
      fm_col[i]=RED;
      Lan_pos[0]=i;
      ev3_speaker_play_tone(523,300);
      ev3_lcd_draw_string("red",70*i,20);
    }
    else if((rsum+gsum+bsum)>300000&&(gsum/bsum)>=2){ 
      fm_col[i]=YELLOW;
      Lan_pos[1]=i;
      ev3_speaker_play_tone(1046,300);
      ev3_lcd_draw_string("yellow",70*i,20);
    }
    else if((rsum+gsum+bsum)<300000&&(gsum/bsum)<=2){
      fm_col[i]=BLACK;
      Lan_pos[2]=i;
      ev3_speaker_play_tone(262,300);
      ev3_lcd_draw_string("black",70*i,20);
    }

    if(i==0)  forward(10,15,0,1);
  }


  back(10,211,0);
  ev3_motor_reset_counts(1);  rotation=18*Wdis/Wdia;
  while(ev3_motor_get_counts(1)< rotation){
    ev3_motor_set_power(1,30-23*ev3_motor_get_counts(1)/rotation);
    ev3_motor_set_power(2,30-23*ev3_motor_get_counts(1)/rotation);
  } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);

  // forward(10,130,0,1);
  colorforward(10,88,0);
  ls(65);
  back(10,50,1);  wall(1000);
  forward(30,30,0,0);
  ls(6);
}

void read_landry(int c){
  crossLine(25,150,rf,0,0,1);
  // Line(20,170,0,0,0);
  back(30,230,0);
  crossLine(17,200,rf,0,27,1);
  tslp_tsk(200);
  
  ev3_motor_reset_counts(1);
  while(abs(ev3_motor_get_counts(1))< 90*(Wdis/Wdia-(c*-0.08+0.35))){
    ev3_motor_set_power(1,(c*-2+1)*30);  ev3_motor_set_power(2,(c*-2+1)*30);
  }	
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);

  // spin(c*-1+1,90);
  back(10,250,0);

  ev3_motor_reset_counts(2);   rsum=0; gsum=0; bsum=0;
  tslp_tsk(1000);

  while(ev3_motor_get_counts(2)<360.0/270*100){
    ev3_motor_set_power(1,-10);  ev3_motor_set_power(2,10);
    rsum+=r_rgb((c-3)*-1);  gsum+=g_rgb((c-3)*-1);  bsum+=b_rgb((c-3)*-1);
  }  
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  
  fprintf(bt,"Landry %d,%d,%d,%d\r\n",rm_cnt,rsum,gsum,bsum); 
  tslp_tsk(dly);
 
  if(abs(((rsum/10000)-(gsum/10000))*10000)<=40000 && rsum/10000+gsum/10000+bsum/10000<=10 && rsum+gsum+bsum>32000){
    Lan_col[rm_cnt]=BLACK;
    ev3_speaker_play_tone(262,300);
    ev3_lcd_draw_string("b",30*rm_cnt,80);
    fprintf(bt,"       BLACK\r\n"); 
  }
  else if(rsum>50000 && (rsum/10000)/(gsum/10000)>=4 && (rsum/10000)/(bsum/10000)>=8){
    Lan_col[rm_cnt]=RED;
    ev3_speaker_play_tone(523,300);
    ev3_lcd_draw_string("r",30*rm_cnt,80);
    fprintf(bt,"       RED\r\n"); 
  }
  else if(rsum>50000 && (rsum/10000)/(gsum/10000)>=1 && (rsum/10000)/(bsum/10000)>=6){
    Lan_col[rm_cnt]=YELLOW;
    ev3_speaker_play_tone(1046,300);
    ev3_lcd_draw_string("y",30*rm_cnt,80);
    fprintf(bt,"       YELLOW\r\n"); 
  }
  else if(abs(rsum-gsum)<20000&&abs(rsum-bsum)<16000&&abs(gsum-bsum)<15000&&gsum/rsum<3){
    Lan_col[rm_cnt]=UNKNOWN;
    ev3_lcd_draw_string("u",30*rm_cnt,80);
    fprintf(bt,"       UNKNOWN\r\n"); 
  }
  forward(20,40,1,1);
}

void pick_landry(int c){
  colorforward(20,40,0);
  spin(c*-1+1,90);
  Line(20,150,0,0,0);
  back(30,260,0);
  // crossLine(15,100,rf,0,0,1);
  // back(12,150,0);
  crossLine(14,230,rf,0,30,1);
  tslp_tsk(500);
  ev3_motor_reset_counts(1);
  while(abs(ev3_motor_get_counts(1))< 90*(Wdis/Wdia-(c*-0.01+0.2))){
    ev3_motor_set_power(1,(c*-2+1)*30);  ev3_motor_set_power(2,(c*-2+1)*30);
  }	
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);
  // spin(c*-1+1,90);
  tslp_tsk(500);
  back(12,60,0);
  tslp_tsk(500);

  arm(15,120);
  ev3_motor_set_power(0,15);  tslp_tsk(400);  ev3_motor_stop(0,true);
  tslp_tsk(200);
  arm(12,-70);
  tslp_tsk(400);
  forward(10,85,0,0);
  arm(25,45);//アームを開いてマーキングを飛ばす
  
  if(Lan_col[rm_cnt]!=UNKNOWN){ 
    forward(10,120,0,0);
    back(8,30,0);

    ev3_motor_reset_counts(3);
    val=0;
    if(Lan_col[rm_cnt]==RED)  val=0;
    else if(Lan_col[rm_cnt]==YELLOW)  val=1;
    else if(Lan_col[rm_cnt]==BLACK)  val=2;

    ev3_motor_reset_counts(3);
    while(abs(ev3_motor_get_counts(3)) < 140){  ev3_motor_set_power(3,15);  }
    ev3_motor_reset_counts(3);
    while(abs(ev3_motor_get_counts(3)) < 290){
      if(Lan_pos[val]==1 || Lan_col[rm_cnt]==UNKNOWN) break;
      ev3_motor_set_power(3,15*(-1+Lan_pos[val]));
    }
    ev3_motor_stop(3,true);

    arm(6,-120);//アームを最大まで上げる
    tslp_tsk(300);

    arm(10,100);
    ev3_motor_set_power(0,20);  tslp_tsk(400);  ev3_motor_stop(0,true);  tslp_tsk(200);
    arm(10,-35);

    ev3_motor_reset_counts(3);
    val=0;
    if(Lan_col[rm_cnt]==RED)  val=0;
    else if(Lan_col[rm_cnt]==YELLOW)  val=1;
    else if(Lan_col[rm_cnt]==BLACK)  val=2;
  
    while(abs(ev3_motor_get_counts(3)) < 290){
      if(Lan_pos[val]==1 || Lan_col[rm_cnt]==UNKNOWN) break;
      ev3_motor_set_power(3,-15*(-1+Lan_pos[val]));
    }
    ev3_motor_reset_counts(3);  while(abs(ev3_motor_get_counts(3)) < 140){  ev3_motor_set_power(3,-15);  }  ev3_motor_stop(3,true);
    tslp_tsk(300);
  }
  else{  
    forward(14,90,0,0);

    ev3_motor_set_power(0,20);  tslp_tsk(200);  ev3_motor_stop(0,true);  tslp_tsk(200);
    arm(10,-28);
  }
}

void put_landry(){
  arm(17,120);
  ev3_motor_set_power(0,15);  tslp_tsk(300);  ev3_motor_stop(0,true);  tslp_tsk(400);
  arm(12,-21);
  ev3_motor_reset_counts(3);
  while(abs(ev3_motor_get_counts(3)) < 144){  ev3_motor_set_power(3,15);  }  ev3_motor_stop(3,true);  tslp_tsk(300);

  forward(10,130,0,0);
  crossLine(20,120,rf,0,0,0);  tslp_tsk(700);

  forward(15,82,0,1);  tslp_tsk(400);
  back(10,82,0);
  forward(20,82,0,1);  tslp_tsk(400);

  back(20,150,0);
  ev3_motor_set_power(0,15);  tslp_tsk(200);  ev3_motor_stop(0,true);
  tslp_tsk(200);
  arm(12,-85);
  tslp_tsk(200);
  forward(15,100,0,0);
  
  ev3_motor_reset_counts(1);
  while(ev3_motor_get_counts(1)<  20*Wdis/Wdia){
    ev3_motor_set_power(1,15);  ev3_motor_set_power(2,15);
  } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);

  ev3_motor_reset_counts(2);
  while(ev3_motor_get_counts(2)>  -30*Wdis/Wdia +3.2){
    ev3_motor_set_power(1,-15);  ev3_motor_set_power(2,-15);
  }	  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);

  ev3_motor_reset_counts(1);
  while(ev3_motor_get_counts(1)<  30*Wdis/Wdia){
    ev3_motor_set_power(1,15);  ev3_motor_set_power(2,15);
  } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);

  arm(15,40);

  back(15,150,0);
  arm(20,-140);
  rs(180);
  back(20,140,1);wall(300);
  Line(50,100,1,1,0);
  forward(pw,220,0,0);
  rs(51);
  // back(10,12,0);
}

void read_marking(){
  ev3_motor_reset_counts(2);
  int rsm[2]={0,0};  int gsm[2]={0,0};  int bsm[2]={0,0};

  while(ev3_motor_get_counts(2)<360.0/270*70){
    ev3_motor_set_power(1,-20);  ev3_motor_set_power(2,20);
    rsm[0]+=r_rgb(2);  gsm[0]+=g_rgb(2);  bsm[0]+=b_rgb(2);
    rsm[1]+=r_rgb(3);  gsm[1]+=g_rgb(3);  bsm[1]+=b_rgb(3);
  }  
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  
  fprintf(bt,"Marking %d,%d,%d\r\n",rsm[0],gsm[0],bsm[0]); 
  fprintf(bt,"        %d,%d,%d\r\n",rsm[1],gsm[1],bsm[1]); 
  tslp_tsk(dly);
  
  for(int i=0; i<2; i++){
    if(rsm[i]>2000 && gsm[i]/rsm[i]<2){
      ev3_lcd_draw_string("white",70*i,50);
      gw_col[i]=WHITE;
    }
    else{
      ev3_lcd_draw_string("green",70*i,50);
      gw_col[i]=GREEN;
    }
  }
  back(15,150,0);
  crossLine(20,120,rf,0,ad,1);
}

void pick_bottle(){
  back(10,120,0);
  arm(20,130);
  ev3_motor_reset_counts(0);  ev3_motor_set_power(0,20);  tslp_tsk(400);  ev3_motor_stop(0,true);
  arm(10,-57);
  
  tslp_tsk(200);
  Line(14,150,0,0,0);
  arm(8,-40);
  back(10,125,0);
  ev3_motor_reset_counts(2);  rotation=-14*Wdis/Wdia +3;
  while(ev3_motor_get_counts(2)> rotation){
    ev3_motor_set_power(1,-30+23*ev3_motor_get_counts(2)/rotation);
    ev3_motor_set_power(2,-30+23*ev3_motor_get_counts(2)/rotation);
  }	ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);
  arm(8,40);
  forward(10,93,0,0);
  arm(7,-65);
  back(10,100,0);
  ev3_motor_reset_counts(1);
  while(ev3_motor_get_counts(1)< 15*Wdis/Wdia){
    ev3_motor_set_power(1,30-23*ev3_motor_get_counts(1)/rotation);
    ev3_motor_set_power(2,30-23*ev3_motor_get_counts(1)/rotation);
  }	ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);
}

void put_bottle(int rl,int fs){
  arm(7,-11);
  if(rl==2){
    forward(20,15,0,1);
    ev3_motor_reset_counts(1);
    while(abs(ev3_motor_get_counts(1))<Wdis/Wdia*150){
      ev3_motor_set_power(1,-30);
    }
    ev3_motor_stop(1,true);
    forward(10,79,0,1);  tslp_tsk(300);
    back(10,99,0);
    arm(1,12);  tslp_tsk(100);
    arm(7,10);  tslp_tsk(500);
    arm(10,10); tslp_tsk(200);
    back(10,62,0);
    arm(8,-42);
    ls(80);
    forward(10,10,1,1);tslp_tsk(300);ev3_motor_stop(1,true);tslp_tsk(300);ev3_motor_stop(2,true);
    back(10,166,0);

    ev3_motor_reset_counts(2);
    while(abs(ev3_motor_get_counts(2))<Wdis/Wdia*125){
      ev3_motor_set_power(2,30);
    }
    ev3_motor_stop(2,true);
    forward(15,35,0,1);
    arm(1,13);
    tslp_tsk(300);
    arm(8,40);
    back(10,50,0);
    ev3_motor_reset_counts(2);
    while(ev3_motor_get_counts(2)> -10*Wdis/Wdia +3.2){
      ev3_motor_set_power(1,-30+23*ev3_motor_get_counts(2)/rotation);
      ev3_motor_set_power(2,-30+23*ev3_motor_get_counts(2)/rotation);
    }	  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);
    back(10,60,0);
    ls(70);

    while(ev3_color_sensor_get_reflect(1)>rf){
      ev3_motor_set_power(1,40);    ev3_motor_set_power(2,40);
    } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);
    tslp_tsk(dly);

  }
  //////////////////
  else if(fs==0){
    forward(20,18,0,1);
    ev3_motor_reset_counts(1);
    while(abs(ev3_motor_get_counts(1))<Wdis/Wdia*150){
      ev3_motor_set_power(1,(rl*2-1)*30);
    }
    ev3_motor_stop(1,true);
    forward(10,79,0,1);
    tslp_tsk(300);
    back(10,99,0);
    arm(1,12);
    tslp_tsk(100);
    arm(7,10);
    tslp_tsk(500);
    arm(10,10);
    back(10,55,0);
    arm(8,-42);
    spin(rl*-1+1,80);
    forward(10,10,1,1);tslp_tsk(300);ev3_motor_stop(1,true);tslp_tsk(300);ev3_motor_stop(2,true);
    back(10,250,0);
  }
  /////////////////////  rl 0,1
  else if(fs==1){
    ev3_motor_reset_counts(2);
    while(abs(ev3_motor_get_counts(2))<Wdis/Wdia*125){
      ev3_motor_set_power(2,(rl*2-1)*30);
    }
    ev3_motor_stop(2,true);
    forward(15,35,0,1);
    arm(1,13);
    tslp_tsk(300);
    arm(8,40);
    back(10,125,0);
    spin(rl,70);
    while(ev3_color_sensor_get_reflect(0)>rf){
      ev3_motor_set_power(1,(rl*2-1)*30);    ev3_motor_set_power(2,(rl*2-1)*30);
    } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);
    tslp_tsk(dly);

  }
  ev3_motor_reset_counts(0);
  ev3_motor_set_power(0,20);  tslp_tsk(400);  ev3_motor_stop(0,true);
  tslp_tsk(200);
  arm(20,-150);
  Line(20,220,0,1,1);
  rs(180);
}

void move_ball(int c){
  arm(5,-10);
  forward(15,198,0,0);
  arm(10,-40);
  // arm(6,-25);//アームを壁より高いところまで上げる
  // tslp_tsk(300);
  // arm(10,-20);//アームを壁より高いところまで上げる
  tslp_tsk(450);
  back(8,30,0);
  arm(8,-30);//アームを壁より高いところまで上げる
  forward(15,34,0,0);
  spin(c,127);
  back(10,160,0);
  arm(7,-45);//アームを最大まで上げる
  arm(10,24);
  tslp_tsk(500);
  arm(20,-20);
  forward(20,30,0,0);
  spin(c,60);
}


void SGtoly2(){
  ev3_motor_reset_counts(2);  rotation=-45*Wdis/Wdia +10;
  while(ev3_motor_get_counts(2)> rotation){
    ev3_motor_set_power(1,-30+23*ev3_motor_get_counts(2)/rotation);
    ev3_motor_set_power(2,-30+23*ev3_motor_get_counts(2)/rotation);
  } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);
  forward(pw,108,0,1);
  crossLine(19,60,rf,0,0,0);
}


void ly2tobt2(){
  forward(70,400,1,0);
  tslp_tsk(dly);
  crossLine(17,100,rf,0,0,0);
  tslp_tsk(200);
}


void bt2togr2(){
  ev3_motor_reset_counts(1);
  while(abs(ev3_motor_get_counts(1))<Wdis/Wdia*160){
    ev3_motor_set_power(1,-30);
  }
  colorforward(pw,20,0);
  rs(90);
  crossLine(50,50,rf,0,45,0);
  ls(90);
  crossLine(20,200,rf,0,50,0);
}

void bt2togr3(){
  ev3_motor_reset_counts(1);
  while(abs(ev3_motor_get_counts(1))<Wdis/Wdia*160){
    ev3_motor_set_power(1,-30);
  }
  colorforward(33,20,0);
  rs(90);
  crossLine(50,50,rf,0,45,0);
  ls(55);
  while(ev3_color_sensor_get_reflect(0)>rf){
    ev3_motor_set_power(1,20);    ev3_motor_set_power(2,20);
  } ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);
  ls(7);
  Line(20,40,1,1,1);
  Line(50,220,1,1,1);
  crossLine(20,100,rf,0,50,1);
}

void bt2toby3(){
  ls(130);
  colorforward(40,30,0);
  rs(40);
  Line(20,30,1,1,1);
  Line(50,250,1,1,1);
  crossLine(20,100,rf,0,50,1);
}


void gr2toby3(){
  crossLine(pw,170,rf,0,0,0);
  tslp_tsk(400);
  ls(32);
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);  ev3_motor_set_power(1,0);  ev3_motor_set_power(2,0);  tslp_tsk(100);
  while(ev3_motor_get_counts(2)<360/270*500){
    ev3_motor_set_power(1,-50);  ev3_motor_set_power(2,33/*38*/);
  }
  while(ev3_color_sensor_get_reflect(0)>rf){
    ev3_motor_set_power(1,-30);  ev3_motor_set_power(2,30);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true); 
  tslp_tsk(dly); 
  ls(28);
  crossLine(pw,300,rf,0,40,0);
}

void gr3toSG(){
  crossLine(pw,250,rf,0,0,0);
  ls(28);
  forward(40,250,1,0);
  colorforward(30,28,0);
  ls(70);
}

void gr3toby2(){
  crossLine(pw,320,rf,0,0,0);
  tslp_tsk(400);
  ls(33);
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);  ev3_motor_set_power(1,0);  ev3_motor_set_power(2,0);  tslp_tsk(100);
  while(ev3_motor_get_counts(2)<360/270*700){
    ev3_motor_set_power(1,-50);  ev3_motor_set_power(2,34/*38*/);
  }
  while(ev3_color_sensor_get_reflect(0)>rf){
    ev3_motor_set_power(1,-20);  ev3_motor_set_power(2,20);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true); 
  tslp_tsk(dly); 
  ls(28);
  crossLine(pw,50,rf,0,40,0);
}

void gr3toby3(){
  crossLine(pw,320,rf,0,0,0);
  tslp_tsk(400);
  ls(32);
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);  ev3_motor_set_power(1,0);  ev3_motor_set_power(2,0);  tslp_tsk(100);
  while(ev3_motor_get_counts(2)<360/270*500){
    ev3_motor_set_power(1,-50);  ev3_motor_set_power(2,33/*38*/);
  }
  while(ev3_color_sensor_get_reflect(0)>rf){
    ev3_motor_set_power(1,-20);  ev3_motor_set_power(2,20);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true); 
  tslp_tsk(dly); 
  ls(28);
  crossLine(pw,300,rf,0,40,0);
}


void by2togr3(){
  crossLine(pw,190,rf,0,0,0);
  tslp_tsk(400);
  rs(24);
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);  ev3_motor_set_power(1,0);  ev3_motor_set_power(2,0);  tslp_tsk(100);
  while(ev3_motor_get_counts(2)<360/270*900){
    ev3_motor_set_power(1,-40);  ev3_motor_set_power(2,54);
  }
  while(ev3_color_sensor_get_reflect(0)>rf){
    ev3_motor_set_power(1,-35);  ev3_motor_set_power(2,35);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true); 
  forward(10,20,0,0);
  tslp_tsk(dly);
  crossLine(pw,200,rf,0,40,0);}

void by3toSG(){
  crossLine(pw,250,rf,0,ad,0);
  forward(40,380,0,0);
  
  ev3_motor_reset_counts(2);  rotation=-90*Wdis/Wdia;
  while(ev3_motor_get_counts(2)> rotation){
      ev3_motor_set_power(1,-50+10*ev3_motor_get_counts(2)/rotation);
      ev3_motor_set_power(2,-50+10*ev3_motor_get_counts(2)/rotation);
  }	
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);  tslp_tsk(dly);
}

void by3togr3(){
  crossLine(pw,320,rf,0,0,0);
  tslp_tsk(400);
  rs(33);
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);  ev3_motor_set_power(1,0);  ev3_motor_set_power(2,0);  tslp_tsk(100);
  while(ev3_motor_get_counts(2)<360/270*700){
    ev3_motor_set_power(1,-34);  ev3_motor_set_power(2,50);
  }
  while(ev3_color_sensor_get_reflect(0)>rf){
    ev3_motor_set_power(1,-20);  ev3_motor_set_power(2,20);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true); 
  tslp_tsk(dly); 
  rs(28);
  crossLine(pw,300,rf,0,40,0);
}


////////////////////////////////////////////////////////////////////////////////////////
void L(int dis, int change, int brake){
  ev3_motor_reset_counts(2);
  rotation=dis*360.0/(Wdia*10*PI);
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;
  int p=0;

  Kp=0.35;  Kd=10;
  while(ev3_motor_get_power(2) < 55 && abs(angle) < rotation){
    error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
    delivetive = error - lasterror;
    steer = Kp*error + Kd*delivetive;

    if(p<=100) p+=18;
    ev3_motor_set_power(1,-1*(p + steer*1.2));
    ev3_motor_set_power(2,p - steer*1.2);
    
    angle = ev3_motor_get_counts(2);
    lasterror = error;
    if(abs(angle) >= rotation) {break; ev3_speaker_play_tone(2000,1000);}
    fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
    tslp_tsk(1);
  }
  if(change == 1){
    fprintf(bt,"100\r\n");
    fprintf(bt,"0\r\n");

    Kp=0.1;  Kd=8;
    while(abs(angle) < rotation){
      error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
      delivetive = error - lasterror;
      steer = Kp*error + Kd*delivetive;

      ev3_motor_set_power(1,-1*(80 + steer));
      ev3_motor_set_power(2,80 - steer);
    
      angle = ev3_motor_get_counts(2);
      lasterror = error;
      fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
      tslp_tsk(1);
    }
  }

  else{
  fprintf(bt,"100\r\n");
  fprintf(bt,"0\r\n");

  Kp=0.1;  Kd=8;
  while(abs(angle) < rotation-70){
    error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
    delivetive = error - lasterror;
    steer = Kp*error + Kd*delivetive;

    ev3_motor_set_power(1,-1*(80 + steer));
    ev3_motor_set_power(2,80 - steer);
    
    angle = ev3_motor_get_counts(2);
    lasterror = error;
    fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
    tslp_tsk(1);
  }

  fprintf(bt,"100\r\n");
  fprintf(bt,"0\r\n");
  
  Kp=0.1;  Kd=7;
  while(abs(angle) < rotation){
    error = r_rgb(0) - r_rgb(1);//ev3_color_sensor_get_reflect(0) - ev3_color_sensor_get_reflect(1);
    delivetive = error - lasterror;
    steer = Kp*error + Kd*delivetive;

    ev3_motor_set_power(1,-1*(-5 + steer));
    ev3_motor_set_power(2,-5 - steer);
    
    angle = ev3_motor_get_counts(2);
    lasterror = error;
    fprintf(bt,"%d\r\n",(int)ev3_motor_get_power(2));
    tslp_tsk(1);
  }
  }
  
  if(brake == 0){
    ev3_motor_stop(1,true);  
    ev3_motor_stop(2,true);
  }

}

void turn(int brake, int port){
  ev3_motor_reset_counts(port);

  while(abs(ev3_motor_get_counts(port)) < 180*Wdis/Wdia-50){
    ev3_motor_set_power(port,70*(port-2/port));
    ev3_motor_set_power(3-port,0);
  }	
  
  if(brake == 0)
    ev3_motor_stop(port,true);
}


void forward(float power,float dis,int brake,int change){
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  int i=0;
  // モーターリセット
  rotation=dis*360/Wdia/10/3.141592;
  // 回したい角度＝進みたい距離÷(車輪の直径×10π)×360
    while(ev3_motor_get_counts(2)<rotation){
      // 目標距離に達するまで
      if(change==0){
        if(ev3_motor_get_counts(2)>rotation*0.93)  {
          ev3_motor_set_power(1,-10);  ev3_motor_set_power(2,10);}
        else if(ev3_motor_get_counts(2)>rotation*0.9)  {
          ev3_motor_set_power(1,0);  ev3_motor_set_power(2,0);}
        else{ev3_motor_set_power(1,-1.07*power);  if(i==0)tslp_tsk(10);  ev3_motor_set_power(2,power);}
      }
      else{
        ev3_motor_set_power(1,-1.07*power);  if(i==0)tslp_tsk(10);  ev3_motor_set_power(2,power);
      }
  if(i==0)i=1;
  }
  //ev3_motor_set_power(1,5);  ev3_motor_set_power(2,-5);
  if(brake==0){ev3_motor_stop(1,true);  ev3_motor_stop(2,true);}
  tslp_tsk(dly);
  // 何秒とまるか　1000=1秒
}

void colorforward(float power,float add,int brake){
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  while(ev3_color_sensor_get_reflect(0)>rf){
    ev3_motor_set_power(1,-power);    ev3_motor_set_power(2,power);
  }
  ev3_motor_reset_counts(2);
  while(ev3_motor_get_counts(2)<add*360/(Wdia*10*PI)){
    ev3_motor_set_power(1,-power);    ev3_motor_set_power(2,power);
  }
  if(brake==0){ev3_motor_stop(1,true);  ev3_motor_stop(2,true);}
  tslp_tsk(dly);
}

void back(float power,float dis,int brake){
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  int cnt=0;
  rotation=360*dis/270;
  // 進みたい距離÷（車輪の直径×10π）
  tslp_tsk(200);
    while(ev3_motor_get_counts(1)<rotation){
      if(cnt==0) cnt++;
    // if(ev3_motor_get_counts(1)>rotation*0.85)
    //   {ev3_motor_set_power(1,10);  ev3_motor_set_power(2,-10);}
    // else{
      ev3_motor_set_power(1,1.04*power);
      // if(cnt==1) tslp_tsk(10);
      ev3_motor_set_power(2,-power);
    // }
    tslp_tsk(1);
  }
  //ev3_motor_set_power(1,-5);  ev3_motor_set_power(2,5);
  if(brake==0){ev3_motor_stop(1,true);  ev3_motor_stop(2,true);}
  tslp_tsk(dly);
}

void rs(float deg){
  ev3_motor_reset_counts(2);  rotation=-deg*Wdis/Wdia +10;//+pow(deg/10,4)/3000;//-(90-deg)/7 ;//-(deg/45-1.5)
  while(ev3_motor_get_counts(2)> rotation){// 機体の直径/車輪の直径*deg+補正する分    deg*機体の直径/360*PI/車輪の直径/PI*360
    // if(ev3_motor_get_counts(2)< rotation*deg/500){
      ev3_motor_set_power(1,-30+23*ev3_motor_get_counts(2)/rotation);
      ev3_motor_set_power(2,-30+23*ev3_motor_get_counts(2)/rotation);
    // }
    // else{
    //   ev3_motor_set_power(1,-40);  ev3_motor_set_power(2,-40);
    // }
  }	
  ev3_motor_stop(1,true);
  ev3_motor_stop(2,true);
  tslp_tsk(dly);
}

void ls(float deg){
  ev3_motor_reset_counts(1);  rotation=deg*Wdis/Wdia-10;//deg*(Wdis/Wdia-deg/4050);
  while(ev3_motor_get_counts(1)< rotation){// 機体の直径/車輪の直径*deg    deg*機体の直径/360*PI/車輪の直径/PI*360
    ev3_motor_set_power(1,30-23*ev3_motor_get_counts(1)/rotation);
    ev3_motor_set_power(2,30-23*ev3_motor_get_counts(1)/rotation);
  }	
  ev3_motor_stop(1,true);
  ev3_motor_stop(2,true);
  tslp_tsk(dly);
}

void spin(int rl,float deg){
  if(rl==0) rs(deg);
  else if(rl==1) ls(deg);
}

void Line(int power,float dis,int brake,int pd,int change){
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  rotation=dis*360.0/(Wdia*10*PI);  Kp=0.3;  Kd=6;
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;  
  while(abs(angle) < rotation){
    if(pd==0){if(ev3_motor_get_counts(2)<80){Kp=0.6;  Kd=12;}  else {Kp=0.2;  Kd=6;}} else{Kp=0.3;  Kd=13;}

    error=ev3_color_sensor_get_reflect(0)  -  ev3_color_sensor_get_reflect(1);
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

    if(change==0){
      if(ev3_motor_get_counts(1)>rotation*0.90){
        ev3_motor_set_power(1,10+steer);
        ev3_motor_set_power(2,-10-steer);
      }
      else{
        ev3_motor_set_power(1,-1*(power + steer));
        ev3_motor_set_power(2,power - steer);
      }
    }
    else{
      ev3_motor_set_power(1,-1*(power + steer));
      ev3_motor_set_power(2,power - steer);
    }
    
    angle=ev3_motor_get_counts(2);
    lasterror=error;
    tslp_tsk(1);
  }
  if(brake==0){ev3_motor_stop(1,true);  ev3_motor_stop(2,true);}
  tslp_tsk(dly);
}

void crossLine(float power,float dis,float ref,int brake,float add,int pd){
  ev3_motor_reset_counts(1);  ev3_motor_reset_counts(2);
  rotation=dis*360/273.32;
  // 
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;  
  while(angle < rotation){
    if(pd==0){if(ev3_motor_get_counts(2)<150){Kp=0.55;  Kd=14;}  else {Kp=0.2;  Kd=6;}}  else{Kp=0.3; Kd=13;}
      //kp kdで制御する 
    error=ev3_color_sensor_get_reflect(0)  -  ev3_color_sensor_get_reflect(1);
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

    if(ev3_motor_get_counts(1)>rotation*0.9)
      {ev3_motor_set_power(1,15+steer);  ev3_motor_set_power(2,-15-steer);}
    else{ ev3_motor_set_power(1,-1*(power + steer));  ev3_motor_set_power(2,power - steer);}

    angle=ev3_motor_get_counts(2);
    lasterror=error;
    tslp_tsk(1);
  }
  while(ev3_color_sensor_get_reflect(0)>ref && ev3_color_sensor_get_reflect(1)>ref){
    error=ev3_color_sensor_get_reflect(0)  -  ev3_color_sensor_get_reflect(1);
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;
    ev3_motor_set_power(1,-1*(power + steer)*0.75);
    ev3_motor_set_power(2,(power*0.75 - steer)*0.75);
    lasterror=error;
    tslp_tsk(1);
  }
  if(brake==0){ev3_motor_stop(1,true);  ev3_motor_stop(2,true);}
  if(add>0){ev3_motor_reset_counts(2); while(ev3_motor_get_counts(2)<360/273.32*add){ev3_motor_set_power(1,-power*0.8*1.1);  ev3_motor_set_power(2,power*0.8);}  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);}
  else if(add<0){ev3_motor_reset_counts(2); while(ev3_motor_get_counts(2)>360/273.32*add){ev3_motor_set_power(1,power*0.8);  ev3_motor_set_power(2,-power*0.8);}  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);}
  tslp_tsk(dly);
}

void singleLine(int power,int distance){
  ev3_motor_reset_counts(1);
  rotation=distance*360/(Wdia*10*PI);  Kp=0.3;  Kd=11;  int red=27;
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;

  while(r_rgb(0)>red){
    ev3_motor_set_power(1,-20);  ev3_motor_set_power(2,-20);
  }	tslp_tsk(dly);

  while(abs(angle) < rotation){
    error=r_rgb(0)-red;
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

    ev3_motor_set_power(1,-1*(power - steer));
    ev3_motor_set_power(2,power + steer);
    
    angle=ev3_motor_get_counts(1);
    lasterror=error;
    tslp_tsk(1);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);
  tslp_tsk(dly);
}

void singlecrossLine(int power,int distance){
  ev3_motor_reset_counts(1);
  rotation=distance*360/(Wdia*10*PI);  Kp=0.3;  Kd=11;  int red=27;
  angle=0;  error=0;  lasterror=0;  delivetive=0;  steer=0;

  while(r_rgb(0)>red){
    ev3_motor_set_power(1,-25);  ev3_motor_set_power(2,-25);
  }	tslp_tsk(dly);

  while(abs(angle) < rotation){
    error=r_rgb(0)-red;
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

    ev3_motor_set_power(1,-1*(power - steer));
    ev3_motor_set_power(2,power + steer);
    
    angle=ev3_motor_get_counts(1);
    lasterror=error;
    tslp_tsk(1);
  }
  while(r_rgb(1)>red){
    error=r_rgb(0)-red;
    delivetive=error-lasterror;
    steer=Kp*error+Kd*delivetive;

    ev3_motor_set_power(1,-1*(power - steer)*0.5);
    ev3_motor_set_power(2,(power + steer)*0.5);
    
    lasterror=error;
    tslp_tsk(1);
  }
  ev3_motor_stop(1,true);  ev3_motor_stop(2,true);
  tslp_tsk(dly);
}

void backLine(int power,int distance){
  ev3_motor_reset_counts(1);
}

void arm(int power,int angle){
  int cnt=0;
  ev3_motor_reset_counts(0);
  while(abs(ev3_motor_get_counts(0))<Wdis/Wdia*abs(angle)){
    cnt++;
    if(angle>0)  ev3_motor_set_power(0,power);
    else if(angle<0) ev3_motor_set_power(0,-power);
  }
  ev3_motor_stop(0,true);
  
}

void wall(int time){
  // 壁打ちのやつ
  tslp_tsk(time/2);
  ev3_motor_stop(1,true);
  tslp_tsk(time/2);
  ev3_motor_stop(2,true);
}

float sign(float n){
  return (n>0)-(n<0);
}

void color_rgb(int num){
  ev3_color_sensor_get_rgb_raw(num,&rgb_val);
  rval=rgb_val.r;
  gval=rgb_val.g;
  bval=rgb_val.b;
}

void dp_rgb(int num){
  color_rgb(num);
  fprintf(bt,"******\n");
  fprintf(bt,"%d\r\n",rval);
  fprintf(bt,"%d\r\n",gval);
  fprintf(bt,"%d\r\n",bval);
  tslp_tsk(10);
}

int r_rgb(int num){
  color_rgb(num);
  return rgb_val.r;
}

int g_rgb(int num){
  color_rgb(num);
  return rgb_val.g;
}

int b_rgb(int num){
  color_rgb(num);
  return rgb_val.b;
}

void cfg_bluetooth(){
  bt = ev3_serial_open_file(EV3_SERIAL_BT);  //bluetoothポートのオープン
  assert(bt != NULL);   //接続チェック
}

void config(){
  ev3_motor_config (1,  MEDIUM_MOTOR);
  ev3_motor_config (2,  MEDIUM_MOTOR);
  ev3_motor_config (0,  MEDIUM_MOTOR);
  ev3_motor_config (3,  MEDIUM_MOTOR);
	ev3_sensor_config(0, COLOR_SENSOR);
	ev3_sensor_config(1, COLOR_SENSOR);
  ev3_sensor_config(2, COLOR_SENSOR);
  ev3_sensor_config(3, COLOR_SENSOR);
}
