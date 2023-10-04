/*
 * kinematika itu ada dua yaitu forward kinematik dan invers kinematik
 * forward kinematik itu untuk menentukan posisi dari robot
 * inverse kinematik itu untuk merencanakan navigasi jalur untuk pergerakan
 * pokoknya fungsi inverse untuk pergerakan robot, sedangkan fungsi forward untuk posisi robot
 * belajar interrput sama library timerone
 * untuk akses encoder kalau masih bingung cari referensinya banyak di google
 * untuk mengetahui rumusnya silahkan lihat jurnal yang sudah tak upload
 * untuk motor bagi 4 roda omni : motor 1 motor depan sebelah kanan, motor 2 depan sebelah kiri, motor 3 belakang sebelah kiri, motor 4 belakang sebelah kanan
 * untuk motor bagi 3 roda omni : motor 1 motor depan sebelah kanan, motor 2 depan sebelah kiri, motor 3 belakang
 * inverse3 : inverse kinematik roda 3
 * inverse4 : inverse kinematik roda 4
 * forward2 : forward kinematik 2 encoder
 * forward3 : forward kinematik 3 encoder
 * forward4 : forward kinematik 4 encoder
 * pin bisa dirubah tergatung kebutuhan
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <HMC5883L.h>
HMC5883L compass;
LiquidCrystal_I2C lcd (0x27, 16, 2);  //adddress tergantung modul I2C 
#define set_calib 0
#define LPWM1 7
#define RPWM1 6
#define Len1 24
#define Ren1 26
#define LPWM2 9
#define RPWM2 8
#define Len2 28
#define Ren2 30
#define LPWM3 4
#define RPWM3 5
#define Len3 27
#define Ren3 25
#define LPWM4 2
#define RPWM4 3
#define Len4 29
#define Ren4 31
int phi,n;
long a,b,c,d;
int x,y,theta,xg,yg,angle,head;
float alfa,xr,yr;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(115200);
  Serial1.end();Serial2.end();
  lcd.begin();
  lcd.backlight();
  pinMode(LPWM1, OUTPUT);pinMode(RPWM1, OUTPUT);
  pinMode(Len1, OUTPUT);pinMode(Ren1, OUTPUT);
  pinMode(LPWM2, OUTPUT);pinMode(RPWM2, OUTPUT);
  pinMode(Len2, OUTPUT);pinMode(Ren2, OUTPUT);
  pinMode(LPWM3, OUTPUT);pinMode(RPWM3, OUTPUT);
  pinMode(Len3, OUTPUT);pinMode(Ren3, OUTPUT);
  pinMode(LPWM4, OUTPUT);pinMode(RPWM4, OUTPUT);
  pinMode(Len4, OUTPUT);pinMode(Ren4, OUTPUT);
  digitalWrite(Len1, HIGH);digitalWrite(Ren1, HIGH);
  digitalWrite(Len2, HIGH);digitalWrite(Ren2, HIGH);
  digitalWrite(Len3, HIGH);digitalWrite(Ren3, HIGH);
  digitalWrite(Len4, HIGH);digitalWrite(Ren4, HIGH);
  compass.begin();
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(0, 0);
  attachInterrupt(3,encoder4,CHANGE); //pin digital 19 ineterrupt ke 3, untuk metodenya bisa RISING atau CHANGE tapi lebih recommend CHANGE dari anak unisula
  pinMode(33,INPUT);
  digitalWrite(33,HIGH);
  attachInterrupt(2,encoder3,CHANGE); //pin digital 18 ineterrupt ke 2, untuk metodenya bisa RISING atau CHANGE tapi lebih recommend CHANGE dari anak unisula
  pinMode(35,INPUT);
  digitalWrite(35,HIGH);
  attachInterrupt(1,encoder2,CHANGE); //pin digital 3 ineterrupt ke 1, untuk metodenya bisa RISING atau CHANGE tapi lebih recommend CHANGE dari anak unisula
  pinMode(37,INPUT);
  digitalWrite(37,HIGH);
  attachInterrupt(0,encoder1,CHANGE); //pin digital 2 ineterrupt ke 0, untuk metodenya bisa RISING atau CHANGE tapi lebih recommend CHANGE dari anak unisula
  pinMode(39,INPUT);
  digitalWrite(39,HIGH);
//  Timer1.initialize(1000);  //ambil data 1000 microsecond
//  Timer1.attachInterrupt(fordward2);  //time interrupt, menyela program pada looping, kalau bingung samean cari referensi aja
//  Timer1.initialize(1000);  //ambil data 1000 microsecond
//  Timer1.attachInterrupt(fordward3);  //time interrupt, menyela program pada looping, kalau bingung samean cari referensi aja
  Timer1.initialize(1000);  //ambil data 1000 microsecond
  Timer1.attachInterrupt(fordward4);  //time interrupt, menyela program pada looping, kalau bingung samean cari referensi aja
}

void orientasi(){
  Vector norm = compass.readNormalize();
  float heading = atan2(norm.YAxis, norm.XAxis);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;
  if (heading < 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  float headingDegrees = heading * 180/M_PI; 
  n = headingDegrees - (90+set_calib);
  if(n<0){
    n = (n + 360);
    if(n<0){
      n = (n + 360);
    }
  }  
  alfa = (n/360)*6.283185;  //sudut derajat diganti ke radian
  if(n >= 0 && n <= 180){  // sudut derajat biasa diubah menjadi orientasi robot yaitu -180 sampai 180
    phi = map(n,0,180,0,180);
  }
  else if(n > 180 && n <= 359){
    phi = map(n,359,180,-1,-180);    
  }
}

void encoder1(){
  if(digitalRead(39)==1) {
    a--;
  }
  else {
    a++;
  }
}

void encoder2(){
  if(digitalRead(37)==1) {
    b--;
  }
  else {
    b++;
  }
}

void encoder3(){
  if(digitalRead(35)==1) {
    c--;
  }
  else {
    c++;
  }
}

void encoder4(){
  if(digitalRead(33)==1) {
    d--;
  }
  else {
    d++;
  }
}

void forward2(){
  //untuk 2 encoder sebenarnya sama dengan 4 encoder cuman yang dipakai hanya 2, tergantung dari penempatannya encoder menganut yang roda depan atau roda belakang untuk posisinya
  float en1,en2,en3,en4;
  orientasi();
  //rumus untuk pembacaan encoder dari satuan pulsa ke jarak per centimeter
  en1 = a*0.115018315;
  en2 = b*0.115018315;
  //rumus inverse untuk 2 encoder, disini berbeda dengan jurnal pada operasinya saja (min plus nya) karena mengikuti arah putaran dari roda tersebut
  xr += (en1/(4*cos(0.785398)))+(en2/(4*cos(0.785398)));  //0.785398 adalah nilai radian dari 45 derajat
  yr += (-en1/(4*sin(0.785398)))+(en2/(4*sin(0.785398)));  //0.785398 adalah nilai radian dari 45 derajat
  head = n;  //arah hadap robot
  //rumus mengubah posisi lokal robot ke posisi global/lapangan
  xg = xr*cos(alfa)+yr*sin(alfa);  //alfa adalah arah hadap robot yang diubah ke bentuk radian (dari derajat)
  yg = -xr*sin(alfa)+yr*cos(alfa);  //alfa adalah arah hadap robot yang diubah ke bentuk radian (dari derajat)
  head = n;  //arah hadap robot
}

void forward3(){
  float en1,en2,en3;
  orientasi();
  //rumus untuk pembacaan encoder dari satuan pulsa ke jarak per centimeter
  en1 = a*0.115018315;
  en2 = b*0.115018315;
  en3 = c*0.115018315;
  //rumus inverse untuk 3 encoder, disini berbeda dengan jurnal pada operasinya saja (min plus nya) karena mengikuti arah putaran dari roda tersebut
  xr += en1*sin(1.047198)+en2*sin(1.047198)-en3;  //1.047198 adalah nilai radian dari 60 derajat
  yr += -en1*cos(1.047198)+en2*cos(1.047198);  //1.047198 adalah nilai radian dari 60 derajat
  head = n;  //arah hadap robot
  //rumus mengubah posisi lokal robot ke posisi global/lapangan
  xg = xr*cos(alfa)+yr*sin(alfa);  //alfa adalah arah hadap robot yang diubah ke bentuk radian (dari derajat)
  yg = -xr*sin(alfa)+yr*cos(alfa);  //alfa adalah arah hadap robot yang diubah ke bentuk radian (dari derajat)
  head = n;  //arah hadap robot
}

void forward4(){
  float en1,en2,en3,en4;
  orientasi();
  //rumus untuk pembacaan encoder dari satuan pulsa ke jarak per centimeter
  en1 = a*0.115018315;
  en2 = b*0.115018315;
  en3 = c*0.115018315;
  en4 = d*0.115018315;
  //rumus inverse untuk 4 encoder, disini berbeda dengan jurnal pada operasinya saja (min plus nya) karena mengikuti arah putaran dari roda tersebut
  xr += (en1/(4*cos(0.785398)))+(en2/(4*cos(0.785398)))-(en3/(4*cos(0.785398)))-(en4/(4*cos(0.785398)));  //0.785398 adalah nilai radian dari 45 derajat
  yr += (-en1/(4*sin(0.785398)))+(en2/(4*sin(0.785398)))+(en3/(4*sin(0.785398)))-(en4/(4*sin(0.785398)));  //0.785398 adalah nilai radian dari 45 derajat
  head = n;  //arah hadap robot
  //rumus mengubah posisi lokal robot ke posisi global/lapangan
  xg = xr*cos(alfa)+yr*sin(alfa);  //alfa adalah arah hadap robot yang diubah ke bentuk radian (dari derajat)
  yg = -xr*sin(alfa)+yr*cos(alfa);  //alfa adalah arah hadap robot yang diubah ke bentuk radian (dari derajat)
  head = n;  //arah hadap robot
}

void mot_1(int kec){
  int v;
  if(kec<0){
    v = kec * -1;
    analogWrite(LPWM1,(v+1));
    digitalWrite(RPWM1,LOW);
  }
  if(kec>=0){
    analogWrite(RPWM1,(kec+1));
    digitalWrite(LPWM1,LOW);
  }
}
  
void mot_2(int kec){
  int v;
  if(kec<0){
    v = kec * -1;
    analogWrite(LPWM2,(v+1)); 
    digitalWrite(RPWM2,LOW);
  }
  if(kec>=0){
    analogWrite(RPWM2,(kec+1)); 
    digitalWrite(LPWM2,LOW);
  }
}

void mot_3(int kec){
  int v;
  if(kec<0){
    v = kec * -1;
    analogWrite(LPWM3,(v+1)); 
    digitalWrite(RPWM3,LOW);
  }
  if(kec>=0){
    analogWrite(RPWM3,(kec+1)); 
    digitalWrite(LPWM3,LOW);
  }
}

void mot_4(int kec){
  int v;
  if(kec<0){
    v = kec * -1;
    analogWrite(LPWM4,(v+1)); 
    digitalWrite(RPWM4,LOW);
  }
  if(kec>=0){
    analogWrite(RPWM4,(kec+1)); 
    digitalWrite(LPWM4,LOW);
  }
}

void inverse4(int x,int y,int theta){
  float v1,v2,v3,v4,vtot;
  int m1,m2,m3,m4;

//rumus
  v1 = ((x*10)*cos(0.785398))-((y*10)*sin(0.785398))+theta*2.245;  //0.785398 adalah nilai radian dari 45 derajat
  v2 = ((x*10)*cos(0.785398))+((y*10)*sin(0.785398))+theta*2.245;  //0.785398 adalah nilai radian dari 45 derajat
  v3 = ((-x*10)*cos(0.785398))+((y*10)*sin(0.785398))+theta*2.245;  //0.785398 adalah nilai radian dari 45 derajat
  v4 = ((-x*10)*cos(0.785398))-((y*10)*sin(0.785398))+theta*2.245;  //0.785398 adalah nilai radian dari 45 derajat
  
//dibawah ini untuk menghitung vtotal nya dari hasil yang diatas, karena setiap komponennya berbeda maka yang nilainya minus dikalikan -1
  if(v3<0 && v4<0){
    vtot = v1+v2-v3-v4;
  }
  else if(v1<0 && v4<0){
    vtot = -v1+v2+v3-v4;
  }
  else if(v1<0 && v2<0){
    vtot = -v1-v2+v3+v4;
  }
  else if(v2<0 && v3<0){
    vtot = v1-v2-v3+v4;
  }
  else if(v1<0){
    vtot = -v1+v2+v3+v4;
  }
  else if(v2<0){
    vtot = v1-v2+v3+v4;
  }
  else if(v3<0){
    vtot = v1+v2-v3+v4;
  }
  else if(v4<0){
    vtot = v1+v2+v3-v4;
  }
  else {
    vtot = v1+v2+v3+v4;
  }

//mengubah pwm dari kecepatan yang diketahui
  if(v1==0 && v2==0 && v3==0){
    m1 = (v1/vtot)*100;
    m2 = (v2/vtot)*100;
    m3 = (v3/vtot)*100;
    m4 = (v4/vtot)*100;
  }
  else if(v1==0 && v2==0 && v4==0){
    m1 = (v1/vtot)*100;
    m2 = (v2/vtot)*100;
    m3 = (v3/vtot)*100;
    m4 = (v4/vtot)*100;
  }
  else if(v1==0 && v3==0 && v4==0){
    m1 = (v1/vtot)*100;
    m2 = (v2/vtot)*100;
    m3 = (v3/vtot)*100;
    m4 = (v4/vtot)*100;
  }
  else if(v2==0 && v3==0 && v4==0){
    m1 = (v1/vtot)*100;
    m2 = (v2/vtot)*100;
    m3 = (v3/vtot)*100;
    m4 = (v4/vtot)*100;
  }
  else if(v1==0 && v2==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
    m4 = (v4/vtot)*200;
  }
  else if(v1==0 && v3==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
    m4 = (v4/vtot)*200;
  }
  else if(v1==0 && v4==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
    m4 = (v4/vtot)*200;
  }
  else if(v2==0 && v3==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
    m4 = (v4/vtot)*200;
  }
  else if(v2==0 && v4==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
    m4 = (v4/vtot)*200;
  }
  else if(v3==0 && v4==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
    m4 = (v4/vtot)*200;
  }
  else if(v1==0){
    m1 = (v1/vtot)*300;
    m2 = (v2/vtot)*300;
    m3 = (v3/vtot)*300;
    m4 = (v4/vtot)*300;
  }
  else if(v2==0){
    m1 = (v1/vtot)*300;
    m2 = (v2/vtot)*300;
    m3 = (v3/vtot)*300;
    m4 = (v4/vtot)*300;
  }
  else if(v3==0){
    m1 = (v1/vtot)*300;
    m2 = (v2/vtot)*300;
    m3 = (v3/vtot)*300;
    m4 = (v4/vtot)*300;
  }
  else if(v4==0){
    m1 = (v1/vtot)*300;
    m2 = (v2/vtot)*300;
    m3 = (v3/vtot)*300;
    m4 = (v4/vtot)*300;
  }
  else {
    m1 = (v1/vtot)*400;
    m2 = (v2/vtot)*400;
    m3 = (v3/vtot)*400;
    m4 = (v4/vtot)*400;
  }

//constrain untuk membuat nilai batas atas dan bawah, jika bingung bisa cari referensinya
  m1 = constrain(m1,-254,254);
  m2 = constrain(m2,-254,254);
  m3 = constrain(m3,-254,254);
  m4 = constrain(m4,-254,254);
 
//akses motor dari fungsi yang sudah ditentukan
  mot_1(m1);
  mot_2(m2);
  mot_3(m3);
  mot_4(m4);
}

void inverse3(int x,int y,int theta){
  float v1,v2,v3,vtot;
  int m1,m2,m3;

//rumus
  v1 = ((0.333333)*cos(1.047198)*(x*10))-((0.577350)*sin(1.047198)*(y*10))+((1.2)*theta);  //1.047198 adalah nilai radian dari 60 derajat
  v2 = ((0.333333)*cos(1.047198)*(x*10))+((0.577350)*sin(1.047198)*(y*10))+((1.2)*theta);  //1.047198 adalah nilai radian dari 60 derajat
  v3 = (-(0.666667)*cos(1.047198)*(x*10))+((1.2)*theta);  //1.047198 adalah nilai radian dari 60 derajat
  
//dibawah ini untuk menghitung vtotal nya dari hasil yang diatas, karena setiap komponennya berbeda maka yang nilainya minus dikalikan -1
  if (v1<0 && v2<0 && v3<0){
    vtot = -v1-v2-v3;
  }
  else if(v1<0 && v2<0){
    vtot = -v1-v2+v3;
  }
  else if(v1<0 && v3<0){
    vtot = -v1+v2-v3;
  }
  else if(v2<0 && v3<0){
    vtot = v1-v2-v3;
  }
  else if(v1<0){
    vtot = -v1+v2+v3;
  }
  else if(v2<0){
    vtot = v1-v2+v3;
  }
  else if(v3<0){
    vtot = v1+v2-v3;
  }
  else{
    vtot = v1+v2+v3;
  }
    
//mengubah pwm dari kecepatan yang diketahui
  if(v1==0 && v2==0){
    m1 = (v1/vtot)*100;
    m2 = (v2/vtot)*100;
    m3 = (v3/vtot)*100;
  }
  else if(v1==0 && v3==0){
    m1 = (v1/vtot)*100;
    m2 = (v2/vtot)*100;
    m3 = (v3/vtot)*100;
  }
  else if(v2==0 && v3==0){
    m1 = (v1/vtot)*100;
    m2 = (v2/vtot)*100;
    m3 = (v3/vtot)*100;
  }
  else if(v1==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
  }
  else if(v2==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
  }
  else if(v3==0){
    m1 = (v1/vtot)*200;
    m2 = (v2/vtot)*200;
    m3 = (v3/vtot)*200;
  }
  else{
    m1 = (v1/vtot)*300;
    m2 = (v2/vtot)*300;
    m3 = (v3/vtot)*300;
  }
  
//constrain untuk membuat nilai batas atas dan bawah, jika bingung bisa cari referensinya
  m1 = constrain(m1,-254,254);
  m2 = constrain(m2,-254,254);
  m3 = constrain(m3,-254,254);
    
//akses motor dari fungsi yang sudah ditentukan
  mot_1(m1);
  mot_2(m2);
  mot_3(m3);
}


void loop() {

}