#include <Servo.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include<LSM303.h>
#include<SD.h>

const int chipSelect=10;
File dataFile;
Servo myservo;
int i, headingValue;
int headingcompass;
LSM303 compass;

TinyGPS gps;
SoftwareSerial nss(8, 7);
  float flat, flon, x2lat, x2lon;
void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 0);

void setup()
{
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  pinMode(chipSelect, OUTPUT);
  nss.begin(9600); 
  Wire.begin();
  compass.init();
  compass.enableDefault();
  LSM303::vector<int16_t> running_min = {574,-915, 687}, running_max = {584, -907, -679};
  myservo.attach(9);
    // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
 
  // Open up the file we're going to log to!
  dataFile = SD.open("GPStest.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening GPStest.txt");
    // Wait forever since we cant write data
    while (1) ;
  } 
}

void loop()
{
  bool newdata = false;
  unsigned long start = millis();

  // Every fourth of a second we print an update
  while (millis() - start < 250)
  {
    if (feedgps())
      newdata = true;
  }
  
  if (newdata)
  {
    Serial.println("Acquired Data");
    Serial.println("-------------");
    gpsdump(gps);
    Serial.println("-------------");
    Serial.println();
  }
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;

  unsigned long age, date, time, chars;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat, 5); Serial.print(", "); printFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  feedgps();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: "); Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
  distance();
}
  
bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

void distance(){
  float x4;
  float heading; 
 float flat1=flat;     // flat1 = our current latitude. flat is from the gps data. 
 float flon1=flon;  // flon1 = our current longitude. flon is from the fps data.
float dist_calc=0;
float dist_calc2=0;
float diflat=0;
float diflon=0;
int waycont=1;
//For switching waypoints make multiple waypoints in your code
float flat2=50.953142;   //waypoint 1  
float flon2=-1.361897;
float flat3=50.954507;        //waypoint 2
float flon3=-1.361468;
float flat4=50.9546242;       //waypoint 3
float flon4=-1.363893;
float flat5=50.953344;     // waypoint   4
float flon5=-1.364021;
 if(waycont==1){
   x2lat = flat2;      // setting x2lat and x2lon equal to our first waypoint
   x2lon = flon2;   
  }
  if(waycont==2){
    x2lat = flat3;
    x2lon = flon3;
  }
  if(waycont==3){
    x2lat = flat4;
    x2lon = flon4;
  }
  if(waycont==4){
  x2lat = flat5;
  x2lon = flon5;
  }
x2lat=50.953142;  //enter a latitude point here   this is going to be your waypoint
x2lon=-1.361897;;
//---------------------------------- distance formula below. Calculates distance from current location to waypoint
diflat=radians(x2lat-flat1);  //notice it must be done in radians
flat1=radians(flat1);    //convert current latitude to radians
x2lat=radians(x2lat);  //convert waypoint latitude to radians
diflon=radians((x2lon)-(flon1));   //subtract and convert longitudes to radians
dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
dist_calc2= cos(flat1);
dist_calc2*=cos(x2lat);
dist_calc2*=sin(diflon/2.0);                                       
dist_calc2*=sin(diflon/2.0);
dist_calc +=dist_calc2;
dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
dist_calc*=6371000.0; //Converting to meters
Serial.println("distance");
Serial.println(dist_calc);    //print the distance in meters
 flon1 = radians(flon1);  //also must be done in radians
 x2lon = radians(x2lon);  //radians duh.
heading = atan2(sin(x2lon-flon1)*cos(x2lat),cos(flat1)*sin(x2lat)-sin(flat1)*cos(x2lat)*cos(x2lon-flon1)),2*3.1415926535;
heading = heading*180/3.1415926535;  // convert from radians to degrees
int head =heading; //make it a integer now
if(head<0){
  heading+=360;   //if the heading is negative then add 360 to make it positive
}
Serial.println("heading:");
Serial.println(int(heading));   // print the heading.
compass.read();
float headingcompass = compass.heading();
Serial.println("current heading:");
Serial.println(int(headingcompass));
x4=headingcompass-heading;   //getting the difference of our current heading to our needed bearing
int turn;
//-------------------------------------- below tells us which way we need to turn
if(x4<-180){
  turn=5;      //set turn = 5 which means "left"
}
if(x4>=0){
  if(x4<180){
    turn=5;   //set turn = 5 which means "left"
  }
}
int hd = headingcompass;
if(hd==heading){
    turn=3;   //then set turn = 3 meaning go "straight"
}
if(turn==3){
  Serial.println("straight");
  myservo.write(90);
  delay(60);
  return;
}
if(turn==5){
  leftturn();
}
Serial.println("distance"); 
Serial.println(dist_calc);
if(dist_calc<4){             //this goes after the serial print of the distance in the function "distance". refer to my code. 4=radius
waycont+=1;
}
}
void leftturn(){
  float x4;
  float heading;
  if(headingcompass+2>heading){
    if(headingcompass-2<heading){
      myservo.write(90);  //straighten the ailerons if we are heading to the waypoint
    delay(60);
    return;
  }
  }
x4=headingcompass-heading;  
if(x4>=-180){
  if(x4<=0){
     return;         
    }
  }
if(x4>=180){     
  return;
}
myservo.write(110);  //turn the ailerons left
compass.read();
float headingcompass = compass.heading();
leftturn();
}
