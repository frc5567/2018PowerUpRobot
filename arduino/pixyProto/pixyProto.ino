#include <Wire.h>
#include <Pixy.h>

//  SETUP THE DEVICES

//  Plug sda on RoboRIO into 20
//  Plug scl on RoboRIO into 21
//  Connect the two grounds

//  String to be sent to the robot
String piOutput = "none";

// String received from the robot              
String input = "blank";  
const String PIXY = "pi";

//  Declares the Pixy
Pixy pixy;

void setup(){
  Serial.begin(9600);

  //  Connects i2c bus with address #20 as a slave device
  Wire.begin(20);
  
  //  Registers a function to be called when a slave device receives a transmission from a master                
  Wire.onReceive(receiveEvent); 

  //  Register a function to be called when a master requests data from this slave device
  Wire.onRequest(requestEvent);

  //  Initializes the Pixy
  pixy.init();
  Serial.print("Starting");
}

void loop(){

  //  Gets every available object from the Pixy
  uint16_t blocks = pixy.getBlocks();
  
  int biggest = -1;
  double area = 0, temp;

  Serial.print(String(blocks));
  Serial.print("  -  ");

  //  Finds the biggest block sent by the Pixy
  for(int i=0;i<blocks;i++){

    //  Selects the third signature from the Pixy (the Power Cube)
    if(pixy.blocks[i].signature == 3){
      temp = pixy.blocks[i].width * pixy.blocks[i].height;
      if(temp > area){
        area = temp;
        biggest = i;
      }
  }
      
  }

  //  Tells the Rio that the Pixy doesn't see any blocks
  if(!blocks){
    piOutput = "none";
  }

  //  Puts the X, Y, and Area in a string as percents. The lines are used for splitting the string
  else{
    piOutput = String(pixy.blocks[biggest].x / 319.0);   
    piOutput += "|";                
    piOutput += String(pixy.blocks[biggest].y / 199.0); 
    piOutput += "|";          
    piOutput += String(area / 64000); 
    
  }
  
  Serial.print(piOutput);
  Serial.print("\n");
  delay(70); 
}
//  Called when RoboRIO requests a message from this device
void requestEvent(){
  //  Writes data to the RoboRIO as a string
  Wire.write(piOutput.c_str()); 
  
}

//  Called when RoboRIO "gives" this device a message
void receiveEvent(int bytes){

}
