//measureViscosity.ino
#define TOP_SENSOR      A2 
#define MIDDLE_SENSOR   A1 
#define BOTTOM_SENSOR   A0

#define THRESHHOLD      50
#define TIMEOUT         60000

// Function Prototypes
void readAndPrintSensors();
void runViscosityMeasurement();
float calculateVelocity(uint16_t* timeArray);
float calculateViscosity(float velocity);

// Classes for state machines
class IRSensorPoller{
   private:
      enum state_t {
         pollTopSensor,
         pollMiddleSensor,
         pollBottomSensor,
         timeoutExit
      } nextState;
      uint16_t* timeArray;
      uint16_t timeout;
      uint16_t threshold;
      uint16_t startTime;
      bool firstRun;
      uint16_t thresholdArray[3];
   public:
      IRSensorPoller(uint16_t threshold, uint16_t* timeArray, uint16_t timeout);
      void run();
      uint16_t pollSensor(int pin);
};

bool test;

void setup() {
  Serial.begin(9600);
  Serial.println("------- Viscosity Measurement Program Starting -------");
  Serial.print("Threshold is set to: ");
  Serial.println(THRESHHOLD);
  Serial.println("Press 'p' to enter testing mode. Otherwise, any other button.");
  while(!Serial.available()){}
  if (Serial.read() == 'p'){
   test = true;
  }
  else {
   test = false;
  }
  Serial.println();
}

void loop() {
   if (test) {
      readAndPrintSensors();
   }
   else {
      runViscosityMeasurement();
   }
}

void readAndPrintSensors(){
   uint16_t top_sensor = analogRead(TOP_SENSOR);
   uint16_t middle_sensor = analogRead(MIDDLE_SENSOR);
   uint16_t bottom_sensor = analogRead(BOTTOM_SENSOR);
   
   Serial.print("Top Sensor Reading:\t");
   Serial.print(top_sensor);
   Serial.print("\t");

   Serial.print("Middle Sensor Reading:\t");
   Serial.print(middle_sensor);
   Serial.print("\t");

   Serial.print("Bottom Sensor Reading:\t");
   Serial.print(bottom_sensor);

   Serial.println();
}

void runViscosityMeasurement(){
   uint16_t timeArray[3];
   IRSensorPoller sensors = IRSensorPoller(THRESHHOLD, timeArray, TIMEOUT);
   Serial.println("Press any button to take measurement");
   while(!Serial.available()){}
   if(Serial.available()){
      Serial.read();
      sensors.run();
      if (timeArray[0] != 0){
         printTimeArray(timeArray);
         float velocity = calculateVelocity(timeArray);
         // float viscosity = calculateViscosity(velocity);
         // Serial.println("SUCCESS: Viscosity Measurement Calulated");
         // Serial.print("Viscosity = ");
         // Serial.print(viscosity);
         // Serial.println("[units]");
      }
      else {
         Serial.println("ERROR: Viscosity Measurement Failed");
         Serial.println();
      }
   }
}

void printTimeArray(uint16_t* timeArray){
   Serial.print("Time Array = [");
   for(uint8_t i = 0; i < 3; i++){
      Serial.print(timeArray[i]);
      Serial.print(", ");
   }
   Serial.println("]");
}

float calculateVelocity(uint16_t* timeArray){
   float v1 = 45.75/(float(timeArray[1]) - float(timeArray[0])); // [m/s]
   float v2 = 45.75/(float(timeArray[2]) - float(timeArray[1])); // [m/s]
   Serial.println();
   Serial.print("Velocity 1 reported as: ");
   Serial.print(v1, 10);
   Serial.print(" [m/s]");
   Serial.print("\t Velocity 2 reported as: ");
   Serial.print(v2, 10);
   Serial.println(" [m/s]");
   float v_avg = 45.75*2.0/float(timeArray[2] - timeArray[0]); // [m/s] (The average of the two)
   Serial.print("Average Velocity = ");
   Serial.print(v_avg, 10);
   Serial.println(" [m/s] ");
   return v_avg; // [m/s] 
}

// -------------------------- Viscosity Measurement Function ---------------------------------------------------------
float calculateViscosity(float velocity){

   float viscosity = velocity; // Placeholder for now

   return viscosity;
}

IRSensorPoller::IRSensorPoller(uint16_t threshold, 
                              uint16_t* timeArray, uint16_t timeout){
   this->threshold = threshold;
   this->timeArray = timeArray;
   this->timeout = timeout;
   this->nextState = pollTopSensor;
   this->startTime = 0;
   this->firstRun = true;
}

void IRSensorPoller::run(){
   // Get average ambient input from each sensor
   uint16_t averageBuffer [1000][3];
   for(uint16_t row=0; row<1000; row++){
      for(uint8_t column=0; column<3; column++){
         averageBuffer[row][column] = analogRead(TOP_SENSOR-column);
      }
   }
   uint32_t sum = 0;
   for(uint8_t column=0; column<3; column++){
      for(uint16_t row=0; row<1000; row++){
         sum += averageBuffer[row][column];
      }
      uint32_t average = sum/1000;
      this->thresholdArray[column] = uint16_t(average) - this->threshold;
      Serial.print("Sensor ");
      Serial.print(column);
      Serial.print(" Threshold:\t ");
      Serial.println(this->thresholdArray[column]);
   }

   bool runFlag = true;
   while(runFlag){
      switch (this->nextState){

         case pollTopSensor:
            Serial.println("Polling Top Sensor State");
            this->startTime = millis();
            this->timeArray[0] = this->pollSensor(TOP_SENSOR);
            if (this->timeArray[0] == 0){
               this->nextState = timeoutExit;
            }
            else {
               this->firstRun = false;
               this->nextState = pollMiddleSensor;
            }
            break;

         case pollMiddleSensor:
            Serial.println("Polling Middle Sensor State");
            this->timeArray[1] = this->pollSensor(MIDDLE_SENSOR);
            if (this->timeArray[1] == 0){
               this->nextState = timeoutExit;
            }
            else {
               this->nextState = pollBottomSensor;
            }
            break;

         case pollBottomSensor:
            Serial.println("Polling Bottom Sensor State");
            this->timeArray[2] = this->pollSensor(BOTTOM_SENSOR);
            if (this->timeArray[2] == 0){
               this->nextState = timeoutExit;
            }
            else {
               runFlag = false;
            }
            break;

         case timeoutExit:
            this->timeArray[0] = 0;
            runFlag = false;
            break;
      }
   }
}

uint16_t IRSensorPoller::pollSensor(int pin){
   uint16_t pollStart = millis();
   uint16_t reading;
   uint16_t sensorThreshold = this->thresholdArray[TOP_SENSOR-pin];
   while( (uint16_t(millis()) - pollStart < this->timeout) || this->firstRun){
      if ((reading = analogRead(pin)) < sensorThreshold){
         Serial.print("Pin ");
         Serial.print(pin);
         Serial.print(" triggered at: ");
         Serial.println(reading);
         return uint16_t(millis()) - this->startTime;
      }
   }
   Serial.print("Timed Out On Pin: ");
   Serial.println(pin);
   return 0;
}
