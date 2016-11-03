/* measureViscosity.ino
   By Michael George 
   10/26/16
   ME-420 California Polytechnic State Univeristy

   A program for use with tcrt5000 photo emitter/ transistor sensors in close proximity with a glass tube
   in order to make a falling ball viscometer. . The viscometer works by measuring the time difference 
   between three equally spaced points. At these points, photodiodes are present. These photodiodes trip 
   when the ball passes, marking the time that the ball passed each sensor. By measuring the length between 
   each photodiode, we are able to convert the time differences into velocities. Three points are used so 
   that we may confirm terminal velocity has been reached, which further simplifies our theoretical analysis, 
   and allows us to compare our results more easily to theory. Ultimately the average velocity between the 
   top and bottom sensor is used to calculate viscosity. A program was written using Arduino libraries and a 
   custom designed C++ classes to automate the process of going from raw time measurements in milliseconds 
   to viscosity.

*/

// Analog input pin definitions for connecting analog pins
#define TOP_SENSOR      A2 
#define MIDDLE_SENSOR   A1 
#define BOTTOM_SENSOR   A0

#define THRESHHOLD      50
#define TIMEOUT         60000

// Function Prototypes- Descriptions are with source code below
void readAndPrintSensors();
void runViscosityMeasurement();
float calculateVelocity(uint16_t* timeArray);
float calculateViscosity(float velocity);
float queryDensityInput(void);

// Classe for the IR sensor state machine- Specific descriptions in source code below
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

// ----------------------------- Setup and program selection  -----------------------------------------------------

void setup() {
   // Start Serial interface
  Serial.begin(9600);
  // Notify user program is beginning 
  Serial.println("------- Viscosity Measurement Program Starting -------");
  // Let user know the defined threshold offset for program verification
  Serial.print("Threshold offset is set to: ");
  Serial.println(THRESHHOLD);
  // Allow user to select the mode that the program will loop on
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

// -------------------------- Main Program loop  ------------------------------------------------------------------

void loop() {
   // Determine the program mode that the user has selected
   if (test) {
      // The user has opted to simply read the sensors to determine their behavior
      readAndPrintSensors();
   }
   else {
      // The user has opted to take viscosity measurements
      runViscosityMeasurement();
   }
}

// -------------------------- Program to print sensor values  -------------------------------------------------------
// Function to read the sensors and print out all of their values to the user
void readAndPrintSensors(){

   //read all the analog inputs to pins which the sensors are connected to
   uint16_t top_sensor = analogRead(TOP_SENSOR);
   uint16_t middle_sensor = analogRead(MIDDLE_SENSOR);
   uint16_t bottom_sensor = analogRead(BOTTOM_SENSOR);
   
   // Print out the values associated with each sensor
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

// -------------------------- Program to run viscosity measurement --------------------------------------------------
// Function to facilitate the taking of viscosity 

void runViscosityMeasurement(){
   // Prepare a time array
   uint16_t timeArray[3];
   // Construct an object to deal with sensor input
   IRSensorPoller sensors = IRSensorPoller(THRESHHOLD, timeArray, TIMEOUT);
   // Nofity user that program is ready to enable the trigger
   Serial.println("Press any button to take measurement");
   // Wait for button press
   while(!Serial.available()){}
   // When button is pressed, execute the program 
   if(Serial.available()){
      // read in the character
      Serial.read();
      // run the sensor polling state machine to get time matrix
      sensors.run();
      if (timeArray[0] != 0){
         // Display time array
         printTimeArray(timeArray);
         // Calculate velocity from time array
         float velocity = calculateVelocity(timeArray);
         // Calculate viscosity from velocity
         float viscosity = calculateViscosity(velocity);
         // Print result to user
         Serial.println("SUCCESS: Viscosity Measurement Calulated");
         Serial.print("Viscosity = ");
         Serial.print(viscosity);
         Serial.println("[units]");
      }
      else {
         // Measurement encountered error and was not successful
         // display to user
         Serial.println("ERROR: Viscosity Measurement Failed");
         Serial.println();
      }
   }
}

// -------------------------- Query Density function -------------------------------------------------------------
// Function used to ask the user to input density and convert that to a float and return
// NO MORE THAN ONE DECIMAL PLACE SHOULD BE USED ON DENSITY!
// Function made with significant contributions from https://www.arduino.cc/en/Tutorial/StringToFloatExample

float queryDensityInput(void){
   // Initialize necessary varibles
   String inString = "";
   float inDensity;
   // Prompt user to input Density
   Serial.println("Please Input a Density in [units]: ");
   // Read serial input:
   while (Serial.available() > 0) {
      int inChar = Serial.read();
      if (isDigit(inChar)) {
         // convert the incoming byte to a char
         // and add it to the string:
         inString += (char)inChar;
      }
      // if you get a newline, print the string,
      // then the string's value:
      if (inChar == '\n') {
         inDensity = inString.toFloat(); // Can divide by scalar here to get a lower order magnitude
         Serial.print("Input Loopback: ");
         Serial.println(inString);
         Serial.print("Desity Value:");
         Serial.println();
         // clear the string for new input:
         inString = "";
      }
      return inDensity;
   }
}

// -------------------------- Print Time Array function -------------------------------------------------------------
// Function to print out the three elements in the time array to display them to the user

void printTimeArray(uint16_t* timeArray){
   Serial.print("Time Array = [");
   for(uint8_t i = 0; i < 3; i++){
      Serial.print(timeArray[i]);
      Serial.print(", ");
   }
   Serial.println("]");
}

// -------------------------- Velocity calculation function ---------------------------------------------------------
// Function to calculate the velocity based upon the time array that holds the time values between the three sensors
// and a known constant distance between them.

float calculateVelocity(uint16_t* timeArray){
   // Calculate velocity between sensor 1 and 2
   float v1 = 45.75/(float(timeArray[1]) - float(timeArray[0])); // [m/s]
   // Calculate velocity between sensor 2 and 3
   float v2 = 45.75/(float(timeArray[2]) - float(timeArray[1])); // [m/s]
   // Print the two calculated velocities to the user
   Serial.println();
   Serial.print("Velocity 1 reported as: ");
   Serial.print(v1, 10);
   Serial.print(" [m/s]");
   Serial.print("\t Velocity 2 reported as: ");
   Serial.print(v2, 10);
   Serial.println(" [m/s]");
   // Calculate the average of the two velocities
   float v_avg = 45.75*2.0/float(timeArray[2] - timeArray[0]); // [m/s] (The average of the two)
   // Print out the average velocity 
   Serial.print("Average Velocity = ");
   Serial.print(v_avg, 10);
   Serial.println(" [m/s] ");
   return v_avg; // [m/s] 
}

// -------------------------- Viscosity Measurement Function ---------------------------------------------------------
// Function used for calculating the viscosity from the average velocity measured

float calculateViscosity(float velocity){

   // Density if necessary in calculation
   // float density = queryDensityInput();
   float viscosity = 0.0295*pow(velocity, -1.6083); // in centipoise

   return viscosity;
}

// -------------------------- IRSensor Poller state machine class ----------------------------------------------------
// Class for housing the IR sensor polling state machine. Here is the heavy hitter of the viscosity measurement program
// and the most custom designed implementation in the program. Polling means that the sensors value is constantly 
// measured until a condition is reached in this context. 


// --------------------------------- Constructor ----------------------------------------------------------------------
// Called in order to initialize the IR sensor poller and all of it's constant values

IRSensorPoller::IRSensorPoller(uint16_t threshold, 
                              uint16_t* timeArray, uint16_t timeout){
   this->threshold = threshold;
   this->timeArray = timeArray;
   this->timeout = timeout;
   this->nextState = pollTopSensor;
   this->startTime = 0;
   this->firstRun = true;
}

// --------------------------------- run method ----------------------------------------------------------------------
// Called to execute the IR sensor poller state machine. This method will block until the top sensor is triggered, 
// then will wait until a time out or the middle sensor is triggered and likewise for the bottom sensor. Order that 
// the sensors are triggered in is enforced, ensuring a robust measurement of velocity only when all three sensors
// are successfully tripped.

void IRSensorPoller::run(){
   Serial.println("Averaging");
   // Get average ambient input from each sensor
   // number of samples to average the ambient input over (to avoid possibility of crazy noise)
   uint8_t samples = 10;
   // Get an average buffer ready
   uint16_t averageBuffer [samples][3];
   // Loop accross the three sensors collecting the samples and filling the average buffer
   for(uint16_t row=0; row<samples; row++){
      for(uint8_t column=0; column<3; column++){
         averageBuffer[row][column] = analogRead(TOP_SENSOR-column);
      }
   }
   Serial.println("Averaging Done");
   // prepare a sum for the average function to use
   uint32_t sum = 0;
   // unpack the average buffer and calculate the average 
   for(uint8_t column=0; column<3; column++){
      for(uint16_t row=0; row<samples; row++){
         sum += averageBuffer[row][column];
      }
      uint32_t average = sum/samples;
      sum = 0;
      // Subtract off the threshold offset and store it for each sensor
      this->thresholdArray[column] = uint16_t(average) - this->threshold;
      // print it out for debugging
      Serial.print("Sensor ");
      Serial.print(column);
      Serial.print(" Threshold:\t ");
      Serial.println(this->thresholdArray[column]);
   }

   // Set the run flag so that the state machine will run until an exit state sets the flag to false
   bool runFlag = true;
   while(runFlag){
      // Switch upon the possible states that the state machine can be in.
      switch (this->nextState){
         // Poll the top sensor until it is triggered
         case pollTopSensor:
            Serial.println("Polling Top Sensor State");
            this->startTime = millis();
            this->timeArray[0] = this->pollSensor(TOP_SENSOR);
            // Test if the output is valid, or if timeout was reached
            if (this->timeArray[0] == 0){
               this->nextState = timeoutExit;
            }
            else {
               this->firstRun = false;
               // Set the next state to poll the middle sensor
               this->nextState = pollMiddleSensor;
            }
            break;

         // Poll the middle sensor
         case pollMiddleSensor:
            Serial.println("Polling Middle Sensor State");
            this->timeArray[1] = this->pollSensor(MIDDLE_SENSOR);
            // Test if the output is valid, or if timeout was reached
            if (this->timeArray[1] == 0){
               this->nextState = timeoutExit;
            }
            else {
               // Set the next state to poll the bottom sensor
               this->nextState = pollBottomSensor;
            }
            break;

         // Poll the middle sensor
         case pollBottomSensor:
            Serial.println("Polling Bottom Sensor State");
            this->timeArray[2] = this->pollSensor(BOTTOM_SENSOR);
            // Test if the output is valid, or if timeout was reached
            if (this->timeArray[2] == 0){
               this->nextState = timeoutExit;
            }
            else {
               // Set the next state to poll Exit gracefully
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

// --------------------------------- Poll Sensor method -----------------------------------------------------------------
// Called to poll a single sensor. Uses the member data with the sensor so that each sensor triggers at a value that is 
// a constant offset of its ambient or quiescent value. This ensures that the sensor works robustly for all different
// clarities of fluids. This will return the milliseconds between the start of polling and when the sensor is tripped.

uint16_t IRSensorPoller::pollSensor(int pin){
   // mark the start time
   uint16_t pollStart = millis();
   uint16_t reading;
   // grab the threshold that will be used for this specific sensor from the member data
   uint16_t sensorThreshold = this->thresholdArray[TOP_SENSOR-pin];
   // loop until timeout is reached or until the sensor is triggered successfully
   while( (uint16_t(millis()) - pollStart < this->timeout) || this->firstRun){
      // If the reading is greater than the sensor theshold, then mark the end time
      if ((reading = analogRead(pin)) < sensorThreshold){
         // Notify the user of the pin trigger value for debugging
         Serial.print("Pin ");
         Serial.print(pin);
         Serial.print(" triggered at: ");
         Serial.println(reading);
         // return the difference between start and end time of the sensor polling
         return uint16_t(millis()) - this->startTime;
      }
   }
   // Time out was reached, let the user know which pin the timeout occured on
   Serial.print("Timed Out On Pin: ");
   Serial.println(pin);
   // return a 0 which will be used in the time matrix as an invalid value to indicate an error occured
   return 0;
}
