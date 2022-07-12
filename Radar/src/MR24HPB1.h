#ifndef MR24HPB1_H
#define MR24HPB1_H

#include <hardwareserial.h>
#include <functional>
#include "Arduino.h"
#include "MR24HPB1_def.h"

/*
 * This Library is used for the MR24HPB1 Human static presence sensor by Seedstudio. 
 * I do not guarantee that this library will be up todate or bug free, use at your own risk
 */


 class MR24HPB1{
  public:
    /*
     * Create an instance of the sensor 
     * @Param serial the serial port to use for the sensor
     * @Param pin_presence the pin the s1 pin is connected to
     * @Param pin_motion the pin the s2 pin is connected to
     */
    MR24HPB1(HardwareSerial& serialPort,uint8_t pin_presence,uint8_t pin_motion);

    /* Register some event handlers (optional) an Event callback is called whenever the refresh() 
	*/
    void register_on_unoccupied(std::function<void(void)> callback);
    void register_on_occupied(std::function<void(void)> callback);
    void register_on_stationary(std::function<void(void)> callback);
    void register_on_movement(std::function<void(void)> callback);
    void register_on_away_state(std::function<void(uint8_t)> callback);
    void register_on_environmental_state(std::function<void(uint8_t)> callback);
    void register_on_motor_signs(std::function<void(float)> callback);
	
	/*
	*	kind of a delay but it will run the sensor data gathering in the background.
	*	Due to the execution of code the delays might not be exact. 
	*	Do not use for critical timing.
		@Param Delay the delay in ms.
	*/
    void yield(long Delay);
	// refer to the datasheet or the MR24HPB1_def for interpretation
    uint8_t getMotionStatus();
    uint8_t getThreshold();
    uint8_t getSceneSetting();
    uint8_t getMotorSigns();
    uint8_t getEnvironmentalState();
    uint8_t getAbnormalResets(){return abnormalResets;}
	// Get the stored presence state
    boolean getPresence();
	// Returns the type of data that was updated last and 0xFF if no data was recieved between the last call and this call.
    uint8_t getUpdatedMemberType();
    away_state_t  getAwayState();

	// configure the sensor refer to Datasheet and/or MR24HPB1_def for available settings
    int   setThreshold(uint8_t gear);
    int   setSceneSetting(scene_setting_t scene);
    void  Reboot();

	// Initialize the sensor and its settings OPTIONAL
    int begin(uint8_t threshold,scene_setting_t scene);
    int begin();
    
    // Handles the recieving/updating of the sensor data. Needs to be called frequently. e.g in the loop() in the case of arduino
    void refresh();
    

  private:
     int8_t  away_state =-1, threshold=-1, scene_setting=-1, motion_pin=-1, presence_pin=-1;
     int8_t environmental_state=-1; 
     uint8_t  abnormalResets =0;
     uint8_t  updated_member;
     float    motor_signs;
     boolean  presence=false, motion=false, newData=false;
     uint8_t  msg[30];
     uint16_t msg_size;
     
     HardwareSerial& serial;

     uint16_t CRC16(uint8_t *Frame, uint8_t Len);
     uint16_t getMsg_length();
     uint16_t getData_length();
     void     getPinValues();
     boolean  verifyMsg(uint8_t* msg, uint8_t length); //msg should not iclude header
     void     sendMsg(function_cmd_t fc, addr_cmd1_t cmd1,addr_cmd2_t cmd2,uint8_t *data,uint8_t data_length);
     void     recieveMsg();
     void     parseMsg();
     
     
     // Store the callback functions so we can call them later
     std::function<void(void)>    _on_unoccupied;
     std::function<void(void)>    _on_occupied;
     std::function<void(void)>    _on_stationary;
     std::function<void(void)>    _on_movement;
     std::function<void(uint8_t)> _on_away_state;
     std::function<void(uint8_t)> _on_environmental_state;
     std::function<void(float)>   _on_motor_signs;

     
 };








#endif
