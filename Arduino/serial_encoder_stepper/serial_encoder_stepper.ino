#include <RotaryEncoder.h>
#include <Stepper.h>

#include "serial_command_server.hpp"

SerialCommandServer serial_command_server;

uint32_t const ENCODER_COUNT = 2;

RotaryEncoder encoder[ENCODER_COUNT] = {
  RotaryEncoder(5, 6, RotaryEncoder::LatchMode::TWO03),
  RotaryEncoder(7, 8, RotaryEncoder::LatchMode::TWO03)
};

uint32_t const steps_per_revolution = 200;

uint32_t const STEPPER_COUNT = 2;

Stepper stepper[STEPPER_COUNT] = {
  Stepper(steps_per_revolution, 9, 10),
  Stepper(steps_per_revolution, 11, 12)
};

uint8_t encoder_read(uint8_t const &request_size, uint8_t const *request_buffer, uint8_t &response_size, uint8_t *response_buffer)
{
  if (request_size == sizeof(uint32_t)) {
    uint32_t encoder_id;
  
    memcpy(&encoder_id, &request_buffer[0], sizeof(uint32_t));
    
    if (encoder_id >= 0 && encoder_id < ENCODER_COUNT) {
      int32_t position = encoder[encoder_id].getPosition();
      int32_t direction = (int32_t)encoder[encoder_id].getDirection();

      response_size = sizeof(int32_t) + sizeof(int32_t);

      memcpy(&response_buffer[0], &position, sizeof(int32_t));
      memcpy(&response_buffer[sizeof(int32_t)], &direction, sizeof(int32_t));
  
      return SerialCommandStatus::SUCCESS;
    }
  }
  
  return SerialCommandStatus::FAILURE;
}

uint8_t stepper_write(uint8_t const &request_size, uint8_t const *request_buffer, uint8_t &response_size, uint8_t *response_buffer) 
{
  if (request_size == sizeof(uint32_t) + sizeof(uint32_t) + sizeof(int32_t)) {
    uint32_t stepper_id;
    uint32_t speed;
    int32_t step;

    memcpy(&stepper_id, &request_buffer[0], sizeof(uint32_t));
    memcpy(&speed, &request_buffer[sizeof(uint32_t)], sizeof(uint32_t));
    memcpy(&step, &request_buffer[sizeof(uint32_t) + sizeof(uint32_t)], sizeof(int32_t));
    
    if (stepper_id >= 0 && stepper_id < STEPPER_COUNT) {
      stepper[stepper_id].setSpeed(speed);
      stepper[stepper_id].step(step);
      
      return SerialCommandStatus::SUCCESS;
    }
  }

  return SerialCommandStatus::FAILURE;
}

void setup() 
{
  serial_command_server.registerCommand(0, &encoder_read); 
  serial_command_server.registerCommand(1, &stepper_write); 
  
  serial_command_server.enable();
}

void loop() 
{
  serial_command_server.listen();
}
