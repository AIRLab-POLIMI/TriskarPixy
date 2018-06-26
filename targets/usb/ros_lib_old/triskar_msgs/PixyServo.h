#ifndef _ROS_triskar_msgs_PixyServo_h
#define _ROS_triskar_msgs_PixyServo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace triskar_msgs
{

  class PixyServo : public ros::Msg
  {
    public:
      typedef uint16_t _pan_type;
      _pan_type pan;
      typedef uint16_t _tilt_type;
      _tilt_type tilt;

    PixyServo():
      pan(0),
      tilt(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pan >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pan >> (8 * 1)) & 0xFF;
      offset += sizeof(this->pan);
      *(outbuffer + offset + 0) = (this->tilt >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tilt >> (8 * 1)) & 0xFF;
      offset += sizeof(this->tilt);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->pan =  ((uint16_t) (*(inbuffer + offset)));
      this->pan |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->pan);
      this->tilt =  ((uint16_t) (*(inbuffer + offset)));
      this->tilt |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->tilt);
     return offset;
    }

    const char * getType(){ return "triskar_msgs/PixyServo"; };
    const char * getMD5(){ return "a18daab5d67a24020ef45a1f24563f65"; };

  };

}
#endif