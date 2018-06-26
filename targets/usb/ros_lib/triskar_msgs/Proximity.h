#ifndef _ROS_triskar_msgs_Proximity_h
#define _ROS_triskar_msgs_Proximity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace triskar_msgs
{

  class Proximity : public ros::Msg
  {
    public:
      float range[8];

    Proximity():
      range()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      union {
        float real;
        uint32_t base;
      } u_rangei;
      u_rangei.real = this->range[i];
      *(outbuffer + offset + 0) = (u_rangei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rangei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rangei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rangei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->range[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      union {
        float real;
        uint32_t base;
      } u_rangei;
      u_rangei.base = 0;
      u_rangei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rangei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rangei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rangei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->range[i] = u_rangei.real;
      offset += sizeof(this->range[i]);
      }
     return offset;
    }

    const char * getType(){ return "triskar_msgs/Proximity"; };
    const char * getMD5(){ return "65bd8f5b0dde95369b5f950f41b797ff"; };

  };

}
#endif