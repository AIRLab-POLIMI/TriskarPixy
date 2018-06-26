#ifndef _ROS_triskar_msgs_Pixy_h
#define _ROS_triskar_msgs_Pixy_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace triskar_msgs
{

  class Pixy : public ros::Msg
  {
    public:
      typedef int32_t _checksum_type;
      _checksum_type checksum;
      typedef int32_t _signature_type;
      _signature_type signature;
      typedef int32_t _x_type;
      _x_type x;
      typedef int32_t _y_type;
      _y_type y;
      typedef int32_t _width_type;
      _width_type width;
      typedef int32_t _height_type;
      _height_type height;

    Pixy():
      checksum(0),
      signature(0),
      x(0),
      y(0),
      width(0),
      height(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_checksum;
      u_checksum.real = this->checksum;
      *(outbuffer + offset + 0) = (u_checksum.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_checksum.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_checksum.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_checksum.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->checksum);
      union {
        int32_t real;
        uint32_t base;
      } u_signature;
      u_signature.real = this->signature;
      *(outbuffer + offset + 0) = (u_signature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_signature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_signature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_signature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->signature);
      union {
        int32_t real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        int32_t real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_checksum;
      u_checksum.base = 0;
      u_checksum.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_checksum.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_checksum.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_checksum.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->checksum = u_checksum.real;
      offset += sizeof(this->checksum);
      union {
        int32_t real;
        uint32_t base;
      } u_signature;
      u_signature.base = 0;
      u_signature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_signature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_signature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_signature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->signature = u_signature.real;
      offset += sizeof(this->signature);
      union {
        int32_t real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        int32_t real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        int32_t real;
        uint32_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        int32_t real;
        uint32_t base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
     return offset;
    }

    const char * getType(){ return "triskar_msgs/Pixy"; };
    const char * getMD5(){ return "dacb46c9791f644bf147485599c68932"; };

  };

}
#endif