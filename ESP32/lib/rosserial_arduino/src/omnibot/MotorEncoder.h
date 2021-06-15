#ifndef _ROS_omnibot_MotorEncoder_h
#define _ROS_omnibot_MotorEncoder_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace omnibot
{

  class MotorEncoder : public ros::Msg
  {
    public:
      typedef int32_t _en_a_type;
      _en_a_type en_a;
      typedef int32_t _en_b_type;
      _en_b_type en_b;
      typedef int32_t _en_c_type;
      _en_c_type en_c;

    MotorEncoder():
      en_a(0),
      en_b(0),
      en_c(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_en_a;
      u_en_a.real = this->en_a;
      *(outbuffer + offset + 0) = (u_en_a.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_en_a.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_en_a.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_en_a.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->en_a);
      union {
        int32_t real;
        uint32_t base;
      } u_en_b;
      u_en_b.real = this->en_b;
      *(outbuffer + offset + 0) = (u_en_b.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_en_b.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_en_b.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_en_b.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->en_b);
      union {
        int32_t real;
        uint32_t base;
      } u_en_c;
      u_en_c.real = this->en_c;
      *(outbuffer + offset + 0) = (u_en_c.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_en_c.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_en_c.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_en_c.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->en_c);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_en_a;
      u_en_a.base = 0;
      u_en_a.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_en_a.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_en_a.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_en_a.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->en_a = u_en_a.real;
      offset += sizeof(this->en_a);
      union {
        int32_t real;
        uint32_t base;
      } u_en_b;
      u_en_b.base = 0;
      u_en_b.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_en_b.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_en_b.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_en_b.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->en_b = u_en_b.real;
      offset += sizeof(this->en_b);
      union {
        int32_t real;
        uint32_t base;
      } u_en_c;
      u_en_c.base = 0;
      u_en_c.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_en_c.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_en_c.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_en_c.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->en_c = u_en_c.real;
      offset += sizeof(this->en_c);
     return offset;
    }

    virtual const char * getType() override { return "omnibot/MotorEncoder"; };
    virtual const char * getMD5() override { return "e4bb3a1f2069e87a3b7e5d6fafd178b4"; };

  };

}
#endif
