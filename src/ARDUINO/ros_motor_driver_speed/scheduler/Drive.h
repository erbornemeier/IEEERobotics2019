#ifndef _ROS_SERVICE_Drive_h
#define _ROS_SERVICE_Drive_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scheduler
{

static const char DRIVE[] = "scheduler/Drive";

  class DriveRequest : public ros::Msg
  {
    public:
      typedef float _forward_type;
      _forward_type forward;
      typedef float _theta_type;
      _theta_type theta;
      typedef uint8_t _type_type;
      _type_type type;

    DriveRequest():
      forward(0),
      theta(0),
      type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_forward;
      u_forward.real = this->forward;
      *(outbuffer + offset + 0) = (u_forward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_forward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_forward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_forward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->forward);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_forward;
      u_forward.base = 0;
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_forward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->forward = u_forward.real;
      offset += sizeof(this->forward);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
     return offset;
    }

    const char * getType(){ return DRIVE; };
    const char * getMD5(){ return "c4f62ea2e5e6bec6b0fce04af9200ec2"; };

  };

  class DriveResponse : public ros::Msg
  {
    public:

    DriveResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return DRIVE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Drive {
    public:
    typedef DriveRequest Request;
    typedef DriveResponse Response;
  };

}
#endif
