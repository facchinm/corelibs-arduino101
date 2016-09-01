/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.

 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#define ROM_WR_CTRL	0xb0100004
#define ROM_WR_DATA	0xb0100008
#define FLASH_STTS	0xb0000014
#define EEPROM_ADDR 0xfffff000
#define EEPROM_OFFSET 0x00001000
#define CTRL_REG    0xb0000018

#define EEPROM_SIZE 2048 //EEPROM size in bytes


#include <inttypes.h>
#include "Arduino.h"

class CurieEEPROM 
{
public:
  CurieEEPROM() 
  {

  }
  void clear();
  void write(uint32_t address, uint32_t data);
  void write(uint32_t address, uint8_t data) {write8(address, data);};
  void write8(uint32_t address, uint8_t data);
  void update(uint32_t addr, uint32_t value);
  void update8(uint32_t addr, uint8_t value);
  uint32_t read(uint32_t addr);
  uint8_t read8(uint32_t addr);
  
  uint32_t& operator[](int i);
  
  int length();
  int begin();
  int end();
  
  //Functionality to 'get' and 'put' objects to and from EEPROM.
  template< typename T > T &get(uint32_t addr, T &t)
  {
    //make sure address is valid
    if((addr > 0x7FF))
    {
      return t;
    }
    int byteCount = sizeof(T);
    //return if size of object is greater than size of EEPROM
    if(addr + byteCount > EEPROM_SIZE)
    {
      return t;
    }
    byte *bytes = to_bytes(t);
    for(int i = 0; i < byteCount; i++)
    {
      bytes[i] = read8(addr+i);
    }
    from_bytes(bytes, t);
    delete bytes;
    return t;
  }

  template< typename T > T put(uint32_t addr, T t)
  {
    //make sure address is valid
    if((addr > 0x7FF))
    {
      return t;
    }

    int byteCount = sizeof(T);
    //return if size of object is greater than size of EEPROM
    if(addr + byteCount > EEPROM_SIZE)
    {
      return t;
    }

    byte *bytes = to_bytes(t);
    for(int i = 0; i < byteCount; i++)
    {
      write8(addr+i, bytes[i]);
    }
    from_bytes(bytes, t);
    delete bytes;
    return t;
  }

private:

  bool clearCalled = false;

  template< typename T > byte* to_bytes(const T& object)
  {
    size_t buffer_size = sizeof(object);
    byte *buffer = new byte[buffer_size];
    memcpy(buffer, &object, buffer_size);

    return buffer;
  }
  
  template< typename T > uint32_t* to_dwords(const T& object)
  {
    size_t buffer_size = sizeof(object);
    uint32_t *buffer = new uint32_t[buffer_size];
    memcpy(buffer, &object, buffer_size);
 
    return buffer;
  }
  
  template< typename T > T& from_bytes(byte* bytes, T& object )
  {
    memcpy(&object, bytes, sizeof(object));
    return object;
  } 
};

extern CurieEEPROM EEPROM;
