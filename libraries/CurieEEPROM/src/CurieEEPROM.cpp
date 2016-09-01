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
#include "CurieEEPROM.h"

CurieEEPROM EEPROM;



void CurieEEPROM::clear()
{
  clearCalled = true;
  //erase the 2k bytes of the eeprom section inside the otp area
  *(uint32_t*)(ROM_WR_CTRL) = 0x4002;
  //wait for erase to be complete
  //TODO: while(((*(uint32_t*)FLASH_STTS) & 0x01) == 0) //wait for FLASH_STTS.ER_DONE to be set to 1
  delay(1000);
}

void CurieEEPROM::write(uint32_t address, uint32_t data)
{
  for (int i = 0; i<4; i++) {
    write8(address+i, data >> ((3-i)*8) & 0xFF);
  }
}

void CurieEEPROM::write8(uint32_t address, uint8_t data)
{
  //make sure address is valid
  if((address > 0x7FF))
  {
    return;
  }
  
  uint8_t currentValue = read8(address);
  //only do something if value is different from what is currently stored
  if(currentValue==data)
  {
    return;
  }

  if (clearCalled == false) {
    uint8_t dump[EEPROM_SIZE];
    memcpy(dump, (uint32_t *)EEPROM_ADDR, EEPROM_SIZE);
    clear();
    for (int i=0; i<EEPROM_SIZE; i++) {
      write8(i, dump[i]);
    }
  }

  uint32_t currentDword = read(address);

  uint32_t rom_wr_ctrl = 0;

  int offset = address%4;

  uint32_t data32 = (currentDword & ~(uint32_t)(0xFF << ((3-offset)*8)));
  data32 = data32 | (data << ((3-offset)*8));

  //store data into ROM_WR_DATA	register
  *(uint32_t*)(ROM_WR_DATA) = data32;
  address = ((address >> 2) << 2) + EEPROM_OFFSET;
  rom_wr_ctrl = (address)<<2; //shift left 2 bits to store offset into bits 19:2 (WR_ADDR)
  rom_wr_ctrl |= 0x00000001; //set (WR_REQ) bit
  *(uint32_t*)(ROM_WR_CTRL) = rom_wr_ctrl;

/*
  Serial.print("Writing ");
  Serial.print(data32, HEX);
  Serial.print(" in ");
  Serial.println(address, HEX);
*/

  delay(3); //give it enough time to finish writing
}

void CurieEEPROM::update(uint32_t addr, uint32_t value)
{
  write(addr, value);
}
void CurieEEPROM::update8(uint32_t addr, uint8_t value)
{
  write8(addr, value);
}

uint8_t CurieEEPROM::read8(uint32_t address)
{
  if((address > 0x7FF))
  {
    return 0;
  }
  int offset = address%4;
  uint32_t value = *(uint32_t*)(EEPROM_ADDR+(address/4)*4);
  value = (value >> ((3-offset)*8)) & 0xFF;
  return (uint8_t)value;
}

uint32_t CurieEEPROM::read(uint32_t address)
{
  if((address > 0x7FF))
  {
    return 0;
  }
  uint32_t value = *(uint32_t*)(EEPROM_ADDR+(address/4)*4);
  return value;
}

uint32_t& CurieEEPROM::operator[](int i)
{
  return *(uint32_t*)(EEPROM_ADDR+i*sizeof(uint32_t));
}

int CurieEEPROM::length()
{
  return EEPROM_SIZE;
}

int CurieEEPROM::begin()
{
  return 0x00;
}

int CurieEEPROM::end()
{
  return length();
}
