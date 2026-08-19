// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_packet.hpp"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <memory>
#include <string>
#include <cmath>

#include "vesc_driver/datatypes.hpp"
#include "vesc_driver/vesc_packet_factory.hpp"


namespace vesc_driver
{

constexpr CRC::Parameters<crcpp_uint16, 16> VescFrame::CRC_TYPE;

VescFrame::VescFrame(int payload_size)
{
  assert(payload_size >= 0 && payload_size <= 1024);

  if (payload_size < 256) {
    // single byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + payload_size));
    *frame_->begin() = 2;
    *(frame_->begin() + 1) = payload_size;
    payload_.first = frame_->begin() + 2;
  } else {
    // two byte payload size
    frame_.reset(new Buffer(VESC_MIN_FRAME_SIZE + 1 + payload_size));
    *frame_->begin() = 3;
    *(frame_->begin() + 1) = payload_size >> 8;
    *(frame_->begin() + 2) = payload_size & 0xFF;
    payload_.first = frame_->begin() + 3;
  }

  payload_.second = payload_.first + payload_size;
  *(frame_->end() - 1) = 3;
}

VescFrame::VescFrame(const BufferRangeConst & frame, const BufferRangeConst & payload)
{
  /* VescPacketFactory::createPacket() should make sure that the input is valid, but run a few cheap
     checks anyway */
  assert(std::distance(frame.first, frame.second) >= VESC_MIN_FRAME_SIZE);
  assert(std::distance(frame.first, frame.second) <= VESC_MAX_FRAME_SIZE);
  assert(std::distance(payload.first, payload.second) <= VESC_MAX_PAYLOAD_SIZE);
  assert(
    std::distance(frame.first, payload.first) > 0 &&
    std::distance(payload.second, frame.second) > 0);

  frame_.reset(new Buffer(frame.first, frame.second));
  payload_.first = frame_->begin() + std::distance(frame.first, payload.first);
  payload_.second = frame_->begin() + std::distance(frame.first, payload.second);
}

VescPacket::VescPacket(const std::string & name, int payload_size, int payload_id)
: VescFrame(payload_size), name_(name)
{
  assert(payload_id >= 0 && payload_id < 256);
  assert(std::distance(payload_.first, payload_.second) > 0);
  *payload_.first = payload_id;
}

VescPacket::VescPacket(const std::string & name, std::shared_ptr<VescFrame> raw)
: VescFrame(*raw), name_(name)
{
}

/*------------------------------------------------------------------------------------------------*/

VescPacketFWVersion::VescPacketFWVersion(std::shared_ptr<VescFrame> raw)
: VescPacket("FWVersion", raw)
{
  major_ = 0;
  minor_ = 0;
  paired_ = false;
  devVersion_ = 0;
  std::fill(std::begin(uuid_), std::end(uuid_), 0);

  const auto size = std::distance(payload_.first, payload_.second);
  if (size >= 3) {
    major_ = *(payload_.first + 1);
    minor_ = *(payload_.first + 2);
  }

  // Firmware 2.x only returns [command, major, minor]. Newer firmware appends
  // a null-terminated hardware name, UUID and pairing metadata.
  if (size <= 3) {
    return;
  }

  auto cursor = payload_.first + 3;
  while (cursor != payload_.second && *cursor != 0x00) {
    hwname_ += static_cast<char>(*cursor++);
  }
  if (cursor == payload_.second) {
    return;
  }
  ++cursor;

  if (std::distance(cursor, payload_.second) >= 12) {
    std::copy_n(cursor, 12, uuid_);
    cursor += 12;
  }
  if (cursor != payload_.second) {
    paired_ = static_cast<bool>(*cursor++);
  }
  if (cursor != payload_.second) {
    devVersion_ = *cursor;
  }
}

int VescPacketFWVersion::fwMajor() const
{
  return major_;
}

int VescPacketFWVersion::fwMinor() const
{
  return minor_;
}

std::string VescPacketFWVersion::hwname() const
{
  return hwname_.c_str();
}
const uint8_t * VescPacketFWVersion::uuid() const
{
  return uuid_;
}
bool VescPacketFWVersion::paired() const
{
  return paired_;
}
uint8_t VescPacketFWVersion::devVersion() const
{
  return devVersion_;
}

REGISTER_PACKET_TYPE(COMM_FW_VERSION, VescPacketFWVersion)

VescPacketRequestFWVersion::VescPacketRequestFWVersion()
: VescPacket("RequestFWVersion", 1, COMM_FW_VERSION)
{
  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketValues::VescPacketValues(std::shared_ptr<VescFrame> raw)
: VescPacket("Values", raw)
{
}

std::size_t VescPacketValues::payloadSize() const
{
  return static_cast<std::size_t>(std::distance(payload_.first, payload_.second));
}

bool VescPacketValues::usesLegacyLayout() const
{
  // The Xycar VESC firmware 2.18 returns the 56-byte layout used by the
  // original ROS1 driver: six MOS temperatures and one PCB temperature
  // precede the current fields.
  return payloadSize() == 56;
}

int16_t VescPacketValues::readInt16(std::size_t offset) const
{
  if (offset + 1 >= payloadSize()) {
    return 0;
  }
  return static_cast<int16_t>(
    (static_cast<uint16_t>(*(payload_.first + offset)) << 8) |
    static_cast<uint16_t>(*(payload_.first + offset + 1)));
}

int32_t VescPacketValues::readInt32(std::size_t offset) const
{
  if (offset + 3 >= payloadSize()) {
    return 0;
  }
  return static_cast<int32_t>(
    (static_cast<uint32_t>(*(payload_.first + offset)) << 24) |
    (static_cast<uint32_t>(*(payload_.first + offset + 1)) << 16) |
    (static_cast<uint32_t>(*(payload_.first + offset + 2)) << 8) |
    static_cast<uint32_t>(*(payload_.first + offset + 3)));
}

double VescPacketValues::temp_fet() const
{
  if (usesLegacyLayout()) {
    const auto v = std::max({readInt16(1), readInt16(3), readInt16(5), readInt16(7),
      readInt16(9), readInt16(11), readInt16(13)});
    return static_cast<double>(v) / 10.0;
  }
  return static_cast<double>(readInt16(1)) / 10.0;
}

double VescPacketValues::temp_motor() const
{
  return usesLegacyLayout() ? 0.0 : static_cast<double>(readInt16(3)) / 10.0;
}


double VescPacketValues::avg_motor_current() const
{
  return static_cast<double>(readInt32(usesLegacyLayout() ? 15 : 5)) / 100.0;
}


double VescPacketValues::avg_input_current()  const
{
  return static_cast<double>(readInt32(usesLegacyLayout() ? 19 : 9)) / 100.0;
}


double VescPacketValues::avg_id()  const
{
  return usesLegacyLayout() ? 0.0 : static_cast<double>(readInt32(13)) / 100.0;
}

double VescPacketValues::avg_iq()  const
{
  return usesLegacyLayout() ? 0.0 : static_cast<double>(readInt32(17)) / 100.0;
}


double VescPacketValues::duty_cycle_now() const
{
  return static_cast<double>(readInt16(usesLegacyLayout() ? 23 : 21)) / 1000.0;
}

double VescPacketValues::rpm() const
{
  return static_cast<double>(readInt32(usesLegacyLayout() ? 25 : 23));
}

double VescPacketValues::v_in() const
{
  return static_cast<double>(readInt16(usesLegacyLayout() ? 29 : 27)) / 10.0;
}

double VescPacketValues::amp_hours()  const
{
  return static_cast<double>(readInt32(usesLegacyLayout() ? 31 : 29)) / 1e4;
}

double VescPacketValues::amp_hours_charged()  const
{
  return static_cast<double>(readInt32(usesLegacyLayout() ? 35 : 33)) / 1e4;
}

double VescPacketValues::watt_hours()  const
{
  return static_cast<double>(readInt32(usesLegacyLayout() ? 39 : 37)) / 1e4;
}

double VescPacketValues::watt_hours_charged()  const
{
  return static_cast<double>(readInt32(usesLegacyLayout() ? 43 : 41)) / 1e4;
}

int32_t VescPacketValues::tachometer() const
{
  return readInt32(usesLegacyLayout() ? 47 : 45);
}

int32_t VescPacketValues::tachometer_abs() const
{
  return readInt32(usesLegacyLayout() ? 51 : 49);
}

int VescPacketValues::fault_code() const
{
  const auto offset = usesLegacyLayout() ? 55U : 53U;
  return offset < payloadSize() ? static_cast<int32_t>(*(payload_.first + offset)) : 0;
}


double VescPacketValues::pid_pos_now() const
{
  return usesLegacyLayout() ? 0.0 : static_cast<double>(readInt32(54)) / 1e6;
}

int32_t VescPacketValues::controller_id() const
{
  return !usesLegacyLayout() && payloadSize() > 58 ?
    static_cast<int32_t>(*(payload_.first + 58)) : 0;
}

double VescPacketValues::temp_mos1() const
{
  return static_cast<double>(readInt16(usesLegacyLayout() ? 1 : 59)) / 10.0;
}

double VescPacketValues::temp_mos2() const
{
  return static_cast<double>(readInt16(usesLegacyLayout() ? 3 : 61)) / 10.0;
}

double VescPacketValues::temp_mos3() const
{
  return static_cast<double>(readInt16(usesLegacyLayout() ? 5 : 63)) / 10.0;
}

double VescPacketValues::avg_vd()  const
{
  return usesLegacyLayout() ? 0.0 : static_cast<double>(readInt32(65)) / 1e3;
}


double VescPacketValues::avg_vq()  const
{
  return usesLegacyLayout() ? 0.0 : static_cast<double>(readInt32(69)) / 1e3;
}


REGISTER_PACKET_TYPE(COMM_GET_VALUES, VescPacketValues)

VescPacketRequestValues::VescPacketRequestValues()
: VescPacket("RequestValues", 1, COMM_GET_VALUES)
{
  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/


VescPacketSetDuty::VescPacketSetDuty(double duty)
: VescPacket("SetDuty", 5, COMM_SET_DUTY)
{
  /** @todo range check duty */

  int32_t v = static_cast<int32_t>(duty * 100000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrent::VescPacketSetCurrent(double current)
: VescPacket("SetCurrent", 5, COMM_SET_CURRENT)
{
  int32_t v = static_cast<int32_t>(current * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetCurrentBrake::VescPacketSetCurrentBrake(double current_brake)
: VescPacket("SetCurrentBrake", 5, COMM_SET_CURRENT_BRAKE)
{
  int32_t v = static_cast<int32_t>(current_brake * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetRPM::VescPacketSetRPM(double rpm)
: VescPacket("SetRPM", 5, COMM_SET_RPM)
{
  int32_t v = static_cast<int32_t>(rpm);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetPos::VescPacketSetPos(double pos)
: VescPacket("SetPos", 5, COMM_SET_POS)
{
  /** @todo range check pos */

  int32_t v = static_cast<int32_t>(pos * 1000000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 24) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 16) & 0xFF);
  *(payload_.first + 3) = static_cast<uint8_t>((static_cast<uint32_t>(v) >> 8) & 0xFF);
  *(payload_.first + 4) = static_cast<uint8_t>(static_cast<uint32_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}

/*------------------------------------------------------------------------------------------------*/

VescPacketSetServoPos::VescPacketSetServoPos(double servo_pos)
: VescPacket("SetServoPos", 3, COMM_SET_SERVO_POS)
{
  /** @todo range check pos */

  int16_t v = static_cast<int16_t>(servo_pos * 1000.0);

  *(payload_.first + 1) = static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>(static_cast<uint16_t>(v) & 0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}


VescPacketImu::VescPacketImu(std::shared_ptr<VescFrame> raw)
: VescPacket("ImuData", raw)
{
  uint32_t ind = 1;
  mask_ = static_cast<uint32_t>(
    (static_cast<uint16_t>(*(payload_.first + ind       )) << 8) +
    static_cast<uint16_t>(*(payload_.first + ind + 1   ))
  );
  ind += 2;

  if (mask_ & ((uint32_t)1 << 0)) {roll_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 1)) {pitch_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 2)) {yaw_ = getFloat32Auto(&ind);}

  if (mask_ & ((uint32_t)1 << 3)) {acc_x_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 4)) {acc_y_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 5)) {acc_z_ = getFloat32Auto(&ind);}

  if (mask_ & ((uint32_t)1 << 6)) {gyr_x_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 7)) {gyr_y_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 8)) {gyr_z_ = getFloat32Auto(&ind);}

  if (mask_ & ((uint32_t)1 << 9)) {mag_x_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 10)) {mag_y_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 11)) {mag_z_ = getFloat32Auto(&ind);}

  if (mask_ & ((uint32_t)1 << 12)) {q0_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 13)) {q1_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 14)) {q2_ = getFloat32Auto(&ind);}
  if (mask_ & ((uint32_t)1 << 15)) {q3_ = getFloat32Auto(&ind);}
}

int VescPacketImu::mask() const
{
  return mask_;
}

double VescPacketImu::getFloat32Auto(uint32_t * idx) const
{
  int pos = *idx;
  uint32_t res = static_cast<uint32_t>(
    (static_cast<uint32_t>(*(payload_.first + (pos ) )) << 24) +
    (static_cast<uint32_t>(*(payload_.first + (pos + 1) )) << 16) +
    (static_cast<uint32_t>(*(payload_.first + (pos + 2) )) << 8) +
    static_cast<uint32_t>(*(payload_.first + (pos + 3) ))
  );

  *idx += 4;

  int e = (res >> 23) & 0xFF;
  int fr = res & 0x7FFFFF;
  bool negative = res & (1u << 31);

  float f = 0.0;
  if (e != 0 || fr != 0) {
    f = static_cast<float>(fr) / (8388608.0 * 2.0) + 0.5;
    e -= 126;
  }

  if (negative) {
    f = -f;
  }
  return ldexpf(f, e);
}

double VescPacketImu::roll() const
{
  return roll_ * 180 / M_PI;  // da rad a gradi per debug  deg
}

double VescPacketImu::pitch() const
{
  return pitch_ * 180 / M_PI;
}


double VescPacketImu::yaw() const
{
  return yaw_ * 180 / M_PI;
}


double VescPacketImu::acc_x()const
{
  return acc_x_;  // g/s
}

double VescPacketImu::acc_y() const
{
  return acc_y_;
}

double VescPacketImu::acc_z() const
{
  return acc_z_;
}

double VescPacketImu::gyr_x() const
{
  return gyr_x_;  // deg/s
}

double VescPacketImu::gyr_y() const
{
  return gyr_y_;
}

double VescPacketImu::gyr_z() const
{
  return gyr_z_;
}

double VescPacketImu::mag_x() const
{
  return mag_x_;
}

double VescPacketImu::mag_y() const
{
  return mag_y_;
}

double VescPacketImu::mag_z() const
{
  return mag_z_;
}

double VescPacketImu::q_w() const
{
  return q0_;
}

double VescPacketImu::q_x() const
{
  return q1_;
}

double VescPacketImu::q_y() const
{
  return q2_;
}

double VescPacketImu::q_z() const
{
  return q3_;
}

REGISTER_PACKET_TYPE(COMM_GET_IMU_DATA, VescPacketImu)

VescPacketRequestImu::VescPacketRequestImu()
: VescPacket("RequestImuData", 3, COMM_GET_IMU_DATA)
{
  *(payload_.first + 1) = static_cast<uint8_t>(0xFF);
  *(payload_.first + 2) = static_cast<uint8_t>(0xFF);

  uint16_t crc = CRC::Calculate(
    &(*payload_.first), std::distance(payload_.first, payload_.second), VescFrame::CRC_TYPE);
  *(frame_->end() - 3) = static_cast<uint8_t>(crc >> 8);
  *(frame_->end() - 2) = static_cast<uint8_t>(crc & 0xFF);
}
/*------------------------------------------------------------------------------------------------*/
}  // namespace vesc_driver
