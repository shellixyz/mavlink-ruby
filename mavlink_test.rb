
require 'serialport'
require 'crc'
require 'pry'
require_relative 'mavlink_protocol'

sp = SerialPort.new '/dev/ttyACM0', baud: 115200
sp.flow_control = SerialPort::NONE
sp.flush_input

#sp = File.open 'mavlink_sample.raw'

n = 0
loop do

    sp.find MavlinkProtocol::V1_MARKER.chr
    sp.getbyte

    header_data = sp.read MavlinkProtocol::V1_HEADER_SIZE
    header = MavlinkProtocol::HeaderV1.decode header_data
    payload = sp.read header.payload_size
    cksum = sp.read(MavlinkProtocol::CHECKSUM_SIZE).unpack1 'S'
    message_def = MavlinkProtocol.messages.find_by_id header.msgid
    if message_def
        ccksum = MavlinkProtocol.crc header_data + payload + message_def.crc_extra
        if cksum == ccksum
        message = message_def.decode payload
        puts message_def.name.upcase
        pp message
        end
    end

end
