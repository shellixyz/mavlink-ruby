
require 'serialport'
require 'set'
require 'timeout'
require 'monitor'
require_relative 'mavlink_protocol'
require_relative 'mavlink_helpers'

DEBUG = false

class Mavlink

    CHECKSUM_SIZE = MavlinkProtocol::CHECKSUM_SIZE
    Packet = Struct.new :message, :content


    class FailedToGetParam < StandardError; end
    class FailedToSetParam < StandardError; end

    include MavlinkHelpers

    class SyncLookup

        def initialize hash
            @hash = hash
        end

        def keys
            hash.synchronize { hash.keys }
        end

        def [] key
            hash.synchronize { hash[key] }
        end

        private

        attr_reader :hash, :mutex

    end

    class WaitForMessageCond < Struct.new(:name, :cond, :conditions)

        def wait timeout = nil
            cond.wait timeout
        end

    end

    class CommandError < StandardError

        def initialize result, msg
            super msg
            @result = result
        end

        attr_reader :result

    end

    module Error

        class Timeout < StandardError; end

    end

    def initialize serial_device = '/dev/ttyACM0', serial_baud = 115200, sysid: 1, compid: 1
        @serial_device = serial_device
        @serial_baud = serial_baud
        @wait_timeout = 10
        @sysid = sysid
        @compid = compid
        _init
    end

    def with_recv_pool
        raise ArgumentError, 'no block given' unless block_given?
        recv_pool_mutex.synchronize { yield irecv_pool }
    end

    def with_keep_pool
        raise ArgumentError, 'no block given' unless block_given?
        keep_pool_mutex.synchronize { yield ikeep_pool }
    end

    def clear_keep_pool name = nil
        keep_pool_mutex.synchronize do
            if name
                keep_pool[name].clear
            else
                keep_pool.clear
            end
        end
    end

    def keep_all_messages name
        already_keeping = keep_all_messages_names.include? name
        keep_all_messages_names << name if not already_keeping
        if block_given?
            yield.tap do
                unkeep_all_messages name if already_keeping
            end
        else
            nil
        end
    end

    def unkeep_all_messages name
        keep_all_messages_names.delete name
        ikeep_pool.delete name
        nil
    end

    def send_message name, seq, *values
        message = MavlinkProtocol.messages[name]
        raise ArgumentError, "invalid message name: #{name}" if message.nil?
        sp.write message.encode_v2(seq, *values).tap {|x| pp x.unpack('C*') if DEBUG }
        nil
    end

    def send_message_wait_for_reply in_message_name, out_message_name, seq, *values
        cond = add_wait_cond_for_message in_message_name
        monitoring_data = irecv_pool.synchronize do
            irecv_pool.delete in_message_name
            send_message out_message_name, seq, *values
            cond.wait wait_timeout
            remove_wait_cond_for_message cond
            message = irecv_pool[in_message_name]
            raise Error::Timeout if message.nil?
            message.content
        end
    end

    def request_params # TODO: rewrite with conditions
        keep_all_messages :PARAM_VALUE do
            send_message :PARAM_REQUEST_LIST, 1, sysid, compid
            loop { break unless keep_pool[:PARAM_VALUE].empty? } # wait for one param value
            param_count = keep_pool[:PARAM_VALUE].first.content[:param_count]
            loop { break if keep_pool[:PARAM_VALUE].map { |p| p.content[:param_id] }.uniq.count == param_count }
            @params = keep_pool[:PARAM_VALUE].each.with_object({}) do |packet, params|
                params[packet.content[:param_id]] = packet.content[:param_value]
            end
        end
    end

    def param_value name
        cond = add_wait_cond_for_message :PARAM_VALUE, param_id: name.to_s
        irecv_pool.synchronize do
            irecv_pool.delete :PARAM_VALUE
            send_message :PARAM_REQUEST_READ, 1, sysid, compid, name.to_s, -1
            cond.wait wait_timeout
            remove_wait_cond_for_message cond
            message = irecv_pool[:PARAM_VALUE]
            raise FailedToGetParam, "Failed to get param value for #{name}, it might not exist" if message.nil?
            message.content[:param_value]
        end
    end

    def set_param_value name, value
        param_value name unless @param_type_cache.has_key? name.to_s
        param_type = @param_type_cache[name.to_s]
        cond = add_wait_cond_for_message :PARAM_VALUE, param_id: name.to_s
        irecv_pool.synchronize do
            irecv_pool.delete :PARAM_VALUE
            send_message :PARAM_SET, 1, sysid, compid, name.to_s, value, param_type
            cond.wait wait_timeout
            remove_wait_cond_for_message cond
            message = irecv_pool[:PARAM_VALUE]
            raise FailedToSetParam, "Failed to set param #{name}=#{value}, it might not exist" if message.nil?
            message.content[:param_value].tap do |set_value|
                # check doesn't seem to work
                #if set_value > value + 0.000001 or set_value < value - 0.000001
                    #raise "param #{name}: set value (#{set_value}) different from what was requested (#{value})"
                #end
            end
        end
    end

    def add_wait_cond_for_message name, conditions = {}
        WaitForMessageCond.new(name, irecv_pool.new_cond, conditions).tap do |cond_struct|
            @wait_for_message_conds << cond_struct
        end
    end

    def remove_wait_cond_for_message cond
        @wait_for_message_conds.delete cond
        nil
    end

    def wait_for_message name, conditions = {}
        cond = add_wait_cond_for_message name, conditions
        irecv_pool.synchronize do
            cond.wait wait_timeout
            message = irecv_pool[name]
            raise Error::Timeout if message.nil?
            yield message if block_given?
        end.tap do
            remove_wait_cond_for_message cond
        end
    end

    def loop_wait_for_message name, conditions = {}
        cond = add_wait_cond_for_message name, conditions
        irecv_pool.synchronize do
            loop do
                cond.wait wait_timeout
                message = irecv_pool[name]
                raise Error::Timeout if message.nil?
                yield message
            end
        end.tap do
            remove_wait_cond_for_message cond
        end
    end

    def with_voltage_and_current period = 0.5
        set_message_interval :SYS_STATUS, period
        loop_wait_for_message :SYS_STATUS do |message|
            raise "battery sensor not enabled" unless message.content[:onboard_control_sensors_enabled].include? :MAV_SYS_STATUS_SENSOR_BATTERY
            voltage = message.content[:voltage_battery] / 1000.0
            current = message.content[:current_battery] / 100.0
            yield voltage, current
        end
    end

    def display_voltage_and_current # XXX just a demo probably doesn't have anything to do here
        with_voltage_and_current do |v,c|
            puts "voltage: #{"%2.3f" % v}V, current: #{"%2.3f" % c}A"
        end
    end

    def command_long_noack command, *params
        raise ArgumentError, "too many params: #{params.count} (max 7)" if params.count > 7
        params.append *[0]*(7-params.count)
        send_message :COMMAND_LONG, 1, sysid, compid, command, 0, *params
    end

    def command_long command, *params
        command_long_noack command, *params
        result = get_ack command
        raise CommandError.new(result, "command #{command} failed: #{result}") unless result == :MAV_RESULT_ACCEPTED
    end

    # request message at specified interval (s)
    def set_message_interval message, interval
        message_id = message.is_a?(Integer) ? message : MavlinkProtocol.messages[message]&.id
        raise ArgumentError, "invalid message: #{message}" if message_id.nil?
        interval_us = interval * 1e6
        command_long :MAV_CMD_SET_MESSAGE_INTERVAL, message_id, interval_us
    end

    def message_interval message
        message_id = message.is_a?(Integer) ? message : MavlinkProtocol.messages[message]&.id
        raise ArgumentError, "invalid message: #{message}" if message_id.nil?
        command_long_noack :MAV_CMD_GET_MESSAGE_INTERVAL, message_id
        wait_for_message :MESSAGE_INTERVAL, message_id: message_id do |message|
            message.content[:interval_us] / 1e6
        end
    end

    def reset_params
        command_long :MAV_CMD_PREFLIGHT_STORAGE, 2, 0, 0
    end

    def reboot_fc reopen: true
        command_long :MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1
        close
        sleep 10
        _init if reopen
        nil
    end

    def get_ack command
        wait_for_message(:COMMAND_ACK, command: command) { |message| message.content[:result] }
    end

    def close
        listen_thread.kill if listening?
        sp.close
        @sp = nil
    end

    attr_reader :recv_pool, :keep_pool, :params, :serial_device, :serial_baud
    attr_accessor :wait_timeout, :sysid, :compid

    attr_reader :sp, :ibuf, :listen_thread, :keep_all_messages_names, :irecv_pool, :ikeep_pool

    private

    def _init
        @sp = SerialPort.new serial_device
        sp.baud = serial_baud
        sp.flow_control = SerialPort::NONE
        sp.flush_input
        @ibuf = String.new
        @keep_all_messages_names = Set.new
        @param_type_cache = Hash.new
        @irecv_pool = Hash.new.extend MonitorMixin
        @ikeep_pool = Hash.new { |h,k| h[k] = Array.new }.extend MonitorMixin
        @recv_pool = SyncLookup.new irecv_pool
        @keep_pool = SyncLookup.new ikeep_pool
        @wait_for_message_conds = Array.new
        listen
    end

    def listen
        @listen_thread = Thread.new do
            sp.flush_input
            loop do
                packet = _read_packet
                message_name = packet.message.name
                puts "#{Time.now} -- #{message_name}: #{packet.content.inspect}" if DEBUG
                irecv_pool.synchronize do
                    irecv_pool[message_name] = packet
                    @param_type_cache[packet.content[:param_id]] = packet.content[:param_type] if message_name == :PARAM_VALUE
                    @wait_for_message_conds.each do |cond_struct|
                        if cond_struct.name == message_name
                            cond_struct.cond.signal if cond_struct.conditions.all? { |name, value| packet.content[name] == value }
                        end
                    end
                end
                ikeep_pool.synchronize { ikeep_pool[message_name] << packet } if keep_all_messages_names.include? message_name
            end
        end
    end

    def listening?
        if listen_thread
            listen_thread.status != false
        else
            false
        end
    end

    def _read_packet timeout = nil
        Timeout.timeout timeout do
            loop do
                byte = nil
                if ibuf.empty?
                    byte = sp.getbyte
                else
                    if next_marker_index = (0...ibuf.bytesize).find { |idx| MavlinkProtocol::MARKERS.values.include? ibuf.bytes[idx] }
                        byte = ibuf.bytes[next_marker_index]
                        ibuf.replace ibuf.byteslice(next_marker_index+1..-1)
                    else
                        ibuf.clear
                    end
                end

                if MavlinkProtocol::MARKERS.values.include? byte
                    proto_version = MavlinkProtocol::MARKERS.invert[byte]
                    header_size = MavlinkProtocol::HEADER_SIZE[proto_version]
                    needed_bytes = header_size
                    if ibuf.bytesize < needed_bytes
                        data = sp.read(needed_bytes - ibuf.bytesize)
                        ibuf << data if data
                    end
                    if ibuf.bytesize == needed_bytes
                        header = MavlinkProtocol.decode_header ibuf, proto_version
                        needed_bytes += header.payload_size + CHECKSUM_SIZE
                        while ibuf.bytesize < needed_bytes
                            data = sp.read(needed_bytes - ibuf.bytesize)
                            ibuf << data if data
                        end
                        rcksum = ibuf.byteslice(header_size + header.payload_size, CHECKSUM_SIZE).unpack1 'S'
                        if message = MavlinkProtocol.messages.find_by_id(header.msgid)
                            header_data = ibuf.byteslice 0, header_size
                            payload = ibuf.byteslice header_size, header.payload_size
                            ccksum = MavlinkProtocol.crc header_data + payload + message.crc_extra
                            if rcksum == ccksum
                                content = message.decode payload
                                ibuf.replace ibuf.byteslice(needed_bytes..-1)
                                return Packet.new(message, content)
                            end
                        end
                    end
                end
            end
        end
    end

    def self.decode_packet packet_data
        packet_data = packet_data.clone
        packet_data = packet_data.unpack 'C*' unless packet_data.is_a?(Array)
        marker_byte = packet_data.shift
        packet_data = packet_data.pack 'C*'
        proto_version = MavlinkProtocol::MARKERS.invert[marker_byte]
        header_size = MavlinkProtocol::HEADER_SIZE[proto_version]
        header = MavlinkProtocol.decode_header packet_data, proto_version
        rcksum = packet_data.byteslice(header_size + header.payload_size, CHECKSUM_SIZE).unpack1 'S'
        message = MavlinkProtocol.messages.find_by_id(header.msgid)
        raise "message not found for msgid: #{header.msgid}" if message.nil?
        header_data = packet_data.byteslice 0, header_size
        payload = packet_data.byteslice header_size, header.payload_size
        ccksum = MavlinkProtocol.crc header_data + payload + message.crc_extra
        raise "bad CRC (expected #{rcksum}, got #{ccksum}" if rcksum != ccksum
        content = message.decode payload
        Packet.new(message, content)
    end

end

if $0 == __FILE__
    require 'pry'
    port = ARGV[0] || '/dev/ttyACM0'
    baud = ARGV[1] || 115200
    mav = Mavlink.new port, baud
    mav.pry
end
