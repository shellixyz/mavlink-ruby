#!/usr/bin/env ruby

require 'nokogiri'
require 'crc'

# TODO: add param attributes: label, units, minValue, maxValue

class MavlinkProtocol

    V1_MARKER = 0xFE
    V2_MARKER = 0xFD
    MARKERS = { v1: V1_MARKER, v2: V2_MARKER }

    V1_HEADER_SIZE = 5
    V2_HEADER_SIZE = 9
    HEADER_SIZE = { v1: V1_HEADER_SIZE, v2: V2_HEADER_SIZE }

    SIGNATURE_SIZE = 13
    CHECKSUM_SIZE = 2

    INCOMPAT_FLAGS = {
        signed: 0x01
    }

    class HeaderV1 < Struct.new(:payload_size, :seq, :sysid, :compid, :msgid)

        def self.decode data
            new *data.unpack('C5')
        end

        def pack
            to_a.pack 'C*'
        end

    end

    class HeaderV2 < Struct.new(:payload_size, :incompat_flags, :compat_flags, :seq, :sysid, :compid, :msgid)

        def self.decode data
            #msg_id = data.byteslice(6, 3).unpack('C*').each.with_index.reduce(0) { |res, (n, i)| res + (n << i * 8) }
            msg_id = (data.byteslice(6, 3) + "\0").unpack1 'L'
            new *data.unpack('C6'), msg_id
        end

        def pack
            msgid_packed = [msgid].pack('L').byteslice(0, 3)
            to_a[0..-2].pack('C*') + msgid_packed
        end

    end

    class Enum < Struct.new(:name, :description, :entries)

        def self.from_xml_node enum_node
            name = enum_node.attr('name').to_sym
            description = enum_node.xpath('description').first&.content
            entries = Entries.from_xml_nodes enum_node.xpath('entry')
            new name, description, entries
        end

        class Entry < Struct.new(:name, :value, :description, :params)

            def self.from_xml_node entry_node
                n = entry_node
                params = n.xpath('param').sort { |a,b| a.attr('index').to_i <=> b.attr('index').to_i }.map { |p| p.content }
                new n.attr('name').to_sym, n.attr('value').to_i, n.xpath('description').first&.content, params
            end

            def inspect
                "#<#{self.class}:#{"0x%016x" % object_id} name=#{name}, value=#{value}, description=#{description}>"
            end

        end

        class Entries

            include Enumerable

            def self.from_xml_nodes entry_nodes
                new entry_nodes.map { |e| Entry.from_xml_node e }
            end

            def initialize entries
                @entries = entries
            end

            def each &block
                @entries.each &block
            end

            def [] name
                entries.find { |e| e.name == name }
            end

            def entry_names
                entries.map &:name
            end

            def find_by_value value
                entries.find { |e| e.value == value }
            end

            def append *entries
                @entries.append *entries
            end

        end

        def decode_bitmask bitmask
            entries.reduce([]) { |result, entry| result.tap { result.push entry.name if bitmask & entry.value != 0 } }
        end

    end

    class Message < Struct.new(:name, :id, :description, :fields, :field_extensions)

        class Field < Struct.new(:name, :type, :description, :enum_name, :display, :print_format, :units)

            module Type

                DATA_SIZE = {
                    int64_t: 8,
                    double: 8,
                    int32_t: 4,
                    float: 4,
                    int16_t: 2,
                    int8_t: 1,
                    char: 1
                }

                PACK_CHAR = {
                    int64_t: ?q,
                    double: ?d,
                    int32_t: ?l,
                    float: ?f,
                    int16_t: ?s,
                    int8_t: ?c,
                    char: ?a
                }

                Info = Struct.new :type, :unit_size, :signed, :count, :size, :pack_string, :unpack_string

                def self.info type
                    md = type.match /\A(?<type>(?<schar>u)?(?<base_type>int(?:8|16|32|64)_t|char|float|double))(?:_mavlink_version)?(?:\[(?<count>\d+)\])?\Z/
                    raise ArgumentError, "invalid type: #{type}" if md.nil?
                    count = md['count']&.to_i || 1
                    signed = md['schar'] != 'u'
                    base_type = md['base_type'].to_sym
                    type = md['type'].to_sym
                    unit_size = DATA_SIZE[base_type]
                    size = unit_size * count
                    pack_string = PACK_CHAR[base_type]
                    pack_string.upcase! if not signed
                    pack_string += count.to_s if count > 1
                    unpack_string = pack_string.clone
                    unpack_string[0] = 'A' if unpack_string[0] == 'a'
                    Info.new type, unit_size, signed, count, size, pack_string, unpack_string
                end

            end

            def self.from_xml_node field_node
                new field_node.attr('name').to_sym, field_node.attr('type').to_sym, field_node.content, field_node.attr('enum')&.to_sym, field_node.attr('display')&.to_sym, field_node.attr('print_format'), field_node.attr('units')
            end

            def type_info
                @type_info ||= Type.info type
            end

            def unit_size
                type_info.unit_size
            end

            def pack_string
                type_info.pack_string
            end

            def unpack_string
                type_info.unpack_string
            end

            def size # bytes
                type_info.size
            end

            def value_count
                type_info.type == :char ? 1 : type_info.count
            end

            def enum
                MavlinkProtocol.enums[enum_name] if enum_name
            end

        end

        def self.from_xml_node message_node
            name = message_node.attr('name').to_sym
            id = message_node.attr('id').to_i
            message_node_children = message_node.element_children.to_a
            description = message_node_children.shift.content if not message_node_children.empty? and message_node_children.first.name == 'description'
            message_node_children.keep_if { |node| %w{ field extensions }.include? node.name }
            extensions_index = message_node_children.map(&:name).index 'extensions'
            base_stop = extensions_index ? extensions_index - 1 : -1
            extensions_start = extensions_index + 1 if extensions_index
            base_fields = message_node_children[0..base_stop].map { |field_node| Field.from_xml_node field_node }
            extension_fields = extensions_start ? message_node_children[extensions_start..-1].map { |field_node| Field.from_xml_node field_node } : []
            new name, id, description, base_fields, extension_fields
        end

        def fields_reordered
            @fields_reordered ||= fields.sort { |a,b| b.unit_size <=> a.unit_size }
        end

        def all_fields
            @all_fields ||= fields + field_extensions
        end

        def all_fields_reordered
            @all_fields_reordered ||= fields_reordered + field_extensions
        end

        def pack_string
            @pack_string ||= all_fields_reordered.map(&:pack_string).join
        end

        def unpack_string
            @unpack_string ||= all_fields_reordered.map(&:unpack_string).join
        end

        def expected_payload_size
            @expected_payload_size ||= all_fields.map(&:size).sum
        end

        def field_names
            @field_names ||= fields.map &:name
        end

        def all_field_names
            @all_field_names ||= all_fields.map &:name
        end

        def decode payload
            missing_bytes = expected_payload_size - payload.bytesize
            payload += ?\0 * missing_bytes if missing_bytes > 0
            data = payload.unpack(unpack_string).compact

            data_grouped = all_fields_reordered.reduce([]) { |r, field| r.push data.shift(*(field.value_count > 1 ? [field.value_count] : [])) }

            data_grouped.map!.with_index do |value, index|
                field = all_fields_reordered[index]
                case
                when field.display == :bitmask
                    field.enum.decode_bitmask value
                when field.enum
                    enum_entry = field.enum.entries.find_by_value value
                    raise ArgumentError, "No entry with value #{value} in enum #{field.enum.name}" if enum_entry.nil?
                    enum_entry.name
                else
                    value
                end
            end

            all_fields_reordered.map(&:name).zip(data_grouped).to_h
        end

        def encode_payload *values
            values_hash = nil
            if values.count == 1 and values.first.is_a? Hash
                values_hash = values.first
                unknown_keys = values_hash.keys - all_field_names
                raise ArgumentError, "unknown keys: #{unknown_keys.join(', ')}" unless unknown_keys.empty?
                missing_keys = all_field_names - values_hash.keys
                raise ArgumentError, "missing keys: #{missing_keys.join(', ')}" unless missing_keys.empty?
                hash_argument = true
            else
                raise ArgumentError, "values: #{values.count}, expected #{all_fields.count}" if values.count != all_fields.count
            end
            payload_values = all_fields_reordered.map do |field|
                value = hash_argument ? values_hash[field.name] : values[all_field_names.index(field.name)]
                if field.enum and value.is_a?(Symbol)
                    enum_entry = field.enum.entries[value]
                    raise ArgumentError, "invalid enum entry name: #{value}" if enum_entry.nil?
                    value = enum_entry.value
                end
                value
            end
            payload_values.flatten.pack pack_string
        end

        def encode_v1 seq, *values
            payload = encode_payload *values
            header = HeaderV1.new(payload.bytesize, seq, 1, 1, id).pack
            crc = MavlinkProtocol.packed_crc header + payload + crc_extra
            V1_MARKER.chr + header + payload + crc
        end

        def self.compact_v2_payload payload
            index = payload.bytesize
            payload.reverse.each_byte do |byte|
                break if byte != 0 or index == 1
                index -= 1
            end
            index < payload.bytesize ? payload.byteslice(0...index) : payload
        end

        def encode_v2 seq, *values
            payload = self.class.compact_v2_payload encode_payload(*values)
            header = HeaderV2.new(payload.bytesize, 0, 0, seq, 0xff, 0xbe, id).pack
            crc = MavlinkProtocol.packed_crc header + payload + crc_extra
            V2_MARKER.chr + header + payload + crc
        end

        def crc_extra
            crc = MavlinkProtocol::CRC.new
            crc.update name.to_s + ' '
            fields_reordered.each do |field|
                type_info = field.type_info
                crc.update type_info.type.to_s + ' '
                crc.update field.name.to_s + ' '
                crc.update type_info.count.chr if type_info.count > 1
            end
            ((crc.to_i & 0xff) ^ (crc.to_i >> 8)).chr
        end

        def inspect
            "#<#{self.class}:#{"0x%016x" % object_id} name=#{name}>"
        end

    end

    module CRC

        def self.new
            ::CRC::CRC16_CCITT.new 0xffff
        end

    end

    module FindById

        def find_by_id id
            values.find { |m| m.id == id }
        end

    end

    def self.crc data
        ::CRC.crc16_ccitt data, 0xffff
    end

    def self.packed_crc data
        [ crc(data) ].pack 'S'
    end

    def self.read_header io, version
        header_size = HEADER_SIZE[version]
        raise ArgumentError, "invalid version: #{version}" if header_size.nil?
        io.read header_size
    end

    def self.decode_header header_data, version
        raise ArgumentError, "invalid version: #{version}" unless %i{ v1 v2 }.include? version
        header_class = const_get "Header" + version.to_s.capitalize
        header_class.decode header_data
    end

    def self.xml_docs
        xml_files = File.join(File.dirname(__FILE__), 'mavlink/message_definitions/v1.0/*.xml')
        @xml_docs ||= Dir.glob(xml_files).map do |f|
            Nokogiri::XML File.open(f)
        end
    end

    def self.enums
        if @enums.nil?
            @enums = Hash.new
            xml_docs.each do |xml_doc|
                xml_doc.xpath('/mavlink/enums/enum').each do |enum_node|
                    enum = Enum.from_xml_node enum_node
                    if @enums.has_key? enum.name
                        @enums[enum.name].entries.append *enum.entries
                    else
                        @enums[enum.name] = enum
                    end
                end
            end
        end
        @enums
    end

    def self.messages
        @messages ||= xml_docs.flat_map do |xml_doc|
            xml_doc.xpath('/mavlink/messages/message').map do |message_node|
                message = Message.from_xml_node message_node
                [ message.name, message]
            end
        end.to_h.extend FindById
    end

end

if $0 == __FILE__
    require 'pry'
    MavlinkProtocol.pry
end
