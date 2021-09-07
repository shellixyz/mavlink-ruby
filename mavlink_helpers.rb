
module MavlinkHelpers

    def set_relay index, setting
        setting =
            case
            when setting.is_a?(Integer)
                setting
            when setting.is_a?(Symbol)
                {off: 0, on: 1}[setting].tap do |converted|
                    raise ArgumentError, "Invalid symbol #{setting} (valid = :on and :off)" if converted.nil?
                end
            when [true, false].include?(setting)
                setting ? 1 : 0
            end
        command_long :MAV_CMD_DO_SET_RELAY, index, setting
    end

end
