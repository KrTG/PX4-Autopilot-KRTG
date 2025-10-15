#include <uORB/topics/force_sensor.h>

class ForceSensorMavlink : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new ForceSensorMavlink(mavlink);
    }
    const char *get_name() const
    {
        return ForceSensorMavlink::get_name_static();
    }
    static const char *get_name_static()
    {
        return "FORCE_SENSOR";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_FORCE_SENSOR;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_FORCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::SubscriptionMultiArray<force_sensor_s, force_sensor_s::MAX_INSTANCES> _force_sensor_subs{ORB_ID::force_sensor};
    ForceSensorMavlink(ForceSensorMavlink &);
    ForceSensorMavlink& operator = (const ForceSensorMavlink &);

protected:
    explicit ForceSensorMavlink(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

	bool send() override
	{
		bool updated = false;

		for (auto &force_sub : _force_sensor_subs) {
			force_sensor_s force_sensor;

			if (force_sub.update(&force_sensor)) {
				mavlink_force_sensor_t force_msg{};

				force_msg.timestamp = force_sensor.timestamp;
				force_msg.id = force_sensor.device_id - 1;
				force_msg.force = force_sensor.force;

				mavlink_msg_force_sensor_send_struct(_mavlink->get_channel(), &force_msg);
				updated = true;
			}
		}

		return updated;
	}

};
