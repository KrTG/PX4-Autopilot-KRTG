#ifndef WEIGHT_SENSOR_HPP
#define WEIGHT_SENSOR_HPP

#include <uORB/topics/force_sensor.h>

class MavlinkStreamWeightSensor : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamWeightSensor(mavlink);
    }
    const char *get_name() const override
    {
        return MavlinkStreamWeightSensor::get_name_static();
    }
    static const char *get_name_static()
    {
        return "WEIGHT_SENSOR";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_WEIGHT_SENSOR;
    }
    uint16_t get_id() override
    {
        return get_id_static();
    }
    unsigned get_size() override
    {
        return MAVLINK_MSG_ID_WEIGHT_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::SubscriptionMultiArray<force_sensor_s, force_sensor_s::MAX_INSTANCES> _force_sensor_subs{ORB_ID::force_sensor};
    MavlinkStreamWeightSensor(MavlinkStreamWeightSensor &);
    MavlinkStreamWeightSensor& operator = (const MavlinkStreamWeightSensor &);

protected:
    explicit MavlinkStreamWeightSensor(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

	bool send() override
	{
		bool updated = false;

		for (auto &force_sub : _force_sensor_subs) {
			force_sensor_s force_sensor;

			if (force_sub.update(&force_sensor)) {
				mavlink_weight_sensor_t weight_msg{};

				weight_msg.timestamp = force_sensor.timestamp;
				weight_msg.id = force_sensor.device_id - 1;
				weight_msg.weight = force_sensor.force;

				mavlink_msg_weight_sensor_send_struct(_mavlink->get_channel(), &weight_msg);
				updated = true;
			}
		}

		return updated;
	}

};

#endif // WEIGHT_SENSOR_HPP
