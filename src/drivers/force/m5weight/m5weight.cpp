#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/force_sensor.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>

// I2C
#define DEVICE_DEFAULT_ADDR 0x26

// SCALES REGISTER
#define WEIGHT_I2C_RAW_ADC_REG         0x00
#define WEIGHT_I2C_CAL_DATA_REG        0x10
#define WEIGHT_I2C_SET_GAP_REG         0x40
#define WEIGHT_I2C_SET_OFFESET_REG     0x50
#define WEIGHT_I2C_CAL_DATA_INT_REG    0x60
#define WEIGHT_I2C_CAL_DATA_STRING_REG 0x70
#define WEIGHT_I2C_FILTER_REG          0x80
#define JUMP_TO_BOOTLOADER_REG         0xFD
#define FIRMWARE_VERSION_REG           0xFE
#define I2C_ADDRESS_REG                0xFF

using namespace time_literals;

class M5Weight: public ModuleBase<M5Weight>, public px4::ScheduledWorkItem, public device::I2C
{

public:
    M5Weight(uint8_t bus, uint16_t address, uint32_t bus_frequency);
    ~M5Weight() override;

    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    static int task_spawn(int argc, char *argv[]);

private:
    void Run() override;
	uORB::Publication<force_sensor_s> _force_sensor_pub{ORB_ID(force_sensor)};

    bool readBytes(uint8_t reg, uint8_t *buffer, uint8_t length);

    float getWeight();
    int32_t getWeightInt();
    bool getWeightString(char* data);
    int32_t getRawADC();

    uint8_t getLPFilter();
    uint8_t getAvgFilter();
    uint8_t getEmaFilter();
    float getGapValue();
    uint8_t getI2CAddress();
    uint8_t getFirmwareVersion();

    uint8_t lp_filter;
    uint8_t avg_filter;
    uint8_t ema_filter;
    float gap_value;
    uint8_t i2c_address;
    uint8_t firmware_version;
};


M5Weight::M5Weight(uint8_t bus, uint16_t address, uint32_t bus_frequency) :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
    I2C(DRV_FORCE_M5WEIGHT, MODULE_NAME, bus, address, bus_frequency)
{
    _force_sensor_pub.advertise();
    init();

    lp_filter = getLPFilter();
    avg_filter = getAvgFilter();
    ema_filter = getEmaFilter();
    gap_value = getGapValue();
    i2c_address = getI2CAddress();
    firmware_version = getFirmwareVersion();
}

M5Weight::~M5Weight() {
    ScheduleClear();
}

int M5Weight::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_INFO("not running");
        return PX4_ERROR;
    }

    return print_usage("Unrecognized command.");
}

int M5Weight::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s\n\n", reason);
	}

    PRINT_MODULE_DESCRIPTION(
    R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to query the M5 weight sensor.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("force", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(DEVICE_DEFAULT_ADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int M5Weight::task_spawn(int argc, char *argv[])
{
    // TODO: Add config
    M5Weight *instance = new M5Weight(2, DEVICE_DEFAULT_ADDR, 100000);

    if (!instance) {
        PX4_ERR("Failed to create hello instance.");
		return PX4_ERROR;
    }

    _object.store(instance);
    instance->ScheduleOnInterval(1_s);
    _task_id = task_id_is_work_queue;

    return 0;
}

bool M5Weight::readBytes(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    if (transfer(&reg, 1, nullptr, 0) != PX4_OK) {
        return false;
    }
    if (transfer(nullptr, 0, buffer, length) != PX4_OK) {
        return false;
    }
    return true;
}

uint8_t M5Weight::getLPFilter()
{
    uint8_t data;
    if (readBytes(WEIGHT_I2C_FILTER_REG, &data, 1)) {
        return data;
    }
    return 69; // Return error code on failure
}

uint8_t M5Weight::getAvgFilter()
{
    uint8_t data;
    // Register is at WEIGHT_I2C_FILTER_REG + 1
    if (readBytes(WEIGHT_I2C_FILTER_REG + 1, &data, 1)) {
        return data;
    }
    return 69; // Return error code on failure
}

uint8_t M5Weight::getEmaFilter()
{
    uint8_t data;
    // Register is at WEIGHT_I2C_FILTER_REG + 2
    if (readBytes(WEIGHT_I2C_FILTER_REG + 2, &data, 1)) {
        return data;
    }
    return 69; // Return error code on failure
}

float M5Weight::getWeight() {
    uint8_t data[4];
    float c;

    if (readBytes(WEIGHT_I2C_CAL_DATA_REG, data, 4)) {
        memcpy(&c, data, 4);
        return c;
    };
    return 69.0f; // Return error code on failure
}

int32_t M5Weight::getWeightInt()
{
    uint8_t data[4];
    if (readBytes(WEIGHT_I2C_CAL_DATA_INT_REG, data, 4)) {
        return (data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    }
    return 69; // Return error code on failure
}

bool M5Weight::getWeightString(char* data)
{
    if (readBytes(WEIGHT_I2C_CAL_DATA_STRING_REG, (uint8_t *)data, 16)) {
        data[16] = '\0';
        return true;
    }
    // Return the string "69" as the error code
    return false;
}

float M5Weight::getGapValue()
{
    uint8_t data[4];
    float c;
    if (readBytes(WEIGHT_I2C_SET_GAP_REG, data, 4)) {
        memcpy(&c, data, 4);
        return c;
    }
    return 69.0f; // Return error code on failure
}

int32_t M5Weight::getRawADC()
{
    uint8_t data[4];
    if (readBytes(WEIGHT_I2C_RAW_ADC_REG, data, 4)) {
        return (data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
    }
    return 69; // Return error code on failure
}

uint8_t M5Weight::getI2CAddress()
{
    uint8_t data;
    if (readBytes(I2C_ADDRESS_REG, &data, 1)) {
        return data;
    }
    return 69; // Return error code on failure
}

uint8_t M5Weight::getFirmwareVersion()
{
    uint8_t data;
    if (readBytes(FIRMWARE_VERSION_REG, &data, 1)) {
        return data;
    }
    return 69; // Return error code on failure
}

void M5Weight::Run()
{
    float force = getWeight();
    int32_t forceInt = getWeightInt();
    int32_t rawAdc = getRawADC();

    force_sensor_s status{};
    status.timestamp = hrt_absolute_time();
    status.device_id = i2c_address;

    status.lp_filter = lp_filter;
    status.avg_filter = avg_filter;
    status.ema_filter = ema_filter;
    status.gap_value = gap_value;
    status.i2c_address = i2c_address;
    status.firmware_version = firmware_version;

    status.force = force;
    status.force_int = forceInt;
    getWeightString(status.force_string);
    status.raw_adc = rawAdc;

    _force_sensor_pub.publish(status);
}

extern "C" __EXPORT int m5weight_main(int argc, char *argv[])
{
    return M5Weight::main(argc, argv);
}
