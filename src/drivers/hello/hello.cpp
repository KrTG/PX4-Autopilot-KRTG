#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_io_heater.h>

class HelloWorld: public ModuleBase<HelloWorld>, public px4::ScheduledWorkItem
{

public:
    HelloWorld();
    ~HelloWorld() = default;

    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    static int task_spawn(int argc, char *argv[]);

private:
    void Run() override;
	uORB::Publication<helloworld_status_s> _helloworld_status_pub{ORB_ID(helloworld_status)};
};


HelloWorld:HelloWorld() :
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
    _helloworld_status_pub.advertise();
}

int HelloWorld::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_INFO("not running");
        return PX4_ERROR;
    }

    return print_usage("Unrecognized command.");
}

int HelloWorld::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s\n\n", reason);
	}

    PRINT_MODULE_DESCRIPTION(
    R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to say hello.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("hello", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int HelloWorld::task_spawn(int argc, char *argv[])
{
    HelloWorld *instance = new HelloWorld();

    if (!instance) {
        PX4_ERR("Failed to create hello instance.");
		return PX4_ERROR;
    }

    _object.store(instance);
    instance->ScheduleOnInterval(1_s);
    _task_id = task_id_is_work_queue;

    return 0;
}

void HelloWorld::Run()
{
    helloworld_status_s status{};
    status.timestamp = hrt_absolute_time();
    status.hello = 12.75;
    _helloworld_status_pub.publish(status);
}

extern "C" __EXPORT int hello_main(int argc, char *argv[])
{
    return HelloWorld::main(argc, argv);
}
