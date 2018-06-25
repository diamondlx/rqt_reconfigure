#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <log4cxx/logger.h>
#include <log4cxx/logstring.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <rcl/rcl.h>
#include <rclcpp/rclcpp.hpp>
#include "zoro_utils/zoro_utils.h"

/*
#define LOG(func, name, format, ...)\
{\
    log4cxx::LoggerPtr logger = get_logger(name);\
    char log_str[128];\
    snprintf(log_str, sizeof(log_str), format, ## __VA_ARGS__);\
    puts(log_str);\
    logger->func(log_str);\
}

#define LOG_DEBUG(name, format, ...) LOG(debug, name, format, ## __VA_ARGS__)
#define LOG_INFO(name, format, ...) LOG(info, name, format, ## __VA_ARGS__)
#define LOG_WARN(name, format, ...) LOG(warn, name, format, ## __VA_ARGS__)
#define LOG_ERROR(name, format, ...) LOG(error, name, format, ## __VA_ARGS__)
#define LOG_FATAL(name, format, ...) LOG(fatal, name, format, ## __VA_ARGS__)
*/


namespace zoro
{
namespace utils
{

static std::shared_ptr<rclcpp::SyncParametersClient> _init_client(std::string node_name )
{
    if(!rcl_ok())
        fprintf(stderr, "rcl is not ok\n");

    auto node = rclcpp::Node::make_shared(node_name);
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
            node, "zoro_set_and_get_parameters" );
    int i = 10;
    while ( !client->wait_for_service(std::chrono::microseconds(50)) && i>0 )
    {
        --i;
        if (!rclcpp::ok())
        {
            LOG_ERROR(__func__, "Interrupted while waiting for the service. Exiting.");
                return 0;
        }
        LOG_ERROR(__func__, "service not available, waiting again try %d", i);
    }
    if (i==0)
        return nullptr;
    return client;
}

bool start_service()
{
    int argc = 1;
    char *argv[] = {(char*)("parameters"), nullptr};
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("zoro_set_and_get_parameters");
    auto parameter_service = std::make_shared<rclcpp::ParameterService>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
}

/* set parameters */
template<typename T>
bool set_parameter(const std::string &key, T val)
{
   static auto client=_init_client("set_parameters_client");
   if ( !client )
   {
       client = _init_client("set_parameters_client");
       return false;
   }
   std::vector<rclcpp::parameter::ParameterVariant> para;
   para.push_back(rclcpp::parameter::ParameterVariant(key, val));
   auto rets = client->set_parameters(para);
   for (auto &it : rets)
   {
       if ( !it.successful )
       {
           fprintf(stderr, "Fail to set parameter : %s", key.c_str());
           return false;
       }
   }
   return true;
}

bool set_parameter(const std::string &key, bool val)
{
    return set_parameter<bool>(key, val);
}
bool set_parameter(const std::string &key, int val)
{
    return set_parameter<int>(key, val);
}
bool set_parameter(const std::string &key, double val)
{
    return set_parameter<double>(key, val);
}
bool set_parameter(const std::string &key,const std::string val)
{
    return set_parameter<std::string>(key, val);
}

/* get parameters */

#define get_parameter_template(type) \
    static auto client = _init_client("get_parameters_client");\
    if (!client)\
    {\
       client =_init_client("get_parameters_client");\
       return false;\
    }\
    if (client->has_parameter(key))\
    {\
      std::vector<std::string> v;\
      v.push_back(key);\
      for (auto &it : client->get_parameters(v) )\
      {\
          val = it.as_##type();\
      }\
      return true;\
    } else {\
        return false;\
    }

bool get_parameter(const std::string &key, bool &val)
{
    get_parameter_template(bool)
}

bool get_parameter(const std::string &key, int &val)
{
    get_parameter_template(int)
}

bool get_parameter(const std::string &key, double &val)
{
    get_parameter_template(double)
}

bool get_parameter(const std::string &key, std::string &val)
{
    get_parameter_template(string)
}


/* logger  */
void zoro_init(const char *logprefix, const char *logconfig)
{
  char *prefix;

  if (logprefix == NULL)
  {
     logprefix = "zoro";
  }

  if (logconfig == NULL)
  {
     prefix = getenv("ZORO_HOME");    
     if ( prefix == NULL )
     {
         prefix = getenv("HOME");

         if ( prefix == NULL )
         {
             prefix = (char*)("/tmp");
         }
     }
  }

  char path[4096];

  snprintf(path, 4096, "%s/.zoro", prefix);

  struct stat st;
  stat(path, &st);
  if ( stat(path, &st) != 0 ) /* path not exists */
  {
     mkdir(path, 0777);
  }

  snprintf(path, 4096, "%s/.zoro/zoro_log.cfg", prefix);

  if ( stat(path, &st) !=0 ) /* log config file not exists */
  {
    FILE *fp;

    fp = fopen(path, "w");
  
    if (fp==NULL)
        return ;

    fprintf(fp, "log4j.rootLogger=WARN, logfile\n");
    fprintf(fp, "\n");
    fprintf(fp, "log4j.appender.stdout=org.apache.log4j.ConsoleAppender\n");
    fprintf(fp, "log4j.appender.stdout.layout=org.apache.log4j.PatternLayout\n");
    fprintf(fp, "log4j.appender.stdout.layout.ConversionPattern=%%d [%%t] %%-5p %%c - %%m %%n\n");
    fprintf(fp, "\n");
    fprintf(fp, "log4j.appender.logfile=org.apache.log4j.RollingFileAppender\n");
    fprintf(fp, "log4j.appender.logfile.File=%s/.zoro/${logprefix}_${pid}.log\n", prefix);
    fprintf(fp, "log4j.appender.logfile.MaxFileSize=1MB\n");
    fprintf(fp, "log4j.appender.logfile.MaxBackupIndex=10\n");
    fprintf(fp, "log4j.appender.logfile.layout=org.apache.log4j.PatternLayout\n");
    fprintf(fp, "log4j.appender.logfile.layout.ConversionPattern=%%d [%%t] %%-5p %%c - %%m %%n\n");
    fprintf(fp, "\n");
    fprintf(fp, "#log4j.logger.talker=info, stdout\n");
    fprintf(fp, "\n");
    fprintf(fp, "#log4j.logger.participant=info\n");
    fprintf(fp, "#log4j.logger.rmw_client=info\n");
    fprintf(fp, "#log4j.logger.rmw_count=info\n");
    fprintf(fp, "#log4j.logger.rmw_publisher=info\n");
    fprintf(fp, "#log4j.logger.rmw_request=info\n");
    fprintf(fp, "#log4j.logger.rmw_response=info\n");
    fprintf(fp, "#log4j.logger.rmw_service=info\n");
    fprintf(fp, "#log4j.logger.rmw_service_names_and_types=info\n");
    fprintf(fp, "#log4j.logger.rmw_service_server_is_avaliable=info\n");
    fprintf(fp, "#log4j.logger.rmw_topic_names_and_types=info\n");
    fprintf(fp, "#log4j.logger.topic_manager=info\n");
    fprintf(fp, "#log4j.logger.fastrtps_discovery=info\n");
    fprintf(fp, "#log4j.logger.sharedmem_block_queue=info\n");
    fprintf(fp, "#log4j.logger.sharedmem_publisher_impl=info\n");
    fprintf(fp, "#log4j.logger.sharedmem_segment=info\n");
    fprintf(fp, "#log4j.logger.sharedmem_subscriber_impl=info\n");
    fprintf(fp, "#log4j.logger.sharedmem_transport=info\n");
    fprintf(fp, "#log4j.logger.sharedmem_util=info\n");
    fprintf(fp, "#log4j.logger.custom_client_info=info\n");
    fprintf(fp, "#log4j.logger.custom_service_info=info\n");
    fprintf(fp, "#log4j.logger.rmw_get_gid_for_publisher=info\n");
    fprintf(fp, "#log4j.logger.rmw_publish=info\n");

    fclose(fp);
  }

  char str[4096];

  snprintf(str, 4096, "%s", logprefix);
  setenv("logprefix", str, 1);

  snprintf(str, 4096, "%ld", (long)getpid());
  setenv("pid", str, 1);

  log4cxx::PropertyConfigurator::configure(path);
}

log4cxx::LoggerPtr get_logger(const char *name)
{
    static std::unordered_map<std::string, log4cxx::LoggerPtr>loggers; 

    auto it = loggers.find(name);
    if ( it == loggers.end())
    {
        log4cxx::LoggerPtr ptr = log4cxx::Logger::getLogger(name);//fail??
        loggers[name] = ptr;
        return ptr;
    } else {
        return it->second;
    }
}

}  // namespace utils
}  // namespace zoro
