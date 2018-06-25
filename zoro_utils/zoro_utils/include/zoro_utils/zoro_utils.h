#pragma once

#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <log4cxx/logger.h>
#include <log4cxx/logstring.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>

namespace zoro
{
namespace utils
{

/* start service */
bool start_service();

/* set parameters */
bool set_parameter(const std::string &key, bool val);
bool set_parameter(const std::string &key, int val);
bool set_parameter(const std::string &key, float val);
bool set_parameter(const std::string &key, std::string val);

/* get parameters */
bool get_parameter(const std::string &key, bool &val);
bool get_parameter(const std::string &key, int &val);
bool get_parameter(const std::string &key, double &val);
bool get_parameter(const std::string &key, std::string &val);

/* Logger C++ using log4cxx */

void zoro_init(const char * logprefix = NULL, const char * logconfig = NULL);
log4cxx::LoggerPtr get_logger(const char *name);

#define LOG(func, name, format, ...) {\
    log4cxx::LoggerPtr logger = zoro::utils::get_logger(name);\
    char log_str[4096];\
    snprintf(log_str, sizeof(log_str), format, ## __VA_ARGS__);\
    logger->func(log_str);\
}

#define LOG_DEBUG(name, format, ...) LOG(debug, name, format, ## __VA_ARGS__)
#define LOG_INFO(name, format, ...) LOG(info, name, format, ## __VA_ARGS__)
#define LOG_WARN(name, format, ...) LOG(warn, name, format, ## __VA_ARGS__)
#define LOG_ERROR(name, format, ...) LOG(error, name, format, ## __VA_ARGS__)
#define LOG_FATAL(name, format, ...) LOG(fatal, name, format, ## __VA_ARGS__)

}  // namespace utils
}  // namespace zoro
