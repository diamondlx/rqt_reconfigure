#include <Python.h>
#include "rclcpp/rclcpp.hpp"
#include "rcl/rcl.h"
#include <log4cxx/logger.h>
#include <log4cxx/logstring.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include "test_client.cpp"
#include "server_py.hpp"
#include "rclcpp/parameter.hpp"

typedef struct
{
    PyObject_HEAD
    log4cxx::LoggerPtr logger;
}PyLogObject;

#define LOG_XXX(XXX, self, args)\
  static PyObject *LOG_##XXX(PyObject *self, PyObject *args)\
{\
    char *str = NULL;\
    PyArg_ParseTuple(args, "s", &str);\
    LOG4CXX_##XXX( ((PyLogObject*)(self))->logger, str);\
    Py_RETURN_NONE;\
}

#define DEF_METHODS(xxx, XXX)\
{\
#xxx, LOG_##XXX, METH_VARARGS,\
"log level "#xxx"."\
}

LOG_XXX(DEBUG, self, args)
LOG_XXX(INFO,  self, args)
LOG_XXX(WARN,  self, args)
LOG_XXX(ERROR, self, args)
LOG_XXX(FATAL, self, args)

static PyObject *PyLogObject_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
  Py_RETURN_NONE;
}

static PyObject *PyLogObject_init(PyLogObject *self, PyObject *args, PyObject *kwds)
{
  Py_RETURN_NONE;
}

static PyMethodDef PyLogObject_methods[] = {
  DEF_METHODS(debug, DEBUG),
  DEF_METHODS(info,   INFO),
  DEF_METHODS(warn,   WARN),
  DEF_METHODS(fatal, FATAL),
  DEF_METHODS(error, ERROR),
  {NULL, NULL, 0, NULL}
};

/* PyLogObject Type  */
static PyTypeObject PyLogObjectType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "zoro_utils.logger",
    sizeof(PyLogObject),
    0,                         /* tp_itemsize */
    0,                         /* tp_dealloc */
    0,                         /* tp_print */
    0,                         /* tp_getattr */
    0,                         /* tp_setattr */
    0,                         /* tp_compare */
    0,                         /* tp_repr */
    0,                         /* tp_as_number */
    0,                         /* tp_as_sequence */
    0,                         /* tp_as_mapping */
    0,                         /* tp_hash */
    0,                         /* tp_call */
    0,                         /* tp_str */
    0,                         /* tp_getattro */
    0,                         /* tp_setattro */
    0,                         /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT |
    Py_TPFLAGS_BASETYPE,       /* tp_flags */
    "zoro_utils logger ",          /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    0,                         /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    PyLogObject_methods,       /* tp_methods */
    0,                         /* tp_members */
    0,                         /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)PyLogObject_init,/* tp_init */
    0,                         /* tp_alloc */
    PyLogObject_new,          /* tp_new */
};

/* internal function for get/set parameters  */
static std::shared_ptr<rclcpp::SyncParametersClient> _init_client(std::string node_name )
{

  if(!rcl_ok())
    printf("rcl is not ok\n");

  auto node = rclcpp::Node::make_shared(node_name);
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "zoro_set_and_get_parameters" );
  while (!parameters_client->wait_for_service(std::chrono::nanoseconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
  }
  return parameters_client;
}

static rclcpp::parameter::ParameterVariant _make_param(PyObject* dict, PyObject* key)
{
  std::string key_string= PyUnicode_AsUTF8(key);
  PyObject* value=PyDict_GetItem(dict, key);

  if(PyUnicode_Check(value)){
      return rclcpp::parameter::ParameterVariant(key_string, PyUnicode_AsUTF8(value));
  }

  if(PyBool_Check(value)) {
    PyObject_Print(value, stdout, Py_PRINT_RAW);
    if (value==Py_True) {
      return rclcpp::parameter::ParameterVariant(key_string, true);
    } else {
      return rclcpp::parameter::ParameterVariant(key_string, false);
    }
  }

  if( PyLong_Check(value)) {
    return rclcpp::parameter::ParameterVariant(key_string, PyLong_AsLong(value));
  }

  if( PyFloat_Check(value)) {
    return rclcpp::parameter::ParameterVariant(key_string, PyFloat_AsDouble(value));
  }
  return rclcpp::parameter::ParameterVariant("", 0);
}

static void _set_dict(PyObject* dict, rclcpp::parameter::ParameterVariant parameter)
{
  switch(parameter.get_type()){
    case rclcpp::parameter::ParameterType::PARAMETER_BOOL:
      if(parameter.as_bool()) {
        PyDict_SetItemString(dict, parameter.get_name().data(), Py_True);
      } else {
        PyDict_SetItemString(dict, parameter.get_name().data(), Py_False);
      }
      return;
    case rclcpp::parameter::ParameterType::PARAMETER_INTEGER:
      PyDict_SetItemString(dict, parameter.get_name().data(), PyLong_FromLong(parameter.as_int()));
      return;
    case rclcpp::parameter::ParameterType::PARAMETER_DOUBLE:
      PyDict_SetItemString(dict, parameter.get_name().data(), PyFloat_FromDouble(parameter.as_double()));
      return;
    case rclcpp::parameter::ParameterType::PARAMETER_STRING:
      PyDict_SetItemString(dict, parameter.get_name().data(), PyUnicode_FromString(parameter.as_string().data()));
      return;
    default:
      ;
  }
}

/* log4cxx  */
static PyObject *zoro_init(PyObject *Py_UNUSED(self), PyObject *args, PyObject *kwargs)
{
  static char *kw[] = {(char*)("logprefix"), (char*)("logconfig"), nullptr};

  char *logprefix = NULL;
  char *logconfig = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|ss", kw, &logprefix, &logconfig))
  {
      return NULL;
  }


  char *prefix;

  if (logprefix == NULL)
  {
     logprefix = (char*)("zoro");
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
    {
        fprintf(stderr, "Error: %s open error\n", path);
        fprintf(stderr, "       %s\n", strerror(errno));
        return NULL;
    }

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

  Py_RETURN_NONE;
}

static PyObject *get_logger(PyObject *self, PyObject *args)
{
    using namespace log4cxx;
    char *cat = (char*)("app");
    PyLogObject *ob;

    static std::unordered_map<std::string, PyLogObject*> loggers;

    if (!PyArg_ParseTuple(args, "|s", &cat))
    {
        return NULL;
    }

    auto it = loggers.find(cat);
    if ( it == loggers.end())
    {
        ob =(PyLogObject*)( PyLogObjectType.tp_alloc(&PyLogObjectType, 0));
        if ( ob == NULL )
        {
            return NULL;
        }
        ob->logger = Logger::getLogger(cat);
        loggers[cat] = ob;
    } else {
        ob = it->second;
    }
    Py_XINCREF(ob);
    return (PyObject*)(ob);
}

static PyObject *start_service(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  printf("start_service!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  int argc = 1;
  char *argv[] = {(char*)("parameters"), nullptr};
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("zoro_set_and_get_parameters");
  auto parameter_service = std::make_shared<rclcpp::ParameterService>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  Py_RETURN_NONE;
}

static PyObject *get_parameters(PyObject * Py_UNUSED(self), PyObject *args)
{
  //rcl_shutdown();
  static auto parameters_client=_init_client("get_parameters_client");
  PyObject* list;
  PyArg_ParseTuple(args, "O", &list);
  Py_ssize_t i=0;
  std::vector<std::string>  parameter_names;
  for(i=0;i<PyList_GET_SIZE(list);++i){
      PyObject* key=PyList_GetItem(list,i);
      if(PyUnicode_Check(key)){
        std::string key_string= PyUnicode_AsUTF8(key);
        if( parameters_client->has_parameter(key_string)){
            parameter_names.push_back(key_string);
        }
      }
  }
  PyObject* dict=PyDict_New();
  for (auto & parameter : parameters_client->get_parameters(parameter_names)) {
    _set_dict(dict, parameter);
  }
  //rclcpp::shutdown();
  return dict;
}

static  PyObject *set_parameters(PyObject * Py_UNUSED(self), PyObject *args)
{
  //rcl_shutdown();
  auto parameters_client=_init_client("set_parameters_client");
  PyObject* dict;
  PyArg_ParseTuple(args, "O", &dict);
  PyObject* l=PyDict_Keys(dict);
  Py_ssize_t i=0;
  std::vector<rclcpp::parameter::ParameterVariant> para;
  for(i=0;i<PyList_GET_SIZE(l);++i){
      PyObject* key=PyList_GetItem(l,i);
      if(PyUnicode_Check(key)){
          para.push_back(_make_param(dict, key));
      }
  }
  auto set_parameters_results = parameters_client->set_parameters(para);
  for (auto & result : set_parameters_results) {
    if (!result.successful) {
      printf("Failed to set parameters\n");
      //RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result.reason.c_str())
    }
  }
  //rclcpp::shutdown();
  Py_RETURN_NONE;
}


static PyObject *hello(PyObject * Py_UNUSED(self), PyObject * Py_UNUSED(args))
{
  printf("hello hello!!!!!!!!!!!!!\n");
  //rclcpp::init(argc, argv);
  //auto node = rclcpp::Node::make_shared("zoro_set_and_get_parameters");
  //auto parameter_service = std::make_shared<rclcpp::ParameterService>(node);
  //rclcpp::spin(node);
  //rclcpp::shutdown();
  Py_RETURN_NONE;
}

static PyObject *client_init(PyObject * Py_UNUSED(self), PyObject * args)
{

  char* remote_name;
  if (!PyArg_ParseTuple(args, "z", &remote_name)) {
    return NULL;
  }
  auto node = rclcpp::Node::make_shared("get_parameters_try_client");
  std:: shared_ptr<rqt_reconfigure::Client> test_client = rqt_reconfigure::client_map.get_client(node, remote_name);
  Py_RETURN_NONE;
}

static PyObject *get_description(PyObject * Py_UNUSED(self), PyObject * args)
{

  char* remote_name;
  if (!PyArg_ParseTuple(args, "z", &remote_name)) {
    return NULL;
  }

  auto node = rclcpp::Node::make_shared("get_parameters_try_client");
  std:: shared_ptr<rqt_reconfigure::Client> test_client = rqt_reconfigure::client_map.get_client(node, remote_name);
  //rqt_reconfigure::Client test_client = rqt_reconfigure::Client(node, remote_name);
  std::vector<rclcpp::parameter::ParameterVariant> descritions = test_client->get_description();
  PyObject* dict=PyDict_New();
  for (auto & parameter : descritions) {
    _set_dict(dict, parameter);
  }
  //rclcpp::shutdown();
  return dict;
}
static PyObject *get_values(PyObject * Py_UNUSED(self), PyObject * args)
{
  char* remote_name;
  if (!PyArg_ParseTuple(args, "z", &remote_name)) {
    return NULL;
  }
  printf("remote_name = %s\n", remote_name);
  printf("before node_init\n");
  auto node = rclcpp::Node::make_shared("get_parameters_try_client");
  printf("after node_init\n");
  printf("before client_init\n");
  std:: shared_ptr<rqt_reconfigure::Client> test_client = rqt_reconfigure::client_map.get_client(node, remote_name);
  //rqt_reconfigure::Client test_client = rqt_reconfigure::Client(node, remote_name);
  //rqt_reconfigure::Client_map::check_map.begin();
  //int temp = rqt_reconfigure::client_map.get_client(node, remote_name);
  printf("after client_init\n");
  std::vector<rclcpp::parameter::ParameterVariant> values = test_client->get_values();
  printf("after get_description\n");
  PyObject* dict=PyDict_New();
  for (auto & parameter : values) {
    _set_dict(dict, parameter);
  }
  //rclcpp::shutdown();
  return dict;
}

static  PyObject *update_parameters(PyObject * Py_UNUSED(self), PyObject *args)
{
  //rcl_shutdown();
  //auto parameters_client=_init_client("set_parameters_client");
  printf("update_parameters\n");
  printf("0\n");
  PyObject* dict;
  char* remote_name;
  printf("1\n");
  PyArg_ParseTuple(args, "zO", &remote_name, &dict);
  printf("2\n");
  //printf("remote_name = %s\n", remote_name);
  PyObject* l=PyDict_Keys(dict);
  printf("3\n");
  Py_ssize_t i=0;
  std::vector<rclcpp::parameter::ParameterVariant> para;
  printf("4\n");
  for(i=0;i<PyList_GET_SIZE(l);++i){
      printf("i=%d\n",i);
      PyObject* key=PyList_GetItem(l,i);
      if(PyUnicode_Check(key)){
          para.push_back(_make_param(dict, key));
      }
  }
  printf("5\n");
  auto node = rclcpp::Node::make_shared("get_parameters_try_client");
  std:: shared_ptr<rqt_reconfigure::Client> test_client = rqt_reconfigure::client_map.get_client(node, remote_name);
  //rqt_reconfigure::Client test_client = rqt_reconfigure::Client(node, remote_name);
  test_client->update_params(para);

  std::vector<rclcpp::parameter::ParameterVariant> values = test_client->get_values();

  PyObject* ans=PyDict_New();
  for (auto & parameter : values) {
    _set_dict(ans, parameter);
  }

  return ans;

}

static rclcpp::parameter::ParameterVariant _make_param_add_name(PyObject* dict, PyObject* key, std::string name, std::string short_form)
{
  std::string key_string= short_form + '.' + name;
  PyObject* value=PyDict_GetItem(dict, key);

  if(PyUnicode_Check(value)){
      return rclcpp::parameter::ParameterVariant(key_string, PyUnicode_AsUTF8(value));
  }

  if(PyBool_Check(value)) {
    //PyObject_Print(value, stdout, Py_PRINT_RAW);
    if (value==Py_True) {
      return rclcpp::parameter::ParameterVariant(key_string, true);
    } else {
      return rclcpp::parameter::ParameterVariant(key_string, false);
    }
  }

  if( PyLong_Check(value)) {
    return rclcpp::parameter::ParameterVariant(key_string, PyLong_AsLong(value));
  }

  if( PyFloat_Check(value)) {
    return rclcpp::parameter::ParameterVariant(key_string, PyFloat_AsDouble(value));
  }
  return rclcpp::parameter::ParameterVariant("", 0);
}

static  PyObject *params_service_init(PyObject * Py_UNUSED(self), PyObject *args)
{
  char* service_name;
  PyObject *python_handle;
  PyObject *cfg_list;
  PyArg_ParseTuple(args, "zOO", &service_name, &python_handle, &cfg_list);

  std::vector<rclcpp::parameter::ParameterVariant> cfg_all;

  for(int i = 0; i < PyList_GET_SIZE(cfg_list); ++i) {
    PyObject* param = PyList_GetItem(cfg_list, i);
    PyObject* name = PyDict_GetItem(param, Py_BuildValue("s","name"));
    std::string name_c = PyUnicode_AsUTF8(name);
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","name"), name_c, "name"));
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","description"), name_c, "des"));
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","type"), name_c, "type"));
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","level"), name_c, "lev"));
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","min"), name_c, "min"));
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","max"), name_c, "max"));
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","default"), name_c, "default"));
    cfg_all.push_back(_make_param_add_name(param, Py_BuildValue("s","default"), name_c, "value"));
  }
  rqt_reconfigure::Server_py(service_name, python_handle, cfg_all);
  Py_RETURN_NONE;
}



static PyMethodDef  module_methods[] =
{
  {
    "zoro_init", (PyCFunction)(zoro_init), METH_VARARGS | METH_KEYWORDS,
    "Set the level of a logger."
  },
  {
    "get_logger", get_logger, METH_VARARGS,
    "get logger by specify the catalog"
  },
  {
    "start_service", start_service, METH_NOARGS,
    "Initialize the logging system."
  },
  {
    "set_parameters", set_parameters, METH_VARARGS,
    "Set the level of a logger."
  },
  {
    "get_parameters", get_parameters, METH_VARARGS,
    "Set the level of a logger."
  },
  {
    "hello", hello, METH_NOARGS,
    "hello_description"
  },
  {
    "client_init", client_init, METH_VARARGS,
    "client_init_description"
  },
  {
    "get_description", get_description, METH_VARARGS,
    "get_description good"
  },
  {
    "get_values", get_values, METH_VARARGS,
    "get_values good"
  },
  {
    "update_parameters", update_parameters, METH_VARARGS,
    "update_parameters good"
  },
  {
    "params_service_init", params_service_init, METH_VARARGS,
    "params_service_init"
  },
  {NULL, NULL, 0, NULL}  /* sentinel */
};

PyDoc_STRVAR(zoro_utils__doc__,
  "zoro utils module: logger, get/set_parameters");

/// Define the Python module
static struct PyModuleDef zoro_utils_module =
{
  PyModuleDef_HEAD_INIT,
  "zoro_utils",
  zoro_utils__doc__,
  -1,   /* -1 means that the module keeps state in global variables */
  module_methods,
  NULL,
  NULL,
  NULL,
  NULL
};

// Init function of this module
PyMODINIT_FUNC PyInit__zoro_utils_(void)
{

  PyObject *m;

  if ( PyType_Ready(&PyLogObjectType) <0 )
  {
      return NULL;
  }

  m = PyModule_Create(&zoro_utils_module);

  if (m == NULL)
  {
     return NULL;
  }

  Py_INCREF(&PyLogObjectType);
  PyModule_AddObject(m, "logger", (PyObject*)(&PyLogObjectType));

  return m;
}
