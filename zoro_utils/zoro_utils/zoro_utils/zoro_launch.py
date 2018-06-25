from launch.exit_handler import default_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path

import yaml
import rclpy

import zoro_utils.impl.zoro_launch_xml_parse as zoro_launch_xml_parse
import zoro_utils

def parse_argv(argv):
    res={}
    for args in argv:
      k,v = args.split(':=')
      print ("get args:",args,k,":=",v)
      res[k] = v

    return res

def show_usage():
    print ('<pkg=pkg>, <exec=exec> or <launch_file=l.launch>')
    exit(0)

def set_node_args(node_args, node_attr):
  zoro_utils.set_parameters(node_args)

def load_yaml(yaml_file):
  with open(yaml_file, 'r') as stream:
    try:
        return yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

  return dict()

def launch(launch_descriptor, argv):
    if not argv:
      show_usage()

    rclpy.init(args=None)

    arg_dict = parse_argv(argv)

    ld = launch_descriptor
    if 'launch_file' in arg_dict:
      launch_file = arg_dict['launch_file']
      launch_info=zoro_launch_xml_parse.LaunchXmlParser(launch_file, arg_dict)

      for g in launch_info.groups:
        for node in g.nodes:
          package_name = node.attr['pkg']
          executable_name = node.attr['type']

          set_node_args(node.params, node.attr)
          if len(node.ros_params):
            print ("ros param:",node.ros_params['file'])
            if node.ros_params['command'] == 'load' and node.ros_params['file'].endswith('yaml'):
              yaml_args = load_yaml(node.ros_params['file'])
              set_node_args(yaml_args, node.attr)

          ld.add_process(
              cmd=[get_executable_path(package_name=package_name, executable_name=executable_name)],
              name=executable_name,
              exit_handler=restart_exit_handler,
          )

    else:
      show_usage()

    rclpy.shutdown()

    return ld
