import  xml.dom.minidom
import re


def place(v, args):
  res = re.findall('.*\$\(arg ([^)]*)\).*', v)
  if len(res)>1:
    return [ args[i] if i in args else i for i in res ]
  elif len(res)==1:
    return args[res[0]] if res[0] in args else res[0]
  return v

def parse_params(xml, node_name, replace_args): 
  args_list = []
  for child in xml.childNodes:
    if(child.nodeName == node_name):
      args = {} 
      for k,v in child.attributes.items():
        args[k] = place(v, replace_args)
      args_list.append(args)
  return args_list

class Node:
  def __init__(self, xml, args):
    self.args = args
    self.params = {}
    self.ros_params = {}

    self.attr = {} 
    for k,v in xml.attributes.items():
      self.attr[k] = place(v, args)

    param_args_list =  parse_params(xml, 'param', args)
    self.params= {}
    for args in param_args_list:
      self.params[args["name"]] = args["value"] 

    self.ros_params = parse_params(xml, 'rosparam', self.args)
    if( len(self.ros_params) ):
      self.ros_params = self.ros_params[0]


class Group:
  def __init__(self, group_xml, args):
    self.args = args

    self.nodes = []
    nodes_xml = group_xml.getElementsByTagName('node')
    for node_xml in nodes_xml:
      self.nodes.append(Node(node_xml, args))        

    
class LaunchXmlParser:
  def __init__(self, xml_file,args):
    dom = xml.dom.minidom.parse(xml_file)
    root = dom.documentElement

    self.args = {}
    self.machine_args = {}
    self.groups=[]
    self.params={}

    self.parse_args(root)
    for k,v in args.items():
      print ("replace:",k)
      if k in self.args:
        print ("set value:",k,v,"default:",self.args[k])
      self.args[k] = v

    self.parse_machine(root)
    self.parse_groups(root)
    self.parse_golbal_param(root)
  
  def parse_machine(self, xml):
    self.machine_args = parse_params(xml, 'machine', self.args)
    if len(self.machine_args):
        self.machine_args = self.machine_args[0]

  def parse_groups(self, xml):
    groups_xml = xml.getElementsByTagName('group')
    for group_xml in groups_xml:
      self.groups.append(Group(group_xml, self.args))

  def parse_args(self, xml):
    args_list = parse_params(xml, 'arg', self.args)
    for args in args_list:
      self.args[ args['name'] ] = args['default']

  def parse_golbal_param(self,xml):
    param_args_list = parse_params(xml, 'param', self.args)
    for arg in param_args_list:
      if 'name' not in arg:
        continue
      if 'default' in arg:
        self.params[arg['name']] = arg['default']
      if 'value' in arg:
        self.params[arg['name']] = arg['value']


if __name__ == '__main__':
  l=LaunchXml("./talker.launch")
  print ("args:")
  print (l.args)

  print ("macine:",l.machine_args)
  print ("golbal param:",l.params)

  for g in l.groups:
    print ("node:")
    for node in g.nodes:
      print (node.args)
      print ("param:")
      print (node.param_args)
      print ("ros param")
      print (node.ros_param_args)
  
#  <machine name="$(arg server)" address="$(arg server)" ssh-port="$(arg port)" user="$(arg user)" env-loader="$(arg env_loader)"/>
#print bb,len(bb)
#b= bb[2]
#print b.nodeName

#bb = root.getElementsByTagName('item')
#b= bb[1]
#print b.nodeName
