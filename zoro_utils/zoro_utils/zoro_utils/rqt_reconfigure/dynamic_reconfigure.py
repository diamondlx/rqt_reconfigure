import os
import zoro_utils as zu

class client:
    def __init__(self, remote_name, timeout, config_callback=None):
        self.remote_name = remote_name
        self.timeout = timeout
        self.config_callback = config_callback

    def get_group_descriptions(self):
        description = zu.get_description(self.remote_name)
        print(description)
        names = []
        for k in description.keys():
            if (k[0:4] == 'name'):
                names.append(k[5:])
                print(k[0:4], k[5:])
        dic = {'groups': {}, 'state': True, 'name': 'Default', 'parent': 0, 'type': '', 'id': 0}
        dic['parameters'] = []
        for name in names:
            parameter = {}
            parameter['edit_method'] = ''
            parameter['name'] = name
            parameter['min'] = description['min.' + name]
            parameter['max'] = description['max.' + name]
            #parameter['value'] = description['value.' + name]
            parameter['level'] = description['lev.' + name]
            parameter['default'] = description['default.' + name]
            parameter['type'] = description['type.' + name]
            parameter['description'] = description['des.' + name]

            dic['parameters'].append(parameter)
        print(dic)
        return dic

    def update_configuration(self, configs_tobe_updated):
        print('666666666666update_configuration',configs_tobe_updated)
        in_dict = {}
        for k in configs_tobe_updated.keys():
            in_dict['value.' + k] = configs_tobe_updated[k]
        values = zu.update_parameters(self.remote_name, in_dict)
        print('values' , values)

        names = []
        for k in values.keys():
            if (k[0:5] == 'value'):
                names.append(k[6:])
                #print(k[0:4], k[5:])
        dic = {}
        for name in names:
            dic[name] = values['value.' + name]
        dic['groups'] = dic.copy()
        dic['groups']['groups'] = ''
        print('before callback')
        self.config_callback(dic)
        #return {'int_param': 50, 'double_param': 0.55, 'str_param': 'Hello World', 'bool_param': True, 'groups': {'int_param': 50, 'double_param': 0.55, 'parent': 0, 'bool_param': True, 'groups': {}, 'id': 0, 'size': 1, 'str_param': 'Hello World', 'name': 'Default', 'parameters': {}, 'state': True, 'type': ''}, 'size': 1}
    '''
    def get_configuration(self, configs_tobe_updated):
        print('get_configuration',configs_tobe_updated)
        return {'int_param': 50, 'double_param': 0.55, 'str_param': 'Hello World', 'bool_param': True, 'groups': {'int_param': 50, 'double_param': 0.55, 'parent': 0, 'bool_param': True, 'groups': {}, 'id': 0, 'size': 1, 'str_param': 'Hello World', 'name': 'Default', 'parameters': {}, 'state': True, 'type': ''}, 'size': 1}
    '''

    def close(self):
        return
    def update(self):
        return 1

def find_reconfigure_services():
    cmd = 'ros2 service list'
    ret = os.popen(cmd, 'r', 1).read().split()
    servers = []
    for s in ret:
        if '/describe_parameters' == s[-20:] and s[:-20] != '/get_parameters_try_client':
            servers.append(s[:-20])
    return servers


def DynamicReconfigureParameterException():
    return 0

def DynamicReconfigureCallbackException():
    return 0

if __name__ == '__main__':
    print(find_reconfigure_services())