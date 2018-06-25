import zoro_utils as zu
import rclpy

def wrap_init(remote_name):
	zu.client_init(remote_name)

def wrap_description(remote_name):
	description = zu.get_description(remote_name)
	print(description)
	names = []
	for k in description.keys():
		if (k[0:4] == 'name'):
			names.append(k[5:])
			print(k[0:4], k[5:])
	dic = {'parameters' : []}
	for name in names:
		parameter = {}
		parameter['edit_method'] = ''
		parameter['name'] = name
		parameter['min'] = description['min.' + name]
		parameter['max'] = description['max.' + name]
		parameter['level'] = description['lev.' + name]
		parameter['default'] = description['default.' + name]
		parameter['type'] = description['type.' + name]
		parameter['description'] = description['des.' + name]

		dic['parameters'].append(parameter)


	print(dic)
	return dic

def wrap_values(remote_name):
	values = zu.get_values(remote_name)
	print(values)
	names = []
	for k in values.keys():
		if (k[0:5] == 'value'):
			names.append(k[6:])
			#print(k[0:4], k[5:])
	dic = {'parameters' : []}
	for name in names:
		parameter = {}
		#parameter['edit_method'] = ''
		parameter['name'] = name
		#parameter['min'] = values['min.' + name]
		#parameter['max'] = values['max.' + name]
		#parameter['level'] = values['lev.' + name]
		#parameter['default'] = values['default.' + name]
		parameter['type'] = values['type.' + name]
		#parameter['description'] = values['des.' + name]
		parameter['value'] = values['value.' + name]
		dic['parameters'].append(parameter)


	print(dic)
	return dic

def wrap_update(remote_name, dict):
	in_dict = {}
	for k in dict.keys():
		in_dict['value.' + k] = dict[k]
	values = zu.update_parameters(remote_name, in_dict)
	names = []
	for k in values.keys():
		if (k[0:5] == 'value'):
			names.append(k[6:])
			#print(k[0:4], k[5:])
	dic = {'parameters' : []}
	for name in names:
		parameter = {}
		#parameter['edit_method'] = ''
		parameter['name'] = name
		#parameter['min'] = values['min.' + name]
		#parameter['max'] = values['max.' + name]
		#parameter['level'] = values['lev.' + name]
		#parameter['default'] = values['default.' + name]
		parameter['type'] = values['type.' + name]
		#parameter['description'] = values['des.' + name]
		parameter['value'] = values['value.' + name]
		dic['parameters'].append(parameter)

	return dic





def main(args=None):
    rclpy.init(args=args)

    #description = wrap_description('get_parameters3')
    #print(description)
    #print('python python python ')
    wrap_init('get_parameters3')
    description = wrap_description('get_parameters3')
    print(description)

    values = wrap_values('get_parameters3')
    print('in python get values:', values)

    print('in python return of update:', wrap_update('get_parameters3', {'str_param' : 'changed!!' , 'int_param' : 7}))

    #zu.try_client()
    #rclpy.spin()
    while(1):
    	a = 1
    rclpy.shutdown()


if __name__ == '__main__':
    main()

