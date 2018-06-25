import rclpy
import zoro_utils as zu

def main(args=None):
    rclpy.init(args=args)

    while True:
        dic=zu.get_parameters(["aa","c","dd","bbb","e"])

        print(dic.keys())
    #print(dic.values())

    rclpy.shutdown()


if __name__ == '__main__':
    main()

