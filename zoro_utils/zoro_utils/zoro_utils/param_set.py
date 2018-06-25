import rclpy
import zoro_utils as zu

def main(args=None):
    rclpy.init(args=args)
    zu.set_parameters({"aa":10,"bbb":2.56,"c":"yifan"*1024,"dd":True,"e":None})
    zu.set_parameters({"abc":10})

    print("set ok")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

