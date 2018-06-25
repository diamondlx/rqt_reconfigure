import rclpy
import zoro_utils as zu

def main(args=None):
#    rclpy.init(args=args)
    zu.zoro_init()
    zu.start_service()


#    rclpy.shutdown()


if __name__ == '__main__':
    main()

