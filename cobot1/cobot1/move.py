import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    import sys
    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}", flush=True)
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    pos1 = posx([350.0, 34.5, 300.0, 45.0, 180.0, 45.0])


    while rclpy.ok():

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)

        print(f"Moving to task position: {pos1}")
        movel(pos1, vel=VELOCITY, acc=ACC)

    rclpy.shutdown()

if __name__ == "__main__":
    main()