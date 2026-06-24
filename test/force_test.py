import time
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            # movej,
            movel,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])
    JReady = posj([0, 0, 90, 0, 90, 0])

    while rclpy.ok():
        print(f"Moving to joint position: {JReady}")
        # movej(JReady, vel=VELOCITY, acc=ACC)

        print(f"Moving to task position: {pos}")
        movel(pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        print("Starting task_compliance_ctrl")
        task_compliance_ctrl(stx=[3000, 3000, 500, 300, 300, 400])
        time.sleep(0.5)

        print("Starting set_desired_force")
        set_desired_force(
            fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL
        )

        # 외력이 0 이상 5 이하이면 0
        # 외력이 5 초과이면 -1
        while not check_force_condition(DR_AXIS_Z, max=5):
            print("Waiting for an external force greater than 5 ")
            time.sleep(0.5)
            pass

        print("Starting release_force")
        release_force()
        time.sleep(0.5)

        print("Starting release_compliance_ctrl")
        release_compliance_ctrl()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
