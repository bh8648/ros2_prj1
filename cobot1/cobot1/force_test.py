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
            task_compliance_ctrl,
            set_desired_force,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_BASE,
            get_tool_force,
            get_current_posx,
            set_digital_output,
            wait,
            DR_TOOL
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    pos = posx([496.06, 93.46, 96.92, 20.75, 179.00, 19.09])
    JReady = posj([0, 0, 90, 0, 90, 0])

    def grap_open():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(1.0)

    def grap_close():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(1.0)
    s = 0
    while rclpy.ok():

        movel([0, 0, 0, 0, 0, 90], VELOCITY, ACC, ref=DR_TOOL)

        grap_close()
        wait(2.0)
        print(f"Moving to joint position: {JReady}")


        wait(0.5)
        task_compliance_ctrl([3000, 3000, 500, 300, 300, 300])
        wait(0.5)  # ← 이거 추가
        set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.5)

        while True:
            force_ext = get_tool_force(DR_BASE)
            print(f"#force_ext = {force_ext}")

    
            if force_ext[2] >= 5:
                break
            

    

        release_force()
        time.sleep(0.5)
        
        print("Starting release_compliance_ctrl")      
        release_compliance_ctrl()
        time.sleep(0.5)

        a = get_current_posx()[0]
        a[2] += 5
        movel(a, 80, 100)
        grap_open()
        wait(2.0)
        a[2] -= 60
        movel(a, 80, 100)
        wait(0.5)
        grap_close()
        wait(2.0)
        a[2] += 200.00
        movel(a, 80, 100)

        if s == 1:
            break
        s += 1
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    main()