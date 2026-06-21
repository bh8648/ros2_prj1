import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL






def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    try:
        # from DRCF import get_force_control_state
        import time
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movel,
            set_digital_output,
            wait,
            release_force,
            release_compliance_ctrl,
            task_compliance_ctrl,
            set_desired_force,
            get_tool_force,
            get_current_posx,
            ON, 
            OFF, 
            DR_FC_MOD_REL,
            DR_BASE,
            check_force_condition,
            DR_AXIS_Z,

            
            
        
            

        )

        from DR_common2 import posx, posj


    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    #JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    #pos1 = posx([350.0, 34.5, 300.0, 45.0, 180.0, 45.0])


    

    while rclpy.spin_once():

        System_first_block = posx([249.49, -150.54, 340.00, 163.97, -179.72, 165.02])
    
        System_second_block = posx([357.89, 152.21, 340.00, 23.53, -179.61, 24.84])
        def grap_open():
            set_digital_output(2, OFF)
            set_digital_output(1, ON)
            wait(1.0)

        def grap_close():
            set_digital_output(2, ON)
            set_digital_output(1, OFF)
            wait(1.0)

        time.sleep(0.5)
        grap_close()
        pos = System_first_block
        pos2 = System_second_block
        cnt = 1

        while cnt < 10:

            p1 = System_first_block
            p2 = System_second_block
            movel(pos, 80, 100)

            time.sleep(0.5)
            grap_close()
            time.sleep(0.5)
            task_compliance_ctrl([3000, 3000, 500, 300, 300, 300])
            time.sleep(0.5)  # ← 이거 추가
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            # 이걸로 교체
            

            # force_ext = get_tool_force(DR_BASE)
            
  
            while True:
                force_ext = get_tool_force(DR_BASE)
                print(f"force_ext = {force_ext}")

        
                if force_ext[2] >= 15:
                    break
                
            release_force()
            wait(0.5)
            release_compliance_ctrl()
            wait(0.5)

            a = get_current_posx()[0]
            a[2] += 3
            movel(a, 80, 100)
            grap_open()
            a[2] -= 15
            movel(a, 80, 100)
            grap_close()
            a[2] = 350.00
            movel(a, 80, 100)
            
            # second 블록으로 이동해서 내려놓기
            movel(pos2, 80, 100)
            
            task_compliance_ctrl([3000, 3000, 500, 100, 100, 100])
            time.sleep(0.5)  # ← 이거 추가
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

  
            while True:
                check_force_condition()
                force_ext = get_tool_force(DR_BASE)
                print(f"force_ext = {force_ext}")

        
                if force_ext[2] >= 15:
                    break
                
            release_force()
            wait(0.5)
            release_compliance_ctrl()
            wait(0.5)
            grap_open()
            movel(pos2, 80, 100)


            a = cnt // 3 * 50
            b = cnt % 3 * -50
            pos = posx(p1[0] + a, p1[1] + b, p1[2], p1[3], p1[4], p1[5])
            pos2 = posx(p2[0] + a, p2[1] + b, p2[2], p2[3], p2[4], p2[5])
            cnt += 1
            time.sleep(0.5)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()