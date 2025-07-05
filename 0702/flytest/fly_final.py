from FlyControl import *

import threading
from threading import Thread

fly = FlyControl()

fly.init_Gpio()

fly_mode = 1

if fly_mode == 1:
    fly.set_fly_mode(1)
    fly.init()

    port_thread = Thread(target=fly.PortCom)

    position_send_thread = Thread(target=fly.position_send)

    deal_thread = Thread(target=fly.deal, args=(fly_mode,))

    fly.key_fly()

    port_thread.start()
    print("串口线程已开启")
    position_send_thread.start()
    print("t265线程已开启")
    deal_thread.start()
    print("处理线程已开启")

elif fly_mode == 2:
    fly.set_fly_mode(2)
    file = FileHandler("data.txt")
    file.open_read()
    file.read_data()
    dict_position_info = file.return_dict_info()
    file.file_close()

    # visual_deal = VisualDeal()
    # fly.gpio.oled.show_info(4, "About to recognize ")
    time.sleep(5)
    num_info = visual_deal.visual_deal_mode_2()

#     fly.state_info_send(2, 'A1', num_info)
    str_num_info = "num_info: " + str(num_info)
    fly.gpio.oled.show_info(3, str_num_info)
    str_position_info = dict_position_info[num_info]

    router_list = Containers.dict_router[str_position_info]
    break_index = Containers.dict_road_break_index[str_position_info]
    turn_duoji = Containers.dict_turn_duoji[str_position_info]
    turn_laser = Containers.dict_turn_laser[str_position_info]

    fly.set_router_list(router_list)
    fly.set_router_mode2_break_index(break_index)
    fly.set_turn_duoji(turn_duoji)
    fly.set_turn_laser(turn_laser)

    fly.init()
    
    # fly.state_info_send(2, 'A1', num_info)

    port_thread = Thread(target=fly.PortCom)

    position_send_thread = Thread(target=fly.position_send)

    deal_thread = Thread(target=fly.deal, args=(fly_mode,))

    fly.key_fly()

    port_thread.start()
    print("串口线程已开启")
    position_send_thread.start()
    print("t265线程已开启")
    deal_thread.start()
    print("处理线程已开启")



