#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import Ptp, Lin, Robot, from_euler, Sequence
from prbt_hardware_support.srv import WriteModbusRegister
from prbt_hardware_support.msg import ModbusRegisterBlock, ModbusMsgInStamped
from smart_factory.msg import AgvToSmfStatus, SmfToAgvStatus
from smart_factory.srv import GetBoxPenCenter


# 位置常量（因涉及到实际机械位置，因此不要修改）
START_POSE = [
    math.radians(45),
    0,
    math.radians(-90),
    0,
    math.radians(-90),
    math.radians(0),
]  # 起始关节角度
ROBOT_HANDOUT_MIDDLE_WP = [1.58, -0.25, -1.8, 0, -1.6, 0.8]  # 出料中间路径点
GRIPPER_ORIENTATION = from_euler(0, math.radians(180), math.radians(45))  # 夹爪方向
SUCKER_INFEED_ORIENTATION = from_euler(0, math.radians(180), math.radians(134.5))
SUCKER_OUTFEED_ORIENTATION = from_euler(0, math.radians(180), math.radians(-45))
SAFETY_HEIGHT = 0.12
STOCK_Z_UP = 0.13
STOCK_PEN_X = 0.275
STOCK_PEN_Z_DOWN = 0.09
PLATE_PEN_Z_DOWN = 0.091
STOCK_BOX_X = 0.552
STOCK_BOX_Z_DOWN = 0.07
PLATE_BOX_Z_DOWN = 0.085


# 速度常量
PTP_SCALE = 0.2  # 点到点移动速度
LIN_SCALE = 0.1  # 直线移动速度
PNP_SCALE = 0.01  # 拾取与放置速度比例

# 接收自pss信号


# 发送至PSS Modbus寄存器地址
pss_modbus_write_dic = {
    "box_request_in_process": 1000,
    "box_request_finished": 1001,
    "pen_request_in_process": 1002,
    "pen_request_finished": 1003,
    "box_handout_in_process": 1004,
    "box_handout_finished": 1005,
    "pen_handout_in_process": 1006,
    "pen_handout_finished": 1007,
    "robot_at_home": 1008,
    "robot_program_start": 1009,
    "robot_stopped": 1010,
    "box_missing": 1011,
    "pen_missing": 1012,
    "use_gripper": 1013,
    "use_sucker": 1014,
    "gripper_open": 1015,
    "gripper_close": 1016,
    "sucker_on": 1017,
}


def pss_modbus_write(start_idx, values):
    rospy.wait_for_service("/pilz_modbus_client_node/modbus_write")
    try:
        modbus_write_client = rospy.ServiceProxy(
            "/pilz_modbus_client_node/modbus_write", WriteModbusRegister
        )
        modbus_write_client(ModbusRegisterBlock(start_idx, values))
    except rospy.ServiceException as e:
        print("Modbus write service call failed: %s", e)


def pss_modbus_read_callback(msg):
    global box_request
    global pen_request
    global box_handout
    global pen_handout

    robot_run_permission = msg.holding_registers.data[4]
    box_request = msg.holding_registers.data[21]
    pen_request = msg.holding_registers.data[22]
    box_handout = msg.holding_registers.data[23]
    pen_handout = msg.holding_registers.data[24]
    external_start = msg.holding_registers.data[25]
    external_stop = msg.holding_registers.data[26]
    external_reset = msg.holding_registers.data[27]

    if not robot_run_permission or external_stop:
        r.pause()
        pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [1])

    if robot_run_permission and external_start:
        rospy.sleep(1)
        r.resume()
        pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [0])


def callback_AgvToSmfStatus(msg):
    global agv_at_smf
    global agv_prbt_changing_box
    global agv_prbt_changing_pen
    global agv_prbt_at_home

    agv_at_smf = msg.agvAtSmf
    agv_prbt_changing_box = msg.agvPrbtChangingBox
    agv_prbt_changing_pen = msg.agvPrbtChangingPen
    agv_prbt_at_home = msg.agvPrbtAtHome


def get_box_pen_center():
    """
    拍照并获取笔和名片夹Y方向位置
    """
    global box_centers_y
    global pen_centers_y
    rospy.wait_for_service("get_box_pen_center")
    try:
        get_box_pen_center = rospy.ServiceProxy("get_box_pen_center", GetBoxPenCenter)
        response = get_box_pen_center()
        box_centers_y = response.box_centers_y
        pen_centers_y = response.pen_centers_y
    except rospy.ServiceException as e:
        print("Service call failed: %s", e)
    if len(pen_centers_y) == 0:
        rospy.loginfo("pen missing!")
        pss_modbus_write(pss_modbus_write_dic["pen_missing"], [1])
        smfToAgvMsg.penMissing = True
        smf_to_agv_publisher.publish(smfToAgvMsg)
    else:
        pss_modbus_write(pss_modbus_write_dic["pen_missing"], [0])
        smfToAgvMsg.penMissing = False
        smf_to_agv_publisher.publish(smfToAgvMsg)
    if len(box_centers_y) == 0:
        rospy.loginfo("box missing!")
        pss_modbus_write(pss_modbus_write_dic["box_missing"], [1])
        smfToAgvMsg.boxMissing = True
        smf_to_agv_publisher.publish(smfToAgvMsg)
    else:
        pss_modbus_write(pss_modbus_write_dic["box_missing"], [0])
        smfToAgvMsg.boxMissing = False
        smf_to_agv_publisher.publish(smfToAgvMsg)


def init_modbus_value():
    """
    初始化部分通讯状态
    """
    pss_modbus_write(pss_modbus_write_dic["box_request_in_process"], [0])
    pss_modbus_write(pss_modbus_write_dic["box_request_finished"], [0])
    pss_modbus_write(pss_modbus_write_dic["pen_request_in_process"], [0])
    pss_modbus_write(pss_modbus_write_dic["pen_request_finished"], [0])
    pss_modbus_write(pss_modbus_write_dic["box_handout_in_process"], [0])
    pss_modbus_write(pss_modbus_write_dic["box_handout_finished"], [0])
    pss_modbus_write(pss_modbus_write_dic["pen_handout_in_process"], [0])
    pss_modbus_write(pss_modbus_write_dic["pen_handout_finished"], [0])
    pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [1])
    pss_modbus_write(pss_modbus_write_dic["box_missing"], [0])
    pss_modbus_write(pss_modbus_write_dic["pen_missing"], [0])
    pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [0])
    pss_modbus_write(pss_modbus_write_dic["gripper_open"], [1])
    pss_modbus_write(pss_modbus_write_dic["gripper_close"], [0])
    pss_modbus_write(pss_modbus_write_dic["sucker_on"], [0])
    pss_modbus_write(pss_modbus_write_dic["robot_program_start"], [0])
    rospy.sleep(1)
    pss_modbus_write(pss_modbus_write_dic["robot_program_start"], [1])


if __name__ == "__main__":

    # 创建节点
    rospy.init_node("robot_program_node")
    rospy.loginfo("Smart factory ros program started")

    r = Robot("1")  # 创建化机器人实例

    smf_to_agv_publisher = rospy.Publisher(
        "smf_to_agv_status", SmfToAgvStatus, queue_size=10
    )

    rospy.Subscriber(
        "/pilz_modbus_client_node/modbus_read",
        ModbusMsgInStamped,
        pss_modbus_read_callback,
        queue_size=1,
    )

    rospy.Subscriber("agv_to_smf_status", AgvToSmfStatus, callback_AgvToSmfStatus)

    smfToAgvMsg = SmfToAgvStatus()

    init_modbus_value()

    """
    工艺安全设置
    获取当前机器人位置
    若当前Z位置小于安全高度，则反向提升至安全高度
    若当前位置Y>300mm，Z>400mm，则反向提升100mm
    """
    current_pose = r.get_current_pose()
    if current_pose.position.z < SAFETY_HEIGHT:
        r.move(
            Lin(
                goal=Pose(
                    position=Point(
                        current_pose.position.x, current_pose.position.y, SAFETY_HEIGHT
                    )
                ),
                reference_frame="prbt_base_link",
                vel_scale=LIN_SCALE,
                acc_scale=0.1,
            )
        )
    elif current_pose.position.y > 0.3 and current_pose.position.z > 0.4:
        r.move(
            Lin(
                goal=Pose(position=Point(0, 0, -0.1)),
                reference_frame="prbt_tcp",
                vel_scale=LIN_SCALE,
                acc_scale=0.1,
            )
        )
    r.move(Ptp(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
    pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [1])
    smfToAgvMsg.smfPrbtAtHome = True
    smf_to_agv_publisher.publish(smfToAgvMsg)

    while not rospy.is_shutdown():
        get_box_pen_center()

        # 名片盒取料工序
        if (
            box_request
            and (len(box_centers_y) != 0)
            and not (agv_prbt_changing_box or agv_prbt_changing_pen)
        ):
            pss_modbus_write(pss_modbus_write_dic["box_request_in_process"], [1])
            pss_modbus_write(pss_modbus_write_dic["box_request_finished"], [0])
            pss_modbus_write(pss_modbus_write_dic["use_gripper"], [0])
            pss_modbus_write(pss_modbus_write_dic["use_sucker"], [1])
            pss_modbus_write(pss_modbus_write_dic["sucker_on"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [0])

            smfToAgvMsg.smfPrbtPickingBox = True
            smfToAgvMsg.smfPrbtAtHome = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

            box_stock_pick_up_pose = Pose(
                position=Point(STOCK_BOX_X, box_centers_y[0], STOCK_Z_UP),
                orientation=SUCKER_INFEED_ORIENTATION,
            )
            box_stock_pick_down_pose = Pose(
                position=Point(STOCK_BOX_X, box_centers_y[0], STOCK_BOX_Z_DOWN),
                orientation=SUCKER_INFEED_ORIENTATION,
            )
            box_conveyor_place_up_pose = Pose(
                position=Point(0.3865, 0.165, STOCK_Z_UP),
                orientation=SUCKER_INFEED_ORIENTATION,
            )
            box_conveyor_place_down_pose = Pose(
                position=Point(0.3865, 0.165, PLATE_BOX_Z_DOWN),
                orientation=SUCKER_INFEED_ORIENTATION,
            )

            r.move(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(
                Lin(
                    goal=box_stock_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(
                Lin(
                    goal=box_stock_pick_down_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["sucker_on"], [1])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=box_stock_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(
                Lin(
                    goal=box_conveyor_place_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(
                Lin(
                    goal=box_conveyor_place_down_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["sucker_on"], [0])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=box_conveyor_place_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))

            pss_modbus_write(pss_modbus_write_dic["box_request_in_process"], [0])
            pss_modbus_write(pss_modbus_write_dic["box_request_finished"], [1])
            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [1])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [1])

            smfToAgvMsg.smfPrbtPickingBox = False
            smfToAgvMsg.smfPrbtAtHome = True
            smf_to_agv_publisher.publish(smfToAgvMsg)

        else:
            pss_modbus_write(pss_modbus_write_dic["box_request_finished"], [0])

            smfToAgvMsg.smfPrbtPickingBox = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

        # 笔取料工序
        if (
            pen_request
            and (len(pen_centers_y) != 0)
            and not (agv_prbt_changing_box or agv_prbt_changing_pen)
        ):
            pss_modbus_write(pss_modbus_write_dic["pen_request_in_process"], [1])
            pss_modbus_write(pss_modbus_write_dic["pen_request_finished"], [0])
            pss_modbus_write(pss_modbus_write_dic["use_gripper"], [1])
            pss_modbus_write(pss_modbus_write_dic["use_sucker"], [0])
            pss_modbus_write(pss_modbus_write_dic["gripper_open"], [1])
            pss_modbus_write(pss_modbus_write_dic["gripper_close"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [0])

            smfToAgvMsg.smfPrbtPickingPen = True
            smfToAgvMsg.smfPrbtAtHome = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

            pen_stock_pick_up_pose = Pose(
                position=Point(STOCK_PEN_X, pen_centers_y[0], STOCK_Z_UP),
                orientation=GRIPPER_ORIENTATION,
            )
            pen_stock_pick_down_pose = Pose(
                position=Point(STOCK_PEN_X, pen_centers_y[0], STOCK_PEN_Z_DOWN),
                orientation=GRIPPER_ORIENTATION,
            )
            pen_conveyor_place_up_pose = Pose(
                position=Point(0.393, 0.1665, STOCK_Z_UP),
                orientation=GRIPPER_ORIENTATION,
            )
            pen_conveyor_place_down_pose = Pose(
                position=Point(0.393, 0.1665, PLATE_PEN_Z_DOWN),
                orientation=GRIPPER_ORIENTATION,
            )

            r.move(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
            r.move(
                Lin(
                    goal=pen_stock_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(
                Lin(
                    goal=pen_stock_pick_down_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["gripper_open"], [0])
            pss_modbus_write(pss_modbus_write_dic["gripper_close"], [1])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=pen_stock_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(
                Lin(
                    goal=pen_conveyor_place_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(
                Lin(
                    goal=pen_conveyor_place_down_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["gripper_open"], [1])
            pss_modbus_write(pss_modbus_write_dic["gripper_close"], [0])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=pen_conveyor_place_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))

            pss_modbus_write(pss_modbus_write_dic["pen_request_in_process"], [0])
            pss_modbus_write(pss_modbus_write_dic["pen_request_finished"], [1])
            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [1])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [1])

            smfToAgvMsg.smfPrbtPickingPen = False
            smfToAgvMsg.smfPrbtAtHome = True
            smf_to_agv_publisher.publish(smfToAgvMsg)

        else:
            pss_modbus_write(pss_modbus_write_dic["pen_request_finished"], [0])

            smfToAgvMsg.smfPrbtPickingPen = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

        # 名片盒出料工序
        if box_handout and not (agv_prbt_changing_box or agv_prbt_changing_pen):
            pss_modbus_write(pss_modbus_write_dic["box_handout_in_process"], [1])
            pss_modbus_write(pss_modbus_write_dic["box_handout_finished"], [0])
            pss_modbus_write(pss_modbus_write_dic["use_gripper"], [0])
            pss_modbus_write(pss_modbus_write_dic["use_sucker"], [1])
            pss_modbus_write(pss_modbus_write_dic["sucker_on"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [0])

            smfToAgvMsg.smfPrbtPlacingBox = True
            smfToAgvMsg.smfPrbtAtHome = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

            box_conveyor_pick_up_pose = Pose(
                position=Point(-0.011, 0.33, STOCK_Z_UP),
                orientation=SUCKER_OUTFEED_ORIENTATION,
            )
            box_conveyor_pick_down_pose = Pose(
                position=Point(-0.011, 0.33, PLATE_BOX_Z_DOWN),
                orientation=SUCKER_OUTFEED_ORIENTATION,
            )
            box_outlet_in_pose = [0.082, -0.046, -1.211, 0.615, -1.850, -0.508]
            box_outlet_out_pose = [0.629, 0.069, -1.243, 0.464, -1.458, -0.140]

            box_handout_pick_seq = Sequence()
            box_handout_pick_seq.append(
                Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1)
            )
            box_handout_pick_seq.append(
                Lin(goal=ROBOT_HANDOUT_MIDDLE_WP, vel_scale=LIN_SCALE, acc_scale=0.1),
                blend_radius=0.1,
            )
            box_handout_pick_seq.append(
                Lin(
                    goal=box_conveyor_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(box_handout_pick_seq)
            r.move(
                Lin(
                    goal=box_conveyor_pick_down_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["sucker_on"], [1])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=box_conveyor_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["box_handout_in_process"], [0])
            pss_modbus_write(pss_modbus_write_dic["box_handout_finished"], [1])

            box_handout_place_seq = Sequence()
            box_handout_place_seq.append(
                Lin(goal=ROBOT_HANDOUT_MIDDLE_WP, vel_scale=LIN_SCALE, acc_scale=0.1),
                blend_radius=0.1,
            )
            box_handout_place_seq.append(
                Lin(
                    goal=box_outlet_in_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                ),
                blend_radius=0.1,
            )
            box_handout_place_seq.append(
                Lin(
                    goal=box_outlet_out_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(box_handout_place_seq)

            pss_modbus_write(pss_modbus_write_dic["sucker_on"], [0])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=box_outlet_in_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))

            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [1])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [1])

            smfToAgvMsg.smfPrbtPlacingBox = False
            smfToAgvMsg.smfPrbtAtHome = True
            smf_to_agv_publisher.publish(smfToAgvMsg)

        else:
            pss_modbus_write(pss_modbus_write_dic["box_handout_finished"], [0])

            smfToAgvMsg.smfPrbtPlacingBox = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

        # 笔出料工序
        if pen_handout and not (agv_prbt_changing_box or agv_prbt_changing_pen):
            pss_modbus_write(pss_modbus_write_dic["pen_handout_in_process"], [1])
            pss_modbus_write(pss_modbus_write_dic["pen_handout_finished"], [0])
            pss_modbus_write(pss_modbus_write_dic["use_gripper"], [1])
            pss_modbus_write(pss_modbus_write_dic["use_sucker"], [0])
            pss_modbus_write(pss_modbus_write_dic["gripper_open"], [1])
            pss_modbus_write(pss_modbus_write_dic["gripper_close"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [0])
            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [0])

            smfToAgvMsg.smfPrbtPlacingPen = True
            smfToAgvMsg.smfPrbtAtHome = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

            pen_conveyor_pick_up_pose = Pose(
                position=Point(-0.005, 0.329, STOCK_Z_UP),
                orientation=GRIPPER_ORIENTATION,
            )
            pen_conveyor_pick_down_pose = Pose(
                position=Point(-0.005, 0.329, PLATE_PEN_Z_DOWN),
                orientation=GRIPPER_ORIENTATION,
            )
            pen_outlet_in_pose = [-0.010, 0.040, -1.106, 0.782, -1.889, 1.068]
            pen_outlet_out_pose = [0.439, 0.061, -1.328, 0.659, -1.425, 1.233]

            pen_handout_pick_seq = Sequence()
            pen_handout_pick_seq.append(
                Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1)
            )
            pen_handout_pick_seq.append(
                Lin(goal=ROBOT_HANDOUT_MIDDLE_WP, vel_scale=LIN_SCALE, acc_scale=0.1),
                blend_radius=0.1,
            )
            pen_handout_pick_seq.append(
                Lin(
                    goal=pen_conveyor_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(pen_handout_pick_seq)
            r.move(
                Lin(
                    goal=pen_conveyor_pick_down_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["gripper_open"], [0])
            pss_modbus_write(pss_modbus_write_dic["gripper_close"], [1])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=pen_conveyor_pick_up_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )

            pss_modbus_write(pss_modbus_write_dic["pen_handout_in_process"], [0])
            pss_modbus_write(pss_modbus_write_dic["pen_handout_finished"], [1])

            pen_handout_place_seq = Sequence()
            pen_handout_place_seq.append(
                Lin(goal=ROBOT_HANDOUT_MIDDLE_WP, vel_scale=LIN_SCALE, acc_scale=0.1),
                blend_radius=0.1,
            )
            pen_handout_place_seq.append(
                Lin(
                    goal=pen_outlet_in_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=LIN_SCALE,
                    acc_scale=0.1,
                ),
                blend_radius=0.1,
            )
            pen_handout_place_seq.append(
                Lin(
                    goal=pen_outlet_out_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(pen_handout_place_seq)

            pss_modbus_write(pss_modbus_write_dic["gripper_open"], [1])
            pss_modbus_write(pss_modbus_write_dic["gripper_close"], [0])
            rospy.sleep(0.5)

            r.move(
                Lin(
                    goal=pen_outlet_in_pose,
                    reference_frame="prbt_base_link",
                    vel_scale=PNP_SCALE,
                    acc_scale=0.1,
                )
            )
            r.move(Lin(goal=START_POSE, vel_scale=LIN_SCALE, acc_scale=0.1))
            pss_modbus_write(pss_modbus_write_dic["robot_at_home"], [1])
            pss_modbus_write(pss_modbus_write_dic["robot_stopped"], [1])

            smfToAgvMsg.smfPrbtPlacingPen = False
            smfToAgvMsg.smfPrbtAtHome = True
            smf_to_agv_publisher.publish(smfToAgvMsg)
        else:
            pss_modbus_write(pss_modbus_write_dic["pen_handout_finished"], [0])

            smfToAgvMsg.smfPrbtPlacingBox = False
            smf_to_agv_publisher.publish(smfToAgvMsg)

        rospy.sleep(1)

    rospy.spin()
