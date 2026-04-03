# 语法：导入整个 numpy 库，简写为 np
# 作用：Python 最牛的数值计算工具（算转向扭矩、加速度、车速用）
# 场景：openpilot 计算 Macan 方向盘要转多大力、车要加减速多少
# 归纳：计算工具，数值处理库，简写为:np
import numpy as np
# 语法：从 opendbc.can 模块中导入 CANPacker 工具类
# 作用：CAN报文打包工具，把控制指令转换成汽车能识别的CAN信号
# 场景：给Macan发送转向、加速指令前，必须用它打包CAN数据
# 归纳：CAN通信工具（信号打包器）
from opendbc.can import CANPacker
# 语法：从 opendbc.car 模块中批量导入 Bus、DT_CTRL、structs 三个工具/常量
# 作用：定义CAN总线类型、控制周期、数据结构标准
# 场景：规定openpilot与Macan通信的基础规则和数据格式
# 归纳：CAN通信工具（总线类型、控制周期、数据结构）
from opendbc.car import Bus, DT_CTRL, structs
# 语法：从横向控制模块导入转向扭矩限制函数
# 作用：限制方向盘输出扭矩，保护车辆转向机，防止过载
# 场景：Macan电动助力转向安全保护，避免输出过大扭矩损坏部件
# 归纳：横向控制工具（转向扭矩限制）
from opendbc.car.lateral import apply_driver_steer_torque_limits
# 语法：导入单位转换工具类，简写为 CV
# 作用：车速、距离等单位转换（米/秒 ↔ 公里/小时）
# 场景：Macan仪表盘巡航速度显示、openpilot内部单位统一
# 归纳：单位转换工具Conversions代码里简写为:CV
from opendbc.car.common.conversions import Conversions as CV
# 语法：导入车辆控制器基类
# 作用：所有车型控制器的通用模板，定义必须实现的功能
# 场景：作为Macan控制器的父类，提供通用控制框架
# 归纳：控制器基类，定义控制接口和通用功能(底层规范 / 接口层 )
from opendbc.car.interfaces import CarControllerBase
# 语法：从大众车型模块导入三大平台的CAN控制工具
# 作用：分别对应PQ老平台、MQB横置、MLB纵置平台的CAN指令逻辑
# 场景：【Macan核心】mlbcan是保时捷Macan专属的CAN控制工具
# 归纳：大众平台CAN控制工具（PQ、MQB、MLB）,发送can指令的工具类，封装了不同平台的CAN消息创建逻辑
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
# 语法：从大众参数模块导入总线、控制器参数、平台标记
# 作用：存储大众全系车型的硬件参数、平台标识、控制限值
# 场景：【Macan核心】通过VolkswagenFlags.MLB识别Macan车型
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags
# 语法：导入智能巡航按钮管理接口
# 作用：MLB平台专属，模拟原厂ACC巡航按钮操作
# 场景：Macan自动控制巡航开启、取消、跟车距离调节
from opendbc.sunnypilot.car.volkswagen.icbm import IntelligentCruiseButtonManagementInterface

#别名赋值：简化赋值名称，方便使用
#openpilot 定义的 「仪表盘视觉警报枚举」（车道提醒、接管报警等）控制 Macan 仪表盘弹出的报警图标
VisualAlert = structs.CarControl.HUDControl.VisualAlert
#openpilot 「纵向 ACC 控制状态枚举」（停车、起步、正常跟车）判断 Macan ACC 是在减速、停车还是起步
LongCtrlState = structs.CarControl.Actuators.LongControlState

# 定义一个工具类，专门处理HCA车道保持的故障缓解
# 管理大众/奥迪/保时捷EPS转向机的HCA故障缓解:长时间发送相同扭矩值后,仅一帧把扭矩加减1,骗过转向机
class HCAMitigation:
  # 初始化方法：创建类的时候，自动执行
  def __init__(self, CCP):
    # 计算【最大允许连续相同扭矩的帧数】
    # CCP：车型控制参数  DT_CTRL：控制周期(10ms)  STEER_STEP：发送间隔
    # STEER_TIME_STUCK_TORQUE：根据代码values取值:PQ、MQB、MLB,STEER_TIME_STUCK_TORQUE均为1.9，单位秒，表示连续相同扭矩超过1.9秒就要微调一次
    # 根据values代码里定值:PQ、MQB、MLB平台 STEER_STEP=2（50Hz）
    # 根据_init_.py里定义的DT_CTRL=0.01s，计算出最大连续相同扭矩的帧数为1.9秒/（0.01秒*2）=95帧
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    # 初始化计数器，记录连续相同扭矩的帧数,添加self前缀表示，这个参与本代码的全局计数，不局限在本函数内.
    self._same_torque_frames = 0
  # 实施方法：每次计算新的转向扭矩时调用，监控连续相同扭矩的情况，并在必要时微调扭矩值以避免HCA故障
  def update(self, apply_torque, apply_torque_last):
    # 如果当前扭矩不为0且与上次相同，计数器+1
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      # 如果连续相同扭矩的帧数超过最大值，微调扭矩值，重置计数器
      if self._same_torque_frames > self._max_same_torque_frames:
        # 通过微调扭矩值（扭矩为正数-1；扭矩为负数+1）来欺骗转向机，避免HCA故障
        # (1,-1)=条件成立返回1，不成立返回-1，apply_torque<0就是条件，成立时返回-1，反之返回1.
        apply_torque -= (1, -1)[apply_torque < 0]
        # 计数器清零
        self._same_torque_frames = 0
    else:
      # 如果当前扭矩为0或与上次不同，重置计数器
      self._same_torque_frames = 0
    #返回扭矩值，这个函数其实就是在监控连续相同扭矩的情况，并在必要时微调扭矩值以避免HCA故障
    return apply_torque


# 1. 继承类：CarControllerBase→给你标准格式、基础参数（定规矩）；
#           IntelligentCruiseButtonManagementInterface→控制器直接获得「自动模拟 ACC 按键」的能力
class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    # 2. 父类调用参数初始化
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    #3.从opendbc/car/volkswagen/values代码中传入:CanBus, CarControllerParams, VolkswagenFlags三个函数
    #  opendbc/can中传入CANPacker
    # 传入CarControllerParams里的参数CP
    self.CCP = CarControllerParams(CP)
    # 传入CanBus里的参数CP
    self.CAN = CanBus(CP)
    # 初始化动力总线(Power Train)报文打包器，传入对应总线的DBC协议文件，mlb车型Bus.pt=Bus 0
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    # 传入VolkswagenFlags函数里的PQ布尔值与传入参数CP的flags标识
    # 位运算 & 的优先级 ＞ 逻辑非 not 判断AEB自动刹车是否可用：**不是PQ老平台**的车（MLB/MQB），支持AEB
    # 等效：self.aeb_available = not (CP.flags & VolkswagenFlags.PQ)
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ
   # CP.flags：车型平台身份标记，在interface.py+values.py自动赋值
   # Macan(MLB) = 8，PQ平台=2，MQB=0
   # & 位与运算符 检查是否包含某个标记 不是比较符 ==。也不是逻辑与 and；
    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    # 1. 初始化基础状态变量（记录上一帧的控制数据，用于平滑计算）
    self.apply_torque_last = 0 # 上一帧实际输出的转向扭矩
    self.gra_acc_counter_last = None # 上一帧ACC控制的计数器
    # 2. 实例化HCA防卡死保护类（传入CCP车型控制参数，之前学的大礼包）
    self.hca_mitigation = HCAMitigation(self.CCP)

    # 3. MLB平台专属：EPS转向计时器 修复开关（核心！）
    # 位与&判断：是否为MLB/Macan平台 → bool转成True/False
    # 含义：仅保时捷Macan（MLB）开启【EPS转向超时修复功能】
    self.eps_timer_workaround = bool(CP.flags & VolkswagenFlags.MLB)
    # 4. 初始化EPS修复功能的计时变量
    self.hca_frame_timer_resetting = 0 # 转向计时器重置计数
    self.hca_frame_low_torque = 0 # 低扭矩状态持续帧数
    self.hca_frame_timer_running = 0 # 转向计时器运行帧数

  # 【核心指挥函数】每0.01秒运行一次，计算并发送车辆控制指令
  # 传入参数：都是系统传进来的「实时数据大礼包」
  # self：自己本身（固定写法）
  # CC：openpilot主控制指令（转向、加速、刹车、仪表显示）
  # CC_SP：SunnyPilot扩展指令
  # CS：车辆当前真实状态（车速、方向盘、踏板、CAN数据）
  # now_nanos：当前时间戳（用来计时）
  def update(self, CC, CC_SP, CS, now_nanos):
    # 1. 取出「执行器指令」：转向扭矩、油门、刹车
    actuators = CC.actuators
    # 2. 取出「仪表显示指令」：ACC速度、车道保持提示等
    hud_control = CC.hudControl
    # 3. 创建一个空列表：用来存放所有要发给车辆的CAN消息
    # 所有转向指令、ACC指令，最后都塞进这个列表发出去
    can_sends = []
    # 4. 初始化「最终输出转向扭矩」= 0
    # 最开始先不转方向盘，后面计算完再赋值
    output_torque = 0
    # 【新增】从CS里获取驾驶员是否踩油门
    gas_pressed = CS.out.gasPressed
    # **** Steering Controls(转向控制部分) ************************************************ #
    # 【核心逻辑】每隔 STEER_STEP 帧计算一次转向扭矩，控制方向盘转动，根据values代码里定值,PQ、MQB、MLB平台 STEER_STEP=2（50Hz）
    # % 为取余计算，判断当前帧数是否是 STEER_STEP 的倍数，只有在满足条件的帧才执行转向控制逻辑
    if self.frame % self.CCP.STEER_STEP == 0:
      # 先初始化转向角扭矩为0，后面根据条件计算实际要输出的转向扭矩
      apply_torque = 0
      # 判断是否需要执行转向控制：CC.latActive 表示横向控制是否激活（openpilot要求转向控制生效的条件）
      if CC.latActive:
        # 1. 计算期望转向扭矩：根据openpilot的转向指令（actuators.torque）和车型控制参数（CCP.STEER_MAX）计算出实际要输出的转向扭矩值
        # 根据values代码里定值:PQ、MQB、MLB平台 STEER_MAX=300，表示最大转向扭矩为3.00Nm，actuators.torque是-1.0到1.0的比例值，乘以300得到实际扭矩值
        # 根据controlsd.py里定义的actuators.torque是openpilot横向控制器输出的转向指令，范围是-1.0到1.0，表示转向扭矩的百分比，乘以CCP.STEER_MAX得到实际要输出的转向扭矩值
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        '''
        def apply_driver_steer_torque_limits(apply_torque: int, apply_torque_last: int, driver_torque: float, LIMITS, steer_max: int | None = None):
          #apply_torque: int,        # openpilot 想要输出的目标扭矩
          #apply_torque_last: int,   # 上一次实际输出的扭矩
          #driver_torque: float,    # 司机当前施加的扭矩
          #LIMITS: CarControllerParams.LateralTorqueLimits,  # 车型的转向扭矩限制参数
          # some safety modes utilize a dynamic max steer
          # 一些安全模式使用动态最大转向值，默认用车型固定的 STEER_MAX，但如果 steer_max 参数被传入，则使用该值作为最大转向限制。这允许在特定情况下调整最大转向限制，例如根据车辆状态或环境条件。
          if steer_max is None:
            steer_max = LIMITS.STEER_MAX

          # limits due to driver torque
          # 允许的最大辅助扭矩（司机往右打，辅助就少出力）
          driver_max_torque = steer_max + (LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
          # 允许的最小辅助扭矩（司机往左打，辅助就少出力）
          driver_min_torque = -steer_max + (-LIMITS.STEER_DRIVER_ALLOWANCE + driver_torque * LIMITS.STEER_DRIVER_FACTOR) * LIMITS.STEER_DRIVER_MULTIPLIER
          # 把辅助扭矩锁在安全范围内
          # 根据司机当前施加的扭矩和车型的转向扭矩限制参数，计算出允许的最大和最小辅助扭矩。这些限制确保了openpilot的转向控制不会与司机的输入产生过大的冲突，从而提高安全性。
          max_steer_allowed = max(min(steer_max, driver_max_torque), 0)
          min_steer_allowed = min(max(-steer_max, driver_min_torque), 0)
          apply_torque = np.clip(apply_torque, min_steer_allowed, max_steer_allowed)
          # 第二重安全：速率限制（防止方向盘抖 / 猛打） 这是方向盘手感丝滑的关键！
          # slow rate if steer torque increases in magnitude
          # 扭矩上升：慢一点
          if apply_torque_last > 0:
            apply_torque = np.clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                                  apply_torque_last + LIMITS.STEER_DELTA_UP)
          # 扭矩下降：快一点（更安全）
          else:
            apply_torque = np.clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                                  min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))
          # 输出最终整数扭矩 （因为CAN消息里转向扭矩通常是整数，单位是0.01Nm，所以要四舍五入并转换成整数）
          return int(round(float(apply_torque)))
          '''
        # 2. 应用转向扭矩限制：根据司机当前施加的转向扭矩和车型的转向限制参数，调整openpilot的目标转向扭矩，确保安全性（最大值限定）和舒适性（速率限制）
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

        # 3. 更新EPS转向计时器：如果是MLB/Macan平台，记录连续转向的时间，用于后续的HCA故障缓解逻辑，STEER_STEP=2（50Hz），既没运行一次时间+0,02s.
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        # 4. 判断本次计算的扭矩值跟上一次实际输出的扭矩值是否相同，如果相同，说明可能出现HCA故障的风险，调用HCA防卡死保护类进行微调
        apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
        # 5. 判断是否需要启用HCA：如果计算出的转向扭矩不为0，则启用HCA；如果为0，则禁用HCA（不转方向盘）
        hca_enabled = abs(apply_torque) > 0

        # 6. 6分钟mlb平台的时间炸弹补丁：如果连续转向时间超过4分钟（240秒），并且当前扭矩处于低扭矩状态（小于最大扭矩的20%），则认为转向机可能进入了HCA故障状态，暂时禁用HCA，直到转向机恢复
        # self.eps_timer_workaround = bool(CP.flags & VolkswagenFlags.MLB) 判定是否MLB平台，只有MLB/Macan才开启EPS转向超时修复功能
        # hca_frame_timer_running 记录连续转向的时间，单位是帧数，STEER_STEP=2（50Hz），每运行一次时间+0,02s
        # CCP.STEER_TIME_BM / DT_CTRL 计算出最大连续转向时间对应的帧数，超过这个帧数就要进入低扭矩状态
        # STEER_TIME_BM = STEER_TIME_MAX - 120；STEER_TIME_MAX = 360 计算出：STEER_TIME_BM = 240，单位秒，也就是240/60=4分钟，换算成帧数为240/0.01=24000帧
        if self.eps_timer_workaround and self.hca_frame_timer_running >= self.CCP.STEER_TIME_BM / DT_CTRL:
          # STEER_LOW_TORQUE = int(STEER_MAX * 0.20)=60，表示低扭矩状态的阈值为最大转向扭矩的20%，也就是0.60Nm，当扭矩小于0.60Nm时，认为是低扭矩状态
          if abs(apply_torque) <= self.CCP.STEER_LOW_TORQUE:
            # hca_frame_low_torque 记录低扭矩状态持续的帧数，每执行一次增加2帧;
            self.hca_frame_low_torque += self.CCP.STEER_STEP
            # 如果低扭矩状态持续的帧数超过 STEER_TIME_LOW_TORQUE / DT_CTRL 对应的帧数24000，说明转向机可能进入了HCA故障状态，需要暂时禁用HCA，直到转向机恢复
            if self.hca_frame_low_torque >= self.CCP.STEER_TIME_LOW_TORQUE / DT_CTRL:
              hca_enabled = False
          else:
            # 如果当前扭矩不在低扭矩范围内，重置低扭矩计数器，并且如果正在重置计时器，则暂时禁用HCA，直到转向机恢复
            self.hca_frame_low_torque = 0
            if self.hca_frame_timer_resetting > 0:
              apply_torque = 0
      # 如果转向控制没有激活，或者因为HCA故障缓解逻辑而禁用了HCA，则重置相关计时器和状态，确保安全性
      else:
        self.hca_frame_low_torque = 0
        hca_enabled = False
        apply_torque = 0

      # 7. 根据是否启用HCA，决定最终输出的转向扭矩：如果启用HCA，则输出计算得到的转向扭矩；如果禁用HCA，则输出0（不转方向盘），并且更新计时器状态
      if hca_enabled:
        output_torque = apply_torque
        self.hca_frame_timer_resetting = 0
      else:
        output_torque = 0
        self.hca_frame_timer_resetting += self.CCP.STEER_STEP
        if self.hca_frame_timer_resetting >= self.CCP.STEER_TIME_RESET / DT_CTRL or not self.eps_timer_workaround:
          self.hca_frame_timer_running = 0
          apply_torque = 0

      # 8. 恢复软禁用警报 (从代码1恢复)
      # 如果转向机处于软禁用状态（连续转向时间超过4分钟，但还没有进入低扭矩状态），则触发EPS警报，提示司机接管
      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      # 9. 记录本次实际输出的转向扭矩存入上一帧的变量，为下一次计算提供参考，参与转向扭矩限制和HCA缓解逻辑
      self.apply_torque_last = apply_torque
      # 10. 创建转向控制的CAN消息，并添加到发送列表中：根据计算得到的转向扭矩和HCA状态，使用对应平台的CAN控制工具创建转向控制消息，发送给车辆
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, output_torque, hca_enabled))

      # 11. 如果车辆配备了原厂HCA系统，且当前正在输出转向扭矩，则发送额外的CAN消息来模拟驾驶员的转向输入，欺骗HCA系统认为司机仍在操作方向盘，从而避免误触发HCA故障警报或限制转向控制
      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection
        # VW的紧急辅助系统会监测司机是否长时间没有操作方向盘，如果检测到可能的驾驶员不活跃状态，可能会触发警报或限制转向控制。为了避免这种情况，可以模拟一个小的转向扭矩，保持系统认为司机仍在操作方向盘，从而避免误触发紧急辅助系统。
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        # 如果当前实际输出的转向扭矩的绝对值大于模拟扭矩的绝对值，则使用当前实际输出的转向扭矩作为模拟扭矩，以确保在需要较大转向输入时，仍然能够满足紧急辅助系统的检测要求。
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          # 根据CS.out.steeringTorque获取当前车辆实际的转向扭矩值，作为模拟扭矩的值，以确保在需要较大转向输入时，仍然能够满足紧急辅助系统的检测要求。
          ea_simulated_torque = CS.out.steeringTorque
        # 创建EPS更新的CAN消息，包含模拟的转向扭矩值，发送给车辆，以欺骗紧急辅助系统认为司机仍在操作方向盘，从而避免误触发紧急辅助系统的警报或限制转向控制。
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #
    # (修正为 gear_ratio，与 mlbcan.py 匹配)
    # 【核心逻辑】如果openpilot控制纵向ACC，则根据当前巡航状态、加速指令和车辆状态计算加速/减速指令，并发送给车辆
    if self.CP.openpilotLongitudinalControl:

      # 控制 ACC04 发送频率为 25Hz,根据values代码里定值:PQ、MQB、MLB平台 ACC_CONTROL_STEP=4（40Hz），每4帧(0.04s)发送一次ACC控制消息
      if self.frame % 4 == 0:
        # hasattr函数检查CS对象是否有acc04_stock_values属性，并且该属性不为空，如果满足条件，则创建ACC04控制消息并添加到发送列表中。这个消息可能包含一些原厂ACC的状态或参数，用于兼容或欺骗原厂系统。
        # hasattr(a,b)函数用于检查对象a是否具有属性b，返回True或False。(Python里的固定搭配)
        # CS为def update(self, CC, CC_SP, CS, now_nanos):中的CS，是车辆当前真实状态（车速、方向盘、踏板、CAN数据）的数据大礼包，acc04_stock_values是其中可能包含的原厂ACC相关的CAN数据，如果存在且不为空，就发送给车辆。
        if hasattr(CS, 'acc04_stock_values') and CS.acc04_stock_values:
          # 根据CS.acc04_stock_values获取原厂ACC04相关的CAN数据，使用对应平台的CAN控制工具创建ACC04控制消息，并添加到发送列表中。这个消息可能包含一些原厂ACC的状态或参数，用于兼容或欺骗原厂系统。
          can_sends.append(self.CCS.create_acc04_control(self.packer_pt, self.CAN.pt, CS.acc04_stock_values))
      # 每 0.02 秒执行一次（50Hz），发送 ACC 加减速指令
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        # 1. 确定ACC当前工作状态（开启/暂停/故障）
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, gas_pressed)
        # 2. 计算加速指令：根据openpilot的加速指令（actuators.accel）和车型控制参数（CCP.ACCEL_MIN、CCP.ACCEL_MAX）计算出实际要输出的加速度值，并且如果ACC没有激活，则强制加速度为0，确保车辆不受控制地加速或减速
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        # 3. 判断车辆是否正在刹停 / 停车起步
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        # 4. 把所有指令打包成CAN消息，发给车辆执行
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation, v_ego=CS.out.vEgo,
                                                           gear_ratio=getattr(CS, 'gear_ratio', 0.0), CS=CS))


    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl:
      lead_distance = getattr(CS, 'stock_lead_distance', 0)
      lead_object = getattr(CS, 'stock_lead_object', 0)
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive, gas_pressed)
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars, lead_object,
                                                       zeitluecke=getattr(CS, 'stock_zeitluecke', 4)))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    # **** Intelligent Cruise Button Management ******************************** #
    # (保持不变，依赖于 __init__ 中正确初始化了接口)

    if self.CP.flags & VolkswagenFlags.MLB:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.CAN.ext,
                                                                        self.frame, self.last_button_frame))

    new_actuators = actuators.as_builder()
    new_actuators.torque = output_torque / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends