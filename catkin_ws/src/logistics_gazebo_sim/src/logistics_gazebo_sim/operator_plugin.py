import json
import os
import shutil
import signal
import socket
import time
import rospy
import rospkg
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from logistics_gazebo_sim.scenes import SCENES, SCALE, metric_xy
from logistics_gazebo_sim.fleet_operator_telemetry import validate_snapshot
from logistics_gazebo_sim.operator_energy_view import validate_energy_advisory
from logistics_gazebo_sim.operator_risk_view import validate_risk_report
from logistics_gazebo_sim.operator_event_journal import OperatorEventJournal
from logistics_gazebo_sim.operator_task_io import (build_operator_report, load_task,
    validate_task, write_json_atomic)
from logistics_gazebo_sim.operator_config_info import (algorithm_profile,
    scene_identity, source_identity)
from python_qt_binding.QtCore import QObject, QProcess, QProcessEnvironment, Qt, QTimer, Signal
from python_qt_binding.QtGui import QColor, QPixmap
from python_qt_binding.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QFileDialog, QHeaderView, QHBoxLayout, QLabel, QMessageBox, QProgressBar, QPushButton,
    QScrollArea, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)
from qt_gui.plugin import Plugin
from std_msgs.msg import String
from std_srvs.srv import Trigger

SCENE_DEFAULTS = {
  0: (-40.0,-40.0,45.0,45.0,8.0), 1: (-40.0,-40.0,40.0,40.0,37.0),
  2: (-25.0,35.0,-5.0,-15.0,12.0), 3: (-40.0,-40.0,40.0,40.0,8.0),
  4: (-45.0,-45.0,45.0,45.0,8.0), 5: (-40.0,0.0,40.0,0.0,10.0),
  6: (-40.0,-40.0,40.0,20.0,12.0)}
class PointBridge(QObject):
    point_received=Signal(float,float)
class RosUiBridge(QObject):
    state_received=Signal(object);diagnostics_received=Signal(object);risk_received=Signal(object);perception_received=Signal(object);energy_return_received=Signal(object);telemetry_received=Signal(object)

class OperatorPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context); self.setObjectName("LogisticsOperator")
        self.widget=QWidget();self.widget.setWindowTitle("无人机物流仿真上位机");self.widget.setMinimumWidth(680);root=QVBoxLayout(self.widget);root.setContentsMargins(16,14,16,16);root.setSpacing(10)
        package_path=rospkg.RosPack().get_path("logistics_gazebo_sim")
        self.package_path=package_path
        self.build_identity=source_identity(package_path)
        self.scene_identity_cache={}
        header=QHBoxLayout();header.setSpacing(14)
        logo=QLabel();logo.setObjectName("brandLogo");logo.setFixedSize(84,84);logo.setAlignment(Qt.AlignCenter)
        logo_path=os.path.join(package_path,"resources","nuaa.jpg");pixmap=QPixmap(logo_path)
        if pixmap.isNull():
            logo.setText("NUAA");rospy.logwarn("Unable to load rqt branding image: %s",logo_path)
        else:
            logo.setPixmap(pixmap.scaled(80,80,Qt.KeepAspectRatio,Qt.SmoothTransformation))
        heading=QVBoxLayout();heading.setSpacing(3)
        title=QLabel("多无人机物流任务控制台 · ROS OMPL 3D");title.setObjectName("title");subtitle=QLabel("场景预览 · 地图选点 · 低空避障 · 编队往返");subtitle.setObjectName("subtitle")
        heading.addStretch();heading.addWidget(title);heading.addWidget(subtitle);heading.addStretch();header.addWidget(logo);header.addLayout(heading,1);root.addLayout(header)
        style_path=os.path.join(package_path,"config","operator.qss")
        with open(style_path,"r",encoding="utf-8") as style_file:self.widget.setStyleSheet(style_file.read())
        box=QGroupBox("\u4eff\u771f\u4e0e\u89c4\u5212"); form=QFormLayout(box)
        self.scene=QComboBox()
        for i,name in enumerate(("\u57ce\u5e02\u7269\u6d41\u914d\u9001","\u9ad8\u5c42\u5efa\u7b51\u914d\u9001","\u590d\u6742\u4ea4\u53c9\u901a\u884c","\u5bc6\u96c6\u8bbe\u65bd\u914d\u9001","\u57ce\u5e02\u516c\u56ed\u5e94\u6025","\u5de5\u4e1a\u8fd0\u8f93","\u5c71\u533a\u533b\u7597\u8fd0\u8f93")): self.scene.addItem("\u573a\u666f{}\uff1a{}".format(i,name),i)
        form.addRow("\u573a\u666f",self.scene)
        self.start_x,self.start_y,self.goal_x,self.goal_y,self.altitude=[QDoubleSpinBox() for _ in range(5)]
        for spin in (self.start_x,self.start_y,self.goal_x,self.goal_y): spin.setRange(-46.0,46.0);spin.setDecimals(1);spin.setSingleStep(1.0);spin.setSuffix(" m")
        self.altitude.setRange(3.0,45.0);self.altitude.setDecimals(1);self.altitude.setSingleStep(1.0);self.altitude.setSuffix(" m")
        start_row=QHBoxLayout();start_row.addWidget(QLabel("X"));start_row.addWidget(self.start_x);start_row.addWidget(QLabel("Y"));start_row.addWidget(self.start_y);self.pick_start=QPushButton("地图选点");start_row.addWidget(self.pick_start);form.addRow("\u81ea\u9009\u8d77\u70b9",start_row)
        goal_row=QHBoxLayout();goal_row.addWidget(QLabel("X"));goal_row.addWidget(self.goal_x);goal_row.addWidget(QLabel("Y"));goal_row.addWidget(self.goal_y);self.pick_goal=QPushButton("地图选点");goal_row.addWidget(self.pick_goal);form.addRow("\u81ea\u9009\u7ec8\u70b9",goal_row)
        form.addRow("\u98de\u884c\u9ad8\u5ea6",self.altitude)
        self.formation=QComboBox();self.formation.addItem("\u6b63\u4e09\u89d2\u961f\u5f62","triangle");self.formation.addItem("\u5012\u4e09\u89d2\u961f\u5f62","inverted");self.formation.addItem("\u6a2a\u961f","row");form.addRow("\u98de\u884c\u961f\u5f62",self.formation)
        self.formation.addItem("纵向一字队形（窄通道）","column");self.formation.addItem("垂直错层队形","vertical");self.formation.addItem("三维楔形队形","wedge3d");self.formation.addItem("三维螺旋队形","helix")
        self.dynamic_enabled=QCheckBox("启用交叉移动障碍物与在线风险预测");self.dynamic_enabled.setChecked(True)
        form.addRow("动态避障实验",self.dynamic_enabled)
        self.avoidance_mode=QComboBox();self.avoidance_mode.addItem("整队避障（推荐）",("collective_offset","shadow",True));self.avoidance_mode.addItem("ORCA 受限接管",("orca3d","limited",True));self.avoidance_mode.addItem("MPC＋ORCA 对照（仅影子）",("distributed_mpc","shadow",False));self.avoidance_mode.setToolTip("只有整队偏移和 ORCA 受限模式可接管；MPC 与 ORCA 回退仅计算候选，不下发轨迹。");form.addRow("局部避障模式",self.avoidance_mode)
        self.orca_max_speed=QDoubleSpinBox();self.orca_max_speed.setRange(0.5,2.0);self.orca_max_speed.setDecimals(1);self.orca_max_speed.setSingleStep(0.1);self.orca_max_speed.setSuffix(" m/s");self.orca_max_speed.setValue(2.0)
        self.orca_timeout=QDoubleSpinBox();self.orca_timeout.setRange(0.3,0.6);self.orca_timeout.setDecimals(2);self.orca_timeout.setSingleStep(0.05);self.orca_timeout.setSuffix(" s");self.orca_timeout.setValue(0.6)
        orca_row=QHBoxLayout();orca_row.addWidget(QLabel("速度上限"));orca_row.addWidget(self.orca_max_speed);orca_row.addWidget(QLabel("指令超时"));orca_row.addWidget(self.orca_timeout)
        form.addRow("ORCA 安全参数",orca_row)
        self.perception_source=QComboBox();self.perception_source.addItem("仿真感知（稳定）","perception");self.perception_source.addItem("物理 3D 雷达（实验）","lidar");self.perception_source.addItem("Gazebo 真值（对照）","truth");form.addRow("动态障碍数据源",self.perception_source)
        simrow=QHBoxLayout();self.start_sim=QPushButton("\u89c4\u5212\u5e76\u542f\u52a8\u4e09\u673a\u4eff\u771f");self.stop_sim=QPushButton("\u505c\u6b62\u4eff\u771f");simrow.addWidget(self.start_sim);simrow.addWidget(self.stop_sim);form.addRow(simrow);root.addWidget(box)
        files_box=QGroupBox("任务参数与界面报告");files_row=QHBoxLayout(files_box)
        self.save_task_button=QPushButton("保存任务参数")
        self.load_task_button=QPushButton("加载任务参数")
        self.reset_task_button=QPushButton("恢复默认参数")
        self.export_report_button=QPushButton("导出界面报告")
        self.export_report_button.setToolTip("只导出上位机快照，不代替飞行真值或安全验收报告")
        for button in (self.save_task_button,self.load_task_button,
                       self.reset_task_button,self.export_report_button):files_row.addWidget(button)
        root.addWidget(files_box)
        config_box=QGroupBox("版本与下次启动配置");config_layout=QVBoxLayout(config_box)
        self.config_summary=QLabel("正在读取源码与场景版本…")
        self.config_summary.setWordWrap(True)
        config_layout.addWidget(self.config_summary);root.addWidget(config_box)
        self.start_sim.setObjectName("primary");self.stop_sim.setObjectName("secondary");self.start_sim.setToolTip("先校验参数并规划安全航线，再启动 Gazebo/PX4");self.start_sim.setEnabled(False)
        analysis=QGroupBox("规划分析");af=QVBoxLayout(analysis)
        self.analysis_state=QLabel("等待参数分析");self.analysis_state.setObjectName("analysisState")
        self.analysis_detail=QLabel("场景、起终点、高度或队形变化后将自动重新规划");self.analysis_detail.setObjectName("analysisDetail");self.analysis_detail.setWordWrap(True)
        af.addWidget(self.analysis_state);af.addWidget(self.analysis_detail);root.addWidget(analysis)
        mission=QGroupBox("\u4efb\u52a1\u63a7\u5236");row=QHBoxLayout(mission)
        self.start=QPushButton("\u5f00\u59cb");self.pause=QPushButton("\u6682\u505c");self.resume=QPushButton("\u7ee7\u7eed");self.reset=QPushButton("\u91cd\u7f6e\u4efb\u52a1");self.land=QPushButton("\u7d27\u6025\u964d\u843d")
        self.start.setObjectName("primary");self.land.setObjectName("danger");self.pause.setObjectName("secondary");self.resume.setObjectName("secondary");self.reset.setObjectName("secondary")
        for button in (self.start,self.pause,self.resume,self.reset,self.land):row.addWidget(button)
        root.addWidget(mission)
        status=QGroupBox("\u72b6\u6001");sf=QFormLayout(status);self.state=QLabel("\u672a\u542f\u52a8");self.stage=QLabel("-");self.safety=QLabel("\u7b49\u5f85\u6570\u636e");self.dynamic_risk=QLabel("等待动态障碍数据");self.perception_status=QLabel("等待感知数据");self.energy_return=QLabel("等待能量返航数据")
        self.state.setObjectName("statusBadge");self.stage.setObjectName("statusBadge")
        self.progress=QProgressBar();self.progress.setRange(0,1000);self.progress.setValue(0);self.progress.setFormat("%p%")
        sf.addRow("任务",self.state);sf.addRow("阶段",self.stage);sf.addRow("任务进度",self.progress);sf.addRow("静态安全",self.safety);sf.addRow("动态风险",self.dynamic_risk);sf.addRow("感知状态",self.perception_status);sf.addRow("能量返航",self.energy_return);root.addWidget(status)
        fleet_box=QGroupBox("车辆状态 · 低频汇总");fleet_layout=QVBoxLayout(fleet_box)
        self.vehicle_summary=QLabel("等待车辆遥测汇总")
        self.vehicle_table=QTableWidget(0,7)
        self.vehicle_table.setHorizontalHeaderLabels(["车辆","状态","模式","高度","电量","估算剩余","遥测年龄"])
        self.vehicle_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.vehicle_table.setSelectionMode(QTableWidget.NoSelection)
        self.vehicle_table.setAlternatingRowColors(True)
        self.vehicle_table.verticalHeader().setVisible(False)
        self.vehicle_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.vehicle_table.setMinimumHeight(250)
        fleet_layout.addWidget(self.vehicle_summary);fleet_layout.addWidget(self.vehicle_table)
        root.addWidget(fleet_box)
        risk_box=QGroupBox("动态冲突预测 · 只读");risk_layout=QVBoxLayout(risk_box)
        self.risk_summary=QLabel("等待动态风险报告")
        self.risk_table=QTableWidget(0,6)
        self.risk_table.setHorizontalHeaderLabels(["车辆","风险","冲突对象","预测最小净空","碰撞倒计时","最近点（预测）"])
        self.risk_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.risk_table.setSelectionMode(QTableWidget.NoSelection)
        self.risk_table.setAlternatingRowColors(True)
        self.risk_table.verticalHeader().setVisible(False)
        self.risk_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.risk_table.setMinimumHeight(230)
        risk_layout.addWidget(self.risk_summary);risk_layout.addWidget(self.risk_table)
        self.prediction_summary=QLabel("等待障碍物轨迹预测")
        self.prediction_table=QTableWidget(0,4)
        self.prediction_table.setHorizontalHeaderLabels(["障碍物","当前估计","预测中点","预测终点"])
        self.prediction_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.prediction_table.setSelectionMode(QTableWidget.NoSelection)
        self.prediction_table.setAlternatingRowColors(True)
        self.prediction_table.verticalHeader().setVisible(False)
        self.prediction_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.prediction_table.setMinimumHeight(190)
        risk_layout.addWidget(self.prediction_summary);risk_layout.addWidget(self.prediction_table)
        root.addWidget(risk_box)
        energy_box=QGroupBox("返航能量建议 · 只读影子模式");energy_layout=QVBoxLayout(energy_box)
        self.energy_summary=QLabel("等待能量返航建议")
        self.energy_table=QTableWidget(0,5)
        self.energy_table.setHorizontalHeaderLabels(["车辆","级别","预计落地余量","返航槽位","降落顺序"])
        self.energy_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.energy_table.setSelectionMode(QTableWidget.NoSelection)
        self.energy_table.setAlternatingRowColors(True)
        self.energy_table.verticalHeader().setVisible(False)
        self.energy_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.energy_table.setMinimumHeight(190)
        energy_layout.addWidget(self.energy_summary);energy_layout.addWidget(self.energy_table)
        root.addWidget(energy_box)
        events_box=QGroupBox("规划与安全事件 · 最近 50 条");events_layout=QVBoxLayout(events_box)
        self.event_summary=QLabel("暂无事件；仅记录状态变化，不重复记录每帧遥测")
        self.event_table=QTableWidget(0,4)
        self.event_table.setHorizontalHeaderLabels(["时间","来源","事件","恢复建议/原因"])
        self.event_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.event_table.setSelectionMode(QTableWidget.NoSelection)
        self.event_table.setAlternatingRowColors(True)
        self.event_table.verticalHeader().setVisible(False)
        self.event_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.event_table.setMinimumHeight(260)
        events_layout.addWidget(self.event_summary);events_layout.addWidget(self.event_table)
        root.addWidget(events_box);root.addStretch()
        self.scroll=QScrollArea();self.scroll.setWidgetResizable(True);self.scroll.setWidget(self.widget)
        context.add_widget(self.scroll)
        self.process=QProcess(self.widget);self.analysis_process=QProcess(self.widget);self.simulation_log_tail=b""
        self.process.setProcessChannelMode(QProcess.MergedChannels);self.process.readyReadStandardOutput.connect(lambda:self.drain_simulation_output())
        self.simulation_stop_requested=False;self.simulation_start_pending=False
        self.process.started.connect(lambda:self.simulation_process_started())
        self.process.errorOccurred.connect(lambda error:self.simulation_process_error(error))
        self.process.finished.connect(lambda code,status:self.simulation_process_finished(code,status))
        self.analysis_timer=QTimer(self.widget);self.analysis_timer.setSingleShot(True);self.analysis_timer.setInterval(700);self.analysis_timer.timeout.connect(lambda:self.start_analysis())
        self.analysis_timeout=QTimer(self.widget);self.analysis_timeout.setSingleShot(True);self.analysis_timeout.setInterval(20000);self.analysis_timeout.timeout.connect(lambda:self.analysis_timed_out())
        self.analysis_process.finished.connect(lambda code,status:self.analysis_finished(code,status))
        self.valid_analysis_signature=None;self.analysis_running_signature=None;self.analysis_mission=None;self.analysis_report=None;self.analysis_retry=0
        self.pick_mode="start";self.point_bridge=PointBridge();self.point_bridge.point_received.connect(lambda x,y:self.apply_clicked_point(x,y),Qt.QueuedConnection)
        self.scene.currentIndexChanged.connect(lambda _index:self.update_defaults());self.start_sim.clicked.connect(lambda:self.launch_sim());self.stop_sim.clicked.connect(lambda:self.stop_simulation())
        self.avoidance_mode.currentIndexChanged.connect(lambda _index:self.update_algorithm_controls())
        self.dynamic_enabled.toggled.connect(lambda _checked:self.update_algorithm_controls())
        for control in (self.start_x,self.start_y,self.goal_x,self.goal_y,self.altitude,
                        self.orca_max_speed,self.orca_timeout):
            control.valueChanged.connect(lambda _value:self.update_config_summary())
        for control in (self.formation,self.perception_source):
            control.currentIndexChanged.connect(lambda _index:self.update_config_summary())
        self.save_task_button.clicked.connect(lambda:self.save_task_preset())
        self.load_task_button.clicked.connect(lambda:self.load_task_preset())
        self.reset_task_button.clicked.connect(lambda:self.restore_task_defaults())
        self.export_report_button.clicked.connect(lambda:self.export_operator_report())
        self.pick_start.clicked.connect(lambda:self.begin_pick("start"));self.pick_goal.clicked.connect(lambda:self.begin_pick("goal"))
        self.start.clicked.connect(lambda:self.call("/fleet_mission_player/start"));self.pause.clicked.connect(lambda:self.call("/fleet_mission_player/pause"));self.resume.clicked.connect(lambda:self.call("/fleet_mission_player/resume"));self.reset.clicked.connect(lambda:self.call("/fleet_mission_player/reset"));self.land.clicked.connect(lambda:self.call("/fleet_mission_player/land"))
        self.preview_pub=rospy.Publisher("/operator/preview_markers",MarkerArray,queue_size=1,latch=True)
        self.runtime_marker_pubs=[rospy.Publisher(topic,MarkerArray,queue_size=1,latch=True) for topic in ("/fleet/markers","/dynamic_obstacles/markers")]
        self.ros_subscribers=[]
        self.ros_subscribers.append(rospy.Subscriber("/clicked_point",PointStamped,self.clicked_point_cb,queue_size=1))
        for spin in (self.start_x,self.start_y,self.goal_x,self.goal_y,self.altitude):spin.valueChanged.connect(lambda _value:self.parameters_changed())
        self.formation.currentIndexChanged.connect(lambda _index:self.parameters_changed())
        self.perception_source.setToolTip("停止仿真后选择，下一次启动生效；仅切换数据源不会重新生成航线。")
        self.ros_ui_bridge=RosUiBridge()
        self.ros_ui_bridge.state_received.connect(lambda msg:self.state_cb(msg),Qt.QueuedConnection)
        self.ros_ui_bridge.diagnostics_received.connect(lambda msg:self.diag_cb(msg),Qt.QueuedConnection)
        self.ros_ui_bridge.risk_received.connect(lambda msg:self.dynamic_risk_cb(msg),Qt.QueuedConnection)
        self.ros_ui_bridge.perception_received.connect(lambda msg:self.perception_status_cb(msg),Qt.QueuedConnection)
        self.ros_ui_bridge.energy_return_received.connect(lambda msg:self.energy_return_cb(msg),Qt.QueuedConnection)
        self.ros_ui_bridge.telemetry_received.connect(lambda msg:self.operator_snapshot_cb(msg),Qt.QueuedConnection)
        self.ros_subscribers.append(rospy.Subscriber("/fleet/mission_state",String,lambda msg:self.ros_ui_bridge.state_received.emit(msg),queue_size=1))
        self.ros_subscribers.append(rospy.Subscriber("/fleet/diagnostics",DiagnosticArray,lambda msg:self.ros_ui_bridge.diagnostics_received.emit(msg),queue_size=1))
        self.ros_subscribers.append(rospy.Subscriber("/fleet/dynamic_risk",String,lambda msg:self.ros_ui_bridge.risk_received.emit(msg),queue_size=1))
        self.ros_subscribers.append(rospy.Subscriber("/perception/status",String,lambda msg:self.ros_ui_bridge.perception_received.emit(msg),queue_size=1))
        self.ros_subscribers.append(rospy.Subscriber("/fleet/energy_return_advisory",String,lambda msg:self.ros_ui_bridge.energy_return_received.emit(msg),queue_size=1))
        self.ros_subscribers.append(rospy.Subscriber("/fleet/operator_snapshot",String,lambda msg:self.ros_ui_bridge.telemetry_received.emit(msg),queue_size=1))
        self.telemetry_last_rx=None;self.telemetry_stale_reported=False
        self.energy_last_rx=None;self.energy_stale_reported=False
        self.risk_last_rx=None;self.risk_stale_reported=False
        self.event_journal=OperatorEventJournal()
        self.latest_vehicle_snapshot=None;self.latest_risk_snapshot=None;self.latest_energy_snapshot=None
        self.telemetry_timer=QTimer(self.widget);self.telemetry_timer.setInterval(1000)
        self.telemetry_timer.timeout.connect(lambda:self.check_operator_snapshot_age());self.telemetry_timer.start()
        self.energy_timer=QTimer(self.widget);self.energy_timer.setInterval(500)
        self.energy_timer.timeout.connect(lambda:self.check_energy_advisory_age());self.energy_timer.start()
        self.risk_timer=QTimer(self.widget);self.risk_timer.setInterval(500)
        self.risk_timer.timeout.connect(lambda:self.check_dynamic_risk_age());self.risk_timer.start()
        self.update_defaults()
        self.update_algorithm_controls()
    def update_defaults(self):
        self.clear_runtime_markers()
        sx,sy,gx,gy,alt=SCENE_DEFAULTS[self.scene.currentData()]
        for widget,value in ((self.start_x,sx),(self.start_y,sy),(self.goal_x,gx),(self.goal_y,gy),(self.altitude,alt)):widget.setValue(value)
        QTimer.singleShot(50,lambda:self.publish_preview());self.schedule_analysis();self.update_config_summary()
    def update_algorithm_controls(self,_value=None):
        algorithm=self.avoidance_mode.currentData()[0]
        editable=(algorithm=='orca3d' and self.dynamic_enabled.isChecked() and
                  not getattr(self,'simulation_start_pending',False) and
                  (not hasattr(self,'process') or self.process.state()==QProcess.NotRunning))
        self.orca_max_speed.setEnabled(editable)
        self.orca_timeout.setEnabled(editable)
        self.update_config_summary()
    def configuration_provenance(self):
        scene=int(self.scene.currentData())
        if scene not in self.scene_identity_cache:
            self.scene_identity_cache[scene]=scene_identity(self.package_path,scene)
        algorithm=self.avoidance_mode.currentData()[0]
        return {'source':dict(self.build_identity),
                'scene':dict(self.scene_identity_cache[scene]),
                'algorithm':algorithm_profile(algorithm)}
    def update_config_summary(self,_value=None):
        identity=self.configuration_provenance()
        source=identity['source'];scene=identity['scene'];profile=identity['algorithm']
        commit=source['git_commit']+('（工作区未提交）' if source['git_dirty'] else '')
        algorithm=self.avoidance_mode.currentData()[0]
        mode=(profile['label'] if self.dynamic_enabled.isChecked() else
              '动态障碍已关闭，所选算法不会运行')
        parameters=("；ORCA 限速 {:.1f} m/s、超时 {:.2f} s".format(
            self.orca_max_speed.value(),self.orca_timeout.value()) if algorithm=='orca3d' else '')
        self.config_summary.setText(
            "Git {} · 包 {} · 场景 {} SHA-256 {}\n"
            "下次启动：{}（{}）{}；感知 {}；队形 {}；高度 {:.1f} m；起点 ({:.1f},{:.1f}) → 终点 ({:.1f},{:.1f})\n{}".format(
                commit,source['package_version'],scene['scene_id'],scene['world_sha256'][:12],
                mode,profile['contract'],parameters,self.perception_source.currentData(),
                self.formation.currentData(),self.altitude.value(),self.start_x.value(),
                self.start_y.value(),self.goal_x.value(),self.goal_y.value(),profile['scope']))
    def task_configuration(self):
        return validate_task({'schema':1,'scene_id':int(self.scene.currentData()),
            'start_m':[self.start_x.value(),self.start_y.value()],
            'goal_m':[self.goal_x.value(),self.goal_y.value()],
            'altitude_m':self.altitude.value(),'formation':str(self.formation.currentData()),
            'dynamic_obstacles':self.dynamic_enabled.isChecked(),
            'avoidance_mode':self.avoidance_mode.currentData()[0],
            'perception_source':str(self.perception_source.currentData()),
            'orca_max_speed_mps':self.orca_max_speed.value(),
            'orca_command_timeout_s':self.orca_timeout.value()})
    def task_configuration_locked(self):
        return self.process.state()!=QProcess.NotRunning or bool(self.active_runtime_processes())
    def apply_task_configuration(self,task):
        task=validate_task(task)
        controls=(self.scene,self.start_x,self.start_y,self.goal_x,self.goal_y,
                  self.altitude,self.formation,self.dynamic_enabled,
                  self.avoidance_mode,self.perception_source,
                  self.orca_max_speed,self.orca_timeout)
        prior=[control.blockSignals(True) for control in controls]
        try:
            self.scene.setCurrentIndex(task['scene_id'])
            for control,value in ((self.start_x,task['start_m'][0]),
                                  (self.start_y,task['start_m'][1]),
                                  (self.goal_x,task['goal_m'][0]),
                                  (self.goal_y,task['goal_m'][1]),
                                  (self.altitude,task['altitude_m'])):control.setValue(value)
            self.formation.setCurrentIndex(self.formation.findData(task['formation']))
            self.dynamic_enabled.setChecked(task['dynamic_obstacles'])
            for index in range(self.avoidance_mode.count()):
                if self.avoidance_mode.itemData(index)[0]==task['avoidance_mode']:
                    self.avoidance_mode.setCurrentIndex(index);break
            self.perception_source.setCurrentIndex(self.perception_source.findData(task['perception_source']))
            self.orca_max_speed.setValue(task['orca_max_speed_mps'])
            self.orca_timeout.setValue(task['orca_command_timeout_s'])
        finally:
            for control,was_blocked in zip(controls,prior):control.blockSignals(was_blocked)
        self.clear_runtime_markers();self.publish_preview();self.schedule_analysis()
        self.update_algorithm_controls()
    def save_task_preset(self):
        path,_=QFileDialog.getSaveFileName(self.widget,"保存任务参数","mission_preset.json","JSON 文件 (*.json)")
        if not path:return
        try:write_json_atomic(path,self.task_configuration())
        except (OSError,TypeError,ValueError) as exc:
            QMessageBox.warning(self.widget,"保存失败",str(exc));return
        self.state.setText("任务参数已保存；这不是规划验收文件")
    def load_task_preset(self):
        if self.task_configuration_locked():
            QMessageBox.warning(self.widget,"任务正在运行","请先停止仿真，再加载任务参数。")
            return
        path,_=QFileDialog.getOpenFileName(self.widget,"加载任务参数","","JSON 文件 (*.json)")
        if not path:return
        try:self.apply_task_configuration(load_task(path))
        except (OSError,TypeError,ValueError) as exc:
            QMessageBox.warning(self.widget,"加载失败",str(exc));return
        self.state.setText("任务参数已加载；等待重新规划")
    def restore_task_defaults(self):
        if self.task_configuration_locked():
            QMessageBox.warning(self.widget,"任务正在运行","请先停止仿真，再恢复默认参数。")
            return
        sx,sy,gx,gy,alt=SCENE_DEFAULTS[0]
        self.apply_task_configuration({'schema':1,'scene_id':0,'start_m':[sx,sy],
            'goal_m':[gx,gy],'altitude_m':alt,'formation':'triangle',
            'dynamic_obstacles':True,'avoidance_mode':'collective_offset',
            'perception_source':'perception',
            'orca_max_speed_mps':2.0,'orca_command_timeout_s':0.6})
        self.state.setText("已恢复默认参数；等待重新规划")
    def export_operator_report(self):
        path,_=QFileDialog.getSaveFileName(self.widget,"导出上位机界面报告",
                                           "operator_snapshot.json","JSON 文件 (*.json)")
        if not path:return
        try:
            planning={'approved':bool(self.valid_analysis_signature==self.parameter_signature() and
                                      self.analysis_mission and os.path.isfile(self.analysis_mission) and
                                      self.analysis_report and os.path.isfile(self.analysis_report)),
                      'status':self.analysis_state.text(),'detail':self.analysis_detail.text()}
            report=build_operator_report(self.task_configuration(),planning,
                self.latest_vehicle_snapshot,self.latest_risk_snapshot,
                self.latest_energy_snapshot,self.event_journal.rows)
            report['provenance']=self.configuration_provenance()
            write_json_atomic(path,report)
        except (OSError,TypeError,ValueError) as exc:
            QMessageBox.warning(self.widget,"导出失败",str(exc));return
        self.state.setText("界面报告已导出；不代替飞行安全审计")
    def parameter_signature(self):
        return (int(self.scene.currentData()),round(self.start_x.value(),3),
                round(self.start_y.value(),3),round(self.goal_x.value(),3),
                round(self.goal_y.value(),3),round(self.altitude.value(),3),
                str(self.formation.currentData()))
    def parameters_changed(self,_value=None):
        self.publish_preview();self.schedule_analysis()
    def schedule_analysis(self):
        if not hasattr(self,"analysis_timer"):return
        self.valid_analysis_signature=None;self.analysis_mission=None;self.analysis_report=None;self.analysis_retry=0
        self.start_sim.setEnabled(False);self.analysis_state.setText("参数已变化，等待重新分析…")
        self.analysis_state.setStyleSheet("color:#ffb74d;border-color:#8a6530;")
        self.analysis_detail.setText("旧规划已失效；停止调整参数后将自动运行 OMPL、净空复核和 TOPPRA。")
        self.analysis_timer.start()
    def analysis_command(self,mission,report):
        pkg=rospkg.RosPack().get_path("logistics_gazebo_sim")
        return [os.path.join(pkg,"scripts","generate_missions"),
                "--scene",str(self.scene.currentData()),
                "--start-x",str(self.start_x.value()),"--start-y",str(self.start_y.value()),
                "--goal-x",str(self.goal_x.value()),"--goal-y",str(self.goal_y.value()),
                "--altitude",str(self.altitude.value()),
                "--formation",str(self.formation.currentData()),
                "--output",mission,"--report-json",report]
    def start_analysis(self):
        if self.analysis_process.state()!=QProcess.NotRunning:
            self.analysis_running_signature=None;self.analysis_process.kill();self.analysis_process.waitForFinished(1000)
        signature=self.parameter_signature()
        mission="/tmp/logistics_analysis_{}.yaml".format(os.getpid())
        report="/tmp/logistics_analysis_{}.json".format(os.getpid())
        for path in (mission,report):
            try:
                if os.path.isfile(path):os.unlink(path)
            except OSError:pass
        command=self.analysis_command(mission,report)
        self.analysis_running_signature=signature;self.analysis_mission=mission;self.analysis_report=report
        self.analysis_state.setText("正在进行三维规划与净空分析…")
        self.analysis_state.setStyleSheet("color:#64b5f6;border-color:#315f83;")
        self.analysis_detail.setText("当前参数：场景{}，起点({:.1f},{:.1f})，终点({:.1f},{:.1f})，高度{:.1f}m，队形{}".format(
            signature[0],signature[1],signature[2],signature[3],signature[4],signature[5],signature[6]))
        self.start_sim.setEnabled(False)
        self.analysis_process.start(command[0],command[1:]);self.analysis_timeout.start()
    def analysis_timed_out(self):
        if self.analysis_process.state()==QProcess.NotRunning:return
        self.analysis_running_signature=None;self.analysis_process.kill()
        self.valid_analysis_signature=None;self.start_sim.setEnabled(False)
        self.analysis_state.setText("分析超时")
        self.analysis_state.setStyleSheet("color:#ef5350;border-color:#8d3434;")
        self.analysis_detail.setText("规划超过20秒，请调整起终点、高度或队形后重试。")
        self.record_planning_event("TIMEOUT")
    def analysis_finished(self,exit_code,_exit_status):
        self.analysis_timeout.stop()
        signature=self.analysis_running_signature;self.analysis_running_signature=None
        if signature is None or signature!=self.parameter_signature():return
        stdout=bytes(self.analysis_process.readAllStandardOutput()).decode("utf-8","replace").strip()
        stderr=bytes(self.analysis_process.readAllStandardError()).decode("utf-8","replace").strip()
        if exit_code or not self.analysis_report or not os.path.isfile(self.analysis_report) or not os.path.isfile(self.analysis_mission):
            detail=stderr or stdout or "规划器未生成分析报告";diagnostic=None
            if self.analysis_report and os.path.isfile(self.analysis_report):
                try:
                    with open(self.analysis_report,"r",encoding="utf-8") as stream:
                        diagnostic=json.load(stream).get("diagnostic")
                except (OSError,ValueError):diagnostic=None
            environment_markers=("object is not callable","object is not iterable",
                                 "keywords must be strings","Parameter' object",
                                 "unsupported operand type","SafeDumper","SafeLoader",
                                 "has no attribute 'nodeType'","not supported between instances of")
            is_environment=(diagnostic and diagnostic.get("category")=="ENVIRONMENT") or any(marker in detail for marker in environment_markers)
            if self.analysis_retry<3 and is_environment:
                self.analysis_retry+=1;self.analysis_state.setText("环境异常，正在自动重试…")
                self.analysis_detail.setText(detail[:320]);QTimer.singleShot(150,lambda:self.start_analysis());return
            self.valid_analysis_signature=None;self.start_sim.setEnabled(False)
            if diagnostic:
                names={"INPUT":"输入无效","FEASIBILITY":"任务不可行",
                       "PLANNING":"当前时间内未找到路径","DYNAMICS":"动力学轨迹不可行",
                       "ENVIRONMENT":"运行环境异常","INTERNAL":"规划系统异常"}
                self.analysis_state.setText(names.get(diagnostic.get("category"),"当前参数不可执行"))
                suggestion_names={"select_valid_scene":"重新选择场景","adjust_altitude":"调整高度","move_start_or_goal":"移动起点或终点","move_start":"移动起点","move_goal":"移动终点","use_compact_formation":"改用紧凑队形","use_alternate_landing_site":"选择备用降落点","increase_altitude":"提高高度","use_flat_formation":"改用平面队形","use_column":"改用纵向一字","use_vertical_formation":"改用垂直错层","replan_path":"重新规划","retry":"重试","increase_planning_time":"增加规划时间","adjust_route":"调整路线","change_formation":"更换队形","adjust_spacing":"调整间距","reduce_speed":"降低速度","reduce_fleet_size":"减少无人机数量或分组","move_transition_area":"调整队形变换区域","rebuild_workspace":"重新构建工作空间","check_installation":"检查安装","inspect_planner_log":"检查规划日志","inspect_environment":"检查运行环境","inspect_log":"检查日志"}
                suggestions="、".join(suggestion_names.get(item,item) for item in (diagnostic.get("suggestions") or []))
                message=diagnostic.get("message","规划失败")
                extra=diagnostic.get("detail","")
                context=diagnostic.get("context") or {};location=context.get("location");obstacle=context.get("obstacle")
                where=("\n位置：{}".format(location) if location else "")+("，障碍物：{}".format(obstacle) if obstacle else "")
                self.analysis_detail.setText((message+where+("\n建议："+suggestions if suggestions else "")+
                                              ("\n详情："+extra[:320] if extra and extra!=message else ""))[:700])
            else:
                self.analysis_state.setText("当前参数不可执行")
                self.analysis_detail.setText(self.planning_error(detail)[:700])
            self.analysis_state.setStyleSheet("color:#ef5350;border-color:#8d3434;")
            self.record_planning_event(diagnostic.get("category") if diagnostic else "UNKNOWN")
            return
        try:
            with open(self.analysis_report,"r",encoding="utf-8") as stream:report=json.load(stream)
            clearance=report["clearance_analysis"];trajectory=report["trajectory_parameterization"];stages=report["stages"];phases=report.get("phase_analysis",{});formation_schedule=report.get("formation_schedule",{})
            available=float(clearance["minimum_horizontal_clearance_m"]);required=float(clearance["required"]["horizontal_m"]);margin=available-required
            location=clearance.get("critical_location",["-","-","-"]);obstacle=clearance.get("critical_obstacle") or "世界边界"
            self.analysis_state.setText("规划可行 · 净空安全余量 {:.2f} m".format(margin))
            color="#66bb6a" if margin>=.5 else "#ffb74d";border="#376c3a" if margin>=.5 else "#8a6530"
            self.analysis_state.setStyleSheet("color:{};border-color:{};".format(color,border))
            self.analysis_detail.setText(
                "最小水平净空 {:.2f} m / 所需 {:.2f} m；危险对象：{}，位置 ({:.1f}, {:.1f}, {:.1f})\n"
                "地板余量 {:.2f} m，顶部余量 {:.2f} m；预计出航 {:.1f} s，最大速度 {:.2f} m/s，最大加速度 {:.2f} m/s²\n"
                "起飞/巡航/投递/返航共 {} 项检查通过；{} 次队形变换将启用安全距离缩放\n"
                "自动路径队形切换 {} 次；候选采样 {}".format(
                    available,required,obstacle,float(location[0]),float(location[1]),float(location[2]),
                    float(clearance["minimum_floor_clearance_m"]),float(clearance["minimum_ceiling_clearance_m"]),
                    float(stages["outbound_end"])-18.0,float(trajectory["actual_max_speed_mps"]),
                    float(trajectory["actual_max_acceleration_mps2"]),len(phases),
                    sum(1 for value in phases.values() if value.get("safety_scaling_required")),
                    len(formation_schedule.get("switches",[])),formation_schedule.get("formation_sample_counts",{})))
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.valid_analysis_signature=None;self.start_sim.setEnabled(False)
            self.analysis_state.setText("分析报告无效")
            self.analysis_state.setStyleSheet("color:#ef5350;border-color:#8d3434;")
            self.analysis_detail.setText("无法读取规划摘要：{}".format(exc))
            self.record_planning_event("INTERNAL");return
        self.valid_analysis_signature=signature;self.start_sim.setEnabled(True)
    def active_runtime_processes(self):
        result=[]
        for name in os.listdir("/proc"):
            if not name.isdigit():continue
            try:
                with open("/proc/{}/stat".format(name),"r") as stream:state=stream.read().split()[2]
                with open("/proc/{}/cmdline".format(name),"rb") as stream:command=stream.read().replace(b"\0",b" ").decode("utf-8","ignore")
            except (OSError,IndexError):continue
            if state!="Z" and any(token in command for token in ("three_uav_mission.launch","/px4 ","px4 -i")):
                result.append((int(name),command.strip()))
        return result
    def preflight_errors(self):
        errors=[]
        if not self.analysis_mission or not os.path.isfile(self.analysis_mission) or os.path.getsize(self.analysis_mission)<100:
            errors.append("已验证任务文件缺失或为空")
        if not os.access("/tmp",os.W_OK):errors.append("/tmp 不可写")
        try:
            if shutil.disk_usage("/tmp").free<100*1024*1024:errors.append("临时磁盘剩余空间不足100 MB")
        except OSError:errors.append("无法检查临时磁盘")
        try:rospy.get_master().getPid()
        except Exception:errors.append("ROS master不可用")
        probe=socket.socket(socket.AF_INET,socket.SOCK_STREAM);probe.settimeout(.15)
        try:
            if probe.connect_ex(("127.0.0.1",11460))==0:errors.append("Gazebo端口11460已被占用")
        finally:probe.close()
        return errors
    def begin_pick(self,mode):
        self.pick_mode=mode;self.state.setText("请在 RViz 工具栏选择 Publish Point，然后点击地图")
    def clicked_point_cb(self,msg):
        if self.pick_mode is None:self.pick_mode="start"
        self.point_bridge.point_received.emit(float(msg.point.x),float(msg.point.y))
    def apply_clicked_point(self,x,y):
        if not(-46.0<=x<=46.0 and -46.0<=y<=46.0):
            QMessageBox.warning(self.widget,"选点无效","请在 -46~46 m 范围内点选");return
        if self.pick_mode=="start":
            self.start_x.setValue(x);self.start_y.setValue(y);self.pick_mode="goal";self.state.setText("已设置起点，请继续在 RViz 点击终点")
        else:
            self.goal_x.setValue(x);self.goal_y.setValue(y);self.pick_mode=None;self.state.setText("已设置终点，可以开始规划")
        self.publish_preview()
    def new_marker(self,mid,kind,ns):
        m=Marker();m.header.frame_id="world";m.header.stamp=rospy.Time.now();m.ns=ns;m.id=mid;m.type=kind;m.action=Marker.ADD;m.pose.orientation.w=1.0;return m
    def clear_runtime_markers(self):
        if not hasattr(self,"runtime_marker_pubs"):return
        marker=self.new_marker(0,Marker.CUBE,"operator_cleanup");marker.action=Marker.DELETEALL;message=MarkerArray(markers=[marker])
        for publisher in self.runtime_marker_pubs:publisher.publish(message)
    def publish_preview(self):
        if not hasattr(self,"preview_pub"):return
        arr=MarkerArray();clear=self.new_marker(0,Marker.CUBE,"clear");clear.action=Marker.DELETEALL;arr.markers.append(clear);mid=100
        for o in SCENES[self.scene.currentData()]["obstacles"]:
            if o["kind"]=="box":parts=[(o["x"],o["y"],o["w"],o["d"],Marker.CUBE)]
            elif o["kind"]=="cylinder":parts=[(o["x"],o["y"],o["radius"]*2,o["radius"]*2,Marker.CYLINDER)]
            else:parts=[(x,y,w,d,Marker.CUBE) for x,y,w,d in o["rects"]]
            for x,y,w,d,kind in parts:
                m=self.new_marker(mid,kind,"preview_obstacles");mid+=1;cx,cy=metric_xy((x,y)) if kind==Marker.CYLINDER else metric_xy((x+w/2.0,y+d/2.0));m.pose.position.x,m.pose.position.y,m.pose.position.z=cx,cy,o["height"]/2.0;m.scale.x,m.scale.y,m.scale.z=w*SCALE,d*SCALE,o["height"];m.color.r,m.color.g,m.color.b,m.color.a=.45,.48,.52,.65;arr.markers.append(m)
        for mid,x,y,color,label in ((1,self.start_x.value(),self.start_y.value(),(0.1,1.0,0.2),"START"),(2,self.goal_x.value(),self.goal_y.value(),(1.0,0.15,0.1),"GOAL")):
            m=self.new_marker(mid,Marker.CYLINDER,"selection");m.pose.position=Point(x,y,.15);m.scale.x=m.scale.y=1.5;m.scale.z=.3;m.color.r,m.color.g,m.color.b,m.color.a=color[0],color[1],color[2],.95;arr.markers.append(m);t=self.new_marker(mid+10,Marker.TEXT_VIEW_FACING,"selection_labels");t.pose.position=Point(x,y,1.4);t.scale.z=1.0;t.text=label;t.color.r,t.color.g,t.color.b,t.color.a=color[0],color[1],color[2],1.0;arr.markers.append(t)
        self.preview_pub.publish(arr)

    def cleanup_px4_sockets(self):
        if QProcess.execute("pgrep",["-x","px4"])==1:
            for i in range(3):
                path="/tmp/px4-sock-{}".format(i)
                try:
                    if os.path.exists(path):os.unlink(path)
                except OSError as exc:rospy.logwarn("Cannot remove stale PX4 socket %s: %s",path,exc)
    def planning_error(self,text):
        mapping={"E_ALTITUDE":"\u98de\u884c\u9ad8\u5ea6\u5fc5\u987b\u5728 3\uff5e45 m","E_BOUNDARY":"起点、终点或编队包络超出安全边界","E_VERTICAL_CLEARANCE":"当前高度无法容纳所选三维队形，请调整高度或改用平面队形","E_CORRIDOR_TOO_NARROW":"路径局部净空不足，无法容纳完整编队；请提高高度、改用纵队/垂直错层或重新规划","E_FORMATION":"队形参数无效，请检查队形、无人机数量和间距","E_BLOCKED":"\u8d77\u70b9\u6216\u7ec8\u70b9\u4f4d\u4e8e\u969c\u788d\u7269\u5b89\u5168\u533a\u5185\uff0c\u8bf7\u8c03\u6574\u5750\u6807\u6216\u9ad8\u5ea6","E_DISTANCE":"\u8d77\u70b9\u4e0e\u7ec8\u70b9\u8ddd\u79bb\u5fc5\u987b\u81f3\u5c11\u4e3a 3 m","E_NO_PATH":"\u5f53\u524d\u9ad8\u5ea6\u4e0e\u961f\u5f62\u4e0b\u89c4\u5212\u4e0d\u51fa\u5b89\u5168\u8def\u5f84\uff0c\u8bf7\u63d0\u9ad8\u9ad8\u5ea6\u6216\u8c03\u6574\u8d77\u7ec8\u70b9","E_SCENE":"\u573a\u666f\u7f16\u53f7\u65e0\u6548","E_OMPL":"ROS OMPL 三维规划失败，请调整起终点、高度或场景","E_TOPPRA":"B\u6837\u6761\u6216 TOPPRA \u65f6\u95f4\u53c2\u6570\u5316\u5931\u8d25\uff0c\u8bf7\u8c03\u6574\u8d77\u7ec8\u70b9\u6216\u9ad8\u5ea6"}
        for code,message in mapping.items():
            if code in text:return message+"\n\n\u89c4\u5212\u5668\u4fe1\u606f\uff1a"+text
        return "\u822a\u7ebf\u89c4\u5212\u5931\u8d25\uff1a\n"+text
    def simulation_launch_args(self,mission):
        algorithm=self.avoidance_mode.currentData()[0]
        profile=algorithm_profile(algorithm)
        execution=profile['execution'] and self.dynamic_enabled.isChecked()
        return ["logistics_gazebo_sim","three_uav_mission.launch","gui:=true","auto_start:=false",
              "dynamic_obstacles:={}".format(str(self.dynamic_enabled.isChecked()).lower()),
              "dynamic_state_source:={}".format(self.perception_source.currentData()),
              "dynamic_avoidance_execution:={}".format(str(execution).lower()),
              "local_avoidance_algorithm:={}".format(algorithm),
              "orca_control_mode:={}".format(profile['orca_mode']),
              "orca_max_speed_mps:={:.1f}".format(self.orca_max_speed.value()),
              "orca_command_timeout_s:={:.2f}".format(self.orca_timeout.value()),
              "scene_id:={}".format(self.scene.currentData()),
              "spawn_x:={}".format(self.start_x.value()),"spawn_y:={}".format(self.start_y.value()),
              "goal_x:={}".format(self.goal_x.value()),"goal_y:={}".format(self.goal_y.value()),
              "target_z:={}".format(self.altitude.value()),
              "mission_config:={}".format(mission),"gazebo_master_uri:=http://127.0.0.1:11460"]
    def launch_sim(self):
        if self.process.state()!=QProcess.NotRunning:QMessageBox.information(self.widget,"\u63d0\u793a","\u4eff\u771f\u5df2\u7ecf\u5728\u8fd0\u884c");return
        active=self.active_runtime_processes()
        if active:
            pids=",".join(str(item[0]) for item in active)
            self.state.setText("三机仿真已在运行")
            QMessageBox.information(self.widget,"仿真已在运行",
                "检测到正在运行的三机仿真（进程 {}），无需重复启动。\n可直接使用任务控制按钮。".format(pids))
            return
        if self.valid_analysis_signature!=self.parameter_signature() or not self.analysis_mission or not os.path.isfile(self.analysis_mission):
            QMessageBox.warning(self.widget,"规划尚未就绪","当前参数还没有通过三维规划与净空分析，请等待分析完成或调整参数。");self.schedule_analysis();return
        preflight=self.preflight_errors()
        if preflight:
            QMessageBox.critical(self.widget,"启动条件不满足","启动前检查失败：\n- "+"\n- ".join(preflight));return
        mission=self.analysis_mission
        self.clear_runtime_markers();self.cleanup_px4_sockets()
        args=self.simulation_launch_args(mission)
        self.simulation_stop_requested=False;self.simulation_start_pending=True;self.start_sim.setEnabled(False)
        self.set_sensor_controls_enabled(False)
        self.state.setText("规划成功，正在启动 Gazebo 与三机 PX4…")
        px4_root=os.path.expanduser("~/PX4_Firmware")
        environment=QProcessEnvironment.systemEnvironment()
        additions={
            "ROS_PACKAGE_PATH":[px4_root,os.path.join(px4_root,"Tools","sitl_gazebo")],
            "GAZEBO_PLUGIN_PATH":[os.path.join(px4_root,"build","px4_sitl_default","build_gazebo")],
            "GAZEBO_MODEL_PATH":[os.path.join(px4_root,"Tools","sitl_gazebo","models")],
            "LD_LIBRARY_PATH":[os.path.join(px4_root,"build","px4_sitl_default","build_gazebo")]}
        for name,paths in additions.items():
            current=environment.value(name)
            environment.insert(name,":".join(([current] if current else [])+paths))
        self.process.setProcessEnvironment(environment)
        self.process.start("setsid",["roslaunch"]+args)
    def drain_simulation_output(self):
        self.simulation_log_tail=(self.simulation_log_tail+bytes(self.process.readAllStandardOutput()))[-32768:]
    def simulation_process_started(self):
        self.state.setText("启动命令已提交，正在等待 Gazebo 与三机 PX4 就绪…")
    def simulation_process_error(self,_error):
        self.simulation_start_pending=False
        self.set_sensor_controls_enabled(True)
        self.start_sim.setEnabled(self.valid_analysis_signature==self.parameter_signature())
        self.state.setText("仿真启动失败")
        QMessageBox.critical(self.widget,"仿真启动失败",
            "无法启动 roslaunch：{}\n请检查 ROS 环境和启动日志。".format(self.process.errorString()))
    def simulation_process_finished(self,exit_code,_exit_status):
        self.simulation_start_pending=False
        self.set_sensor_controls_enabled(True)
        self.clear_runtime_markers()
        self.start_sim.setEnabled(self.valid_analysis_signature==self.parameter_signature())
        if self.simulation_stop_requested:
            self.state.setText("三机仿真已停止")
        elif exit_code!=0:
            self.state.setText("仿真异常退出（代码 {}）".format(exit_code))
            QMessageBox.warning(self.widget,"仿真异常退出",
                "Gazebo/PX4 启动进程已退出，返回代码 {}。\n请查看 roslaunch 日志。".format(exit_code))
        else:self.state.setText("三机仿真已结束")
        self.simulation_stop_requested=False
    def set_sensor_controls_enabled(self,enabled):
        for control in (self.perception_source,self.avoidance_mode,self.dynamic_enabled):
            control.setEnabled(enabled)
        self.update_algorithm_controls()

    def stop_simulation(self):
        if self.process.state()==QProcess.NotRunning:
            result=QProcess.execute("pkill",["-INT","-f",
                "[r]oslaunch.*three_uav_mission.launch"])
            if result==0:
                self.state.setText("正在停止外部三机仿真…")
                QTimer.singleShot(5000,lambda:self.force_stop_external_simulation())
            else:
                self.state.setText("没有检测到运行中的三机仿真")
            return
        self.simulation_stop_requested=True;self.state.setText("正在停止三机仿真…")
        try:os.killpg(int(self.process.processId()),signal.SIGINT)
        except (OSError,ProcessLookupError):self.process.terminate()
        QTimer.singleShot(5000,lambda:self.force_stop_simulation())
    def force_stop_simulation(self):
        if self.process.state()==QProcess.NotRunning:return
        try:os.killpg(int(self.process.processId()),signal.SIGTERM)
        except (OSError,ProcessLookupError):self.process.kill()
    def force_stop_external_simulation(self):
        if QProcess.execute("pgrep",["-f",
                "[r]oslaunch.*three_uav_mission.launch"])==0:
            QProcess.execute("pkill",["-TERM","-f",
                "[r]oslaunch.*three_uav_mission.launch"])
        else:self.clear_runtime_markers();self.state.setText("三机仿真已停止")
    def call(self,name):
        try:r=rospy.ServiceProxy(name,Trigger)();self.state.setText(r.message)
        except rospy.ServiceException as exc:QMessageBox.warning(self.widget,"\u670d\u52a1\u8c03\u7528\u5931\u8d25",str(exc))
    def render_events(self):
        rows=self.event_journal.rows
        self.event_summary.setText("最近 {} 条事件 · 状态变化去重 · 仅供操作员判断".format(len(rows)))
        self.event_table.setRowCount(len(rows))
        colors={"ERROR":"#ef5350","WARN":"#ffb74d","INFO":"#66bb6a"}
        for index,event in enumerate(rows):
            values=[time.strftime("%H:%M:%S",time.localtime(event['time'])),
                    event['source'],event['title'],event['guidance']]
            for column,value in enumerate(values):
                cell=QTableWidgetItem(value)
                if column==2:cell.setForeground(QColor(colors[event['level']]))
                if column==3:cell.setToolTip(value)
                self.event_table.setItem(index,column,cell)
    def record_planning_event(self,category):
        if self.event_journal.planning_failure(category,self.analysis_state.text(),
                                               self.analysis_detail.text(),time.time()):
            self.render_events()
    def state_cb(self,msg):
        try:
            d=json.loads(msg.data)
            if self.simulation_start_pending:
                self.simulation_start_pending=False
                QMessageBox.information(self.widget,"仿真启动成功",
                    "Gazebo、三机 PX4 与任务节点已连接，可以开始任务。")
            self.state.setText(d["state"]);ph={"TAKEOFF_HOLD":"\u8d77\u98de\u7b49\u5f85","DEPARTURE_FORMATION":"\u51fa\u53d1\u7f16\u961f","OUTBOUND":"\u524d\u5f80\u914d\u9001\u70b9","DELIVERY_FORMATION":"\u6295\u9012\u4e00\u5b57\u7f16\u961f","DELIVERY_DESCENT":"\u6295\u9012\u4e0b\u964d","DELIVERY_RELEASE":"\u8d27\u7269\u6295\u9012","DELIVERY_ASCENT":"\u6295\u9012\u56de\u5347","CRUISE_REFORMATION":"\u6062\u590d\u5de1\u822a\u961f\u5f62","RETURN":"\u8fd4\u822a","HOME_FORMATION":"\u8fd4\u822a\u7f16\u961f","HOME_DESCENT":"\u8d77\u70b9\u964d\u843d"};self.stage.setText("{} - {}".format(d["stage"],ph.get(d.get("phase"),"-")))
            self.progress.setValue(int(1000*d["progress"]))
            if self.event_journal.mission(d,time.time()):self.render_events()
        except Exception:pass
    def diag_cb(self,msg):
        if not msg.status:return
        s=msg.status[0];v={x.key:x.value for x in s.values};self.safety.setText("{} | \u95f4\u8ddd {}m | \u51c0\u7a7a {}m | \u8bef\u5dee {}m".format(s.message,v.get("min_separation_m","-"),v.get("min_obstacle_clearance_m","-"),v.get("max_tracking_error_m","-")));self.safety.setStyleSheet("color:{}".format("#c62828" if s.level>=2 else "#ef6c00" if s.level==1 else "#2e7d32"))
        if self.event_journal.diagnostic(int(s.level),s.message,time.time()):self.render_events()
    def dynamic_risk_cb(self,msg):
        try:
            report=validate_risk_report(json.loads(msg.data))
        except (TypeError,ValueError):
            self.risk_last_rx=None;self.risk_stale_reported=True
            self.latest_risk_snapshot=None
            self.dynamic_risk.setText("动态风险数据无效，状态未知")
            self.risk_summary.setText("风险报告无效，旧预测已隐藏")
            self.prediction_summary.setText("轨迹报告无效，旧预测已隐藏")
            self.risk_table.setRowCount(0);self.prediction_table.setRowCount(0)
            if self.event_journal.risk('STALE',time.time()):self.render_events()
            return
        self.risk_last_rx=time.monotonic();self.risk_stale_reported=False
        level=report['level'];rows=report['rows']
        self.latest_risk_snapshot=report
        if self.event_journal.risk(level,time.time()):self.render_events()
        level_names={"SAFE":"安全","WARNING":"警告","CRITICAL":"严重","STALE":"数据过期"}
        text="{} | 动态障碍 {} 个".format(level_names[level],report['obstacle_count'])
        algorithm=report['algorithm']
        if algorithm is not None:
            text+=" | {}".format({'collective_offset':'整队避障','orca3d':'ORCA',
                                     'distributed_mpc':'MPC'}[algorithm])
        if level in ('WARNING','CRITICAL') and report['plan_viable'] is not None:
            text+=" | {}".format('有候选建议' if report['plan_viable'] else '无可用建议，查看安全状态')
        self.dynamic_risk.setText(text)
        self.dynamic_risk.setStyleSheet("color:{}".format({"CRITICAL":"#c62828","WARNING":"#ef6c00","SAFE":"#2e7d32"}.get(level,"#607d8b")))
        if level=='STALE':
            self.risk_summary.setText("动态障碍或机体状态过期，冲突预测不可用")
            self.prediction_summary.setText("动态数据过期，轨迹预测不可用")
            self.risk_table.setRowCount(0);self.prediction_table.setRowCount(0);return
        pair=report['closest_pair'];separation=report['minimum_separation_m']
        summary="{} 架预测 · {} 个动态障碍".format(len(rows),report['obstacle_count'])
        if pair is not None and separation is not None:
            summary+=" | 最近机间距 {}–{}：{:.2f} m".format(pair[0],pair[1],separation)
        if report['separation_ttc_s'] is not None:
            summary+=" | 机间冲突倒计时 {:.1f} s".format(report['separation_ttc_s'])
        if level=='CRITICAL' and not rows:summary+=" | 风险计算或机间安全异常，请看任务安全状态"
        self.risk_summary.setText(summary)
        self.risk_table.setRowCount(len(rows))
        colors={"SAFE":"#66bb6a","WARNING":"#ffb74d","CRITICAL":"#ef5350"}
        for index,row in enumerate(rows):
            point=row['closest_position_m']
            values=[row['vehicle_id'],level_names[row['level']],row['obstacle_id'] or "-",
                    "{:.2f} m".format(row['clearance_m']) if row['clearance_m'] is not None else "-",
                    "{:.1f} s".format(row['time_to_conflict_s']) if row['time_to_conflict_s'] is not None else "无",
                    "({:.1f}, {:.1f}, {:.1f})".format(*point) if point is not None else "-"]
            for column,value in enumerate(values):
                cell=QTableWidgetItem(value)
                if column==1:cell.setForeground(QColor(colors[row['level']]))
                if column==5 and row['closest_time_s'] is not None:
                    cell.setToolTip("预测 {:.1f} s 后的最近点；不是实测位置".format(row['closest_time_s']))
                self.risk_table.setItem(index,column,cell)
        tracks=report['prediction_tracks']
        if tracks is None:
            self.prediction_summary.setText("未检测到动态障碍，无需轨迹外推" if report['obstacle_count']==0
                                            else "风险报告未提供障碍物轨迹")
            self.prediction_table.setRowCount(0)
        else:
            self.prediction_summary.setText("世界坐标 · 恒速外推 {} 条轨迹{}；不是实测未来位置".format(
                len(tracks),"，另有 {} 条未显示".format(report['prediction_truncated'])
                if report['prediction_truncated'] else ""))
            self.prediction_table.setRowCount(len(tracks))
            if tracks:
                sample_times=[tracks[0]['samples'][index][0] for index in (1,2)]
                self.prediction_table.setHorizontalHeaderLabels(
                    ["障碍物","当前估计","+{:.1f} s".format(sample_times[0]),
                     "+{:.1f} s".format(sample_times[1])])
            for index,track in enumerate(tracks):
                values=[track['id']+("（遮挡估计）" if not track['observed'] else "")]
                values.extend("({:.1f}, {:.1f}, {:.1f})".format(*sample[1:])
                              for sample in track['samples'])
                for column,value in enumerate(values):
                    cell=QTableWidgetItem(value)
                    if column==0 and not track['observed']:
                        cell.setForeground(QColor("#ffb74d"))
                        cell.setToolTip("目标当前被遮挡，位置和轨迹均为记忆外推")
                    self.prediction_table.setItem(index,column,cell)
    def check_dynamic_risk_age(self):
        if (self.risk_last_rx is not None and
                time.monotonic()-self.risk_last_rx>1.5 and
                not self.risk_stale_reported):
            self.risk_stale_reported=True
            self.latest_risk_snapshot=None
            self.dynamic_risk.setText("动态风险报告中断，状态未知")
            self.risk_summary.setText("风险报告中断，旧预测已隐藏")
            self.prediction_summary.setText("风险报告中断，旧轨迹已隐藏")
            self.risk_table.setRowCount(0);self.prediction_table.setRowCount(0)
            if self.event_journal.risk('STALE',time.time()):self.render_events()
    def perception_status_cb(self,msg):
        try:
            value=json.loads(msg.data);state=value.get("state","UNKNOWN")
            if state=="WARMING_UP":
                text="物理雷达背景学习 {}/{}".format(value.get("frame",0),value.get("warmup_frames",8))
            elif state=="TRACKING":
                text="物理雷达 | 点 {} | 原始簇 {} | 过滤 {} | 候选 {} | 确认 {} | 置信 {:.0%}".format(value.get("raw_points",0),value.get("raw_clusters",0),value.get("rejected_oversized",0),value.get("candidate_clusters",0),value.get("confirmed_targets",0),float(value.get("mean_confidence",0.0)))
            else:text="感知源 {} | {}".format(value.get("source","-"),state)
            self.perception_status.setText(text)
        except (TypeError,ValueError,KeyError):self.perception_status.setText("感知状态数据格式错误")
    def energy_return_cb(self,msg):
        try:
            advisory=validate_energy_advisory(json.loads(msg.data))
        except (TypeError,ValueError):
            self.energy_last_rx=None;self.energy_stale_reported=True
            self.latest_energy_snapshot=None
            self.energy_return.setText("能量返航数据格式错误，建议保持")
            self.energy_summary.setText("建议无效，旧数据已隐藏")
            self.energy_table.setRowCount(0);return
        self.energy_last_rx=time.monotonic();self.energy_stale_reported=False
        level=advisory['level'];rows=advisory['rows']
        self.latest_energy_snapshot=advisory
        names={"NORMAL":"正常","LOW":"低余量，建议返航","CRITICAL":"临界，建议备用点降落","STALE":"数据过期，建议保持"}
        text=names[level]+" | 影子模式（不下发控制）"
        if rows:text+=" | 最低预计落地余量 {:.1f} Wh".format(min(row['final_margin_wh'] for row in rows))
        self.energy_return.setText(text)
        self.energy_return.setStyleSheet("color:{}".format({"CRITICAL":"#c62828","LOW":"#ef6c00","NORMAL":"#2e7d32"}.get(level,"#607d8b")))
        if level=='STALE':
            self.energy_summary.setText("能量输入过期，返航建议不可用")
            self.energy_table.setRowCount(0);return
        self.energy_summary.setText("{} 架建议 · 能量数据年龄 {:.1f} s · 槽位和次序仅供参考".format(len(rows),advisory['age_s']))
        self.energy_table.setRowCount(len(rows))
        level_colors={"NORMAL":"#66bb6a","LOW":"#ffb74d","CRITICAL":"#ef5350"}
        for index,row in enumerate(rows):
            values=[row['vehicle_id'],{"NORMAL":"正常","LOW":"低余量","CRITICAL":"临界"}[row['level']],
                    "{:.1f} Wh".format(row['final_margin_wh']),
                    str(row['slot']) if row['slot'] is not None else "未分配",
                    "第 {} 位".format(row['landing_rank'])]
            for column,value in enumerate(values):
                cell=QTableWidgetItem(value)
                if column==1:cell.setForeground(QColor(level_colors[row['level']]))
                if column==2:cell.setToolTip("预计返航所需 {:.1f} Wh；落地余量为模型预测".format(row['required_to_land_wh']))
                self.energy_table.setItem(index,column,cell)
    def check_energy_advisory_age(self):
        if (self.energy_last_rx is not None and
                time.monotonic()-self.energy_last_rx>2.5 and
                not self.energy_stale_reported):
            self.energy_stale_reported=True
            self.latest_energy_snapshot=None
            self.energy_return.setText("能量返航建议已中断，建议保持")
            self.energy_summary.setText("返航建议中断，旧数据已隐藏")
            self.energy_table.setRowCount(0)
    def operator_snapshot_cb(self,msg):
        try:vehicles=validate_snapshot(json.loads(msg.data))
        except (TypeError,ValueError):
            self.latest_vehicle_snapshot=None
            self.vehicle_summary.setText("车辆遥测格式错误");self.vehicle_table.setRowCount(0);return
        self.telemetry_last_rx=time.monotonic();self.telemetry_stale_reported=False
        self.latest_vehicle_snapshot=vehicles
        active=sum(item["status"]=="ARMED" for item in vehicles)
        stale=sum(item["status"]=="STALE" for item in vehicles)
        self.vehicle_summary.setText("{} 架在线快照 · {} 架已解锁 · {} 架遥测过期".format(len(vehicles),active,stale))
        self.vehicle_table.setRowCount(len(vehicles))
        names={"ARMED":"飞行中","READY":"就绪","DISCONNECTED":"断连","STALE":"过期"}
        colors={"ARMED":"#66bb6a","READY":"#64b5f6","DISCONNECTED":"#ef5350","STALE":"#ffb74d"}
        for row,item in enumerate(vehicles):
            position=item.get("position_local_m")
            fraction=item.get("battery_fraction")
            energy=item.get("remaining_wh")
            age=item.get("state_age_s")
            values=[item["vehicle_id"],names[item["status"]],item.get("mode") or "-",
                    "{:.1f} m".format(position[2]) if position is not None else "-",
                    "{:.0f}% ({})".format(100*fraction,item.get("battery_source") or "-") if fraction is not None else "-",
                    "{:.1f} Wh".format(energy) if isinstance(energy,(int,float)) else "-",
                    "{:.1f} s".format(age) if isinstance(age,(int,float)) else "-"]
            for column,value in enumerate(values):
                cell=QTableWidgetItem(value)
                if column==1:cell.setForeground(QColor(colors[item["status"]]))
                if column==3 and position is not None:
                    cell.setToolTip("局部位置 x={:.1f}, y={:.1f}, z={:.1f} m".format(*position))
                self.vehicle_table.setItem(row,column,cell)
    def check_operator_snapshot_age(self):
        if (self.telemetry_last_rx is not None and
                time.monotonic()-self.telemetry_last_rx>3.0 and
                not self.telemetry_stale_reported):
            self.telemetry_stale_reported=True
            self.latest_vehicle_snapshot=None
            self.vehicle_summary.setText("车辆遥测汇总已中断，旧数据已隐藏")
            self.vehicle_table.setRowCount(0)
    def shutdown_plugin(self):
        self.analysis_timer.stop();self.analysis_timeout.stop();self.telemetry_timer.stop();self.energy_timer.stop();self.risk_timer.stop()
        for subscriber in self.ros_subscribers:subscriber.unregister()
        if self.analysis_process.state()!=QProcess.NotRunning:self.analysis_process.kill();self.analysis_process.waitForFinished(1000)
        self.stop_simulation()
