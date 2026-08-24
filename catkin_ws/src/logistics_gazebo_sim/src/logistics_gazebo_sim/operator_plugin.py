import json
import os
import shutil
import signal
import socket
import rospy
import rospkg
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from logistics_gazebo_sim.scenes import SCENES, SCALE, metric_xy
from python_qt_binding.QtCore import QObject, QProcess, QProcessEnvironment, Qt, QTimer, Signal
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QMessageBox, QProgressBar, QPushButton, QVBoxLayout, QWidget)
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

class OperatorPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context); self.setObjectName("LogisticsOperator")
        self.widget=QWidget();self.widget.setWindowTitle("无人机物流仿真上位机");self.widget.setMinimumWidth(680);root=QVBoxLayout(self.widget);root.setContentsMargins(16,14,16,16);root.setSpacing(10)
        package_path=rospkg.RosPack().get_path("logistics_gazebo_sim")
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
        self.avoidance_mode=QComboBox();self.avoidance_mode.addItem("整队偏移（稳定闭环）",("collective_offset","shadow",True));self.avoidance_mode.addItem("3D ORCA（影子模式）",("orca3d","shadow",False));self.avoidance_mode.addItem("分布式 MPC（影子模式）",("distributed_mpc","shadow",False));self.avoidance_mode.addItem("3D ORCA（受限接管）",("orca3d","limited",True));form.addRow("局部避障模式",self.avoidance_mode)
        simrow=QHBoxLayout();self.start_sim=QPushButton("\u89c4\u5212\u5e76\u542f\u52a8\u4e09\u673a\u4eff\u771f");self.stop_sim=QPushButton("\u505c\u6b62\u4eff\u771f");simrow.addWidget(self.start_sim);simrow.addWidget(self.stop_sim);form.addRow(simrow);root.addWidget(box)
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
        status=QGroupBox("\u72b6\u6001");sf=QFormLayout(status);self.state=QLabel("\u672a\u542f\u52a8");self.stage=QLabel("-");self.safety=QLabel("\u7b49\u5f85\u6570\u636e");self.dynamic_risk=QLabel("等待动态障碍数据")
        self.state.setObjectName("statusBadge");self.stage.setObjectName("statusBadge")
        self.progress=QProgressBar();self.progress.setRange(0,1000);self.progress.setValue(0);self.progress.setFormat("%p%")
        sf.addRow("任务",self.state);sf.addRow("阶段",self.stage);sf.addRow("任务进度",self.progress);sf.addRow("静态安全",self.safety);sf.addRow("动态风险",self.dynamic_risk);root.addWidget(status);root.addStretch();context.add_widget(self.widget)
        self.process=QProcess(self.widget);self.analysis_process=QProcess(self.widget)
        self.simulation_stop_requested=False;self.simulation_start_pending=False
        self.process.started.connect(self.simulation_process_started)
        self.process.errorOccurred.connect(self.simulation_process_error)
        self.process.finished.connect(self.simulation_process_finished)
        self.analysis_timer=QTimer(self.widget);self.analysis_timer.setSingleShot(True);self.analysis_timer.setInterval(700);self.analysis_timer.timeout.connect(self.start_analysis)
        self.analysis_timeout=QTimer(self.widget);self.analysis_timeout.setSingleShot(True);self.analysis_timeout.setInterval(20000);self.analysis_timeout.timeout.connect(self.analysis_timed_out)
        self.analysis_process.finished.connect(self.analysis_finished)
        self.valid_analysis_signature=None;self.analysis_running_signature=None;self.analysis_mission=None;self.analysis_report=None;self.analysis_retry=0
        self.pick_mode="start";self.point_bridge=PointBridge();self.point_bridge.point_received.connect(self.apply_clicked_point)
        self.scene.currentIndexChanged.connect(self.update_defaults);self.start_sim.clicked.connect(self.launch_sim);self.stop_sim.clicked.connect(self.stop_simulation)
        self.pick_start.clicked.connect(lambda:self.begin_pick("start"));self.pick_goal.clicked.connect(lambda:self.begin_pick("goal"))
        self.start.clicked.connect(lambda:self.call("/fleet_mission_player/start"));self.pause.clicked.connect(lambda:self.call("/fleet_mission_player/pause"));self.resume.clicked.connect(lambda:self.call("/fleet_mission_player/resume"));self.reset.clicked.connect(lambda:self.call("/fleet_mission_player/reset"));self.land.clicked.connect(lambda:self.call("/fleet_mission_player/land"))
        self.preview_pub=rospy.Publisher("/operator/preview_markers",MarkerArray,queue_size=1,latch=True);rospy.Subscriber("/clicked_point",PointStamped,self.clicked_point_cb,queue_size=1)
        for spin in (self.start_x,self.start_y,self.goal_x,self.goal_y,self.altitude):spin.valueChanged.connect(self.parameters_changed)
        self.formation.currentIndexChanged.connect(self.parameters_changed);self.avoidance_mode.currentIndexChanged.connect(self.parameters_changed)
        rospy.Subscriber("/fleet/mission_state",String,self.state_cb,queue_size=1);rospy.Subscriber("/fleet/diagnostics",DiagnosticArray,self.diag_cb,queue_size=1)
        rospy.Subscriber("/fleet/dynamic_risk",String,self.dynamic_risk_cb,queue_size=1)
        self.update_defaults()
    def update_defaults(self):
        sx,sy,gx,gy,alt=SCENE_DEFAULTS[self.scene.currentData()]
        for widget,value in ((self.start_x,sx),(self.start_y,sy),(self.goal_x,gx),(self.goal_y,gy),(self.altitude,alt)):widget.setValue(value)
        QTimer.singleShot(50,self.publish_preview);self.schedule_analysis()
    def parameter_signature(self):
        return (int(self.scene.currentData()),round(self.start_x.value(),3),
                round(self.start_y.value(),3),round(self.goal_x.value(),3),
                round(self.goal_y.value(),3),round(self.altitude.value(),3),
                str(self.formation.currentData()),str(self.avoidance_mode.currentData()))
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
                self.analysis_detail.setText(detail[:320]);QTimer.singleShot(150,self.start_analysis);return
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
            self.analysis_state.setStyleSheet("color:#ef5350;border-color:#8d3434;");return
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
            self.analysis_detail.setText("无法读取规划摘要：{}".format(exc));return
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
        sid=self.scene.currentData();mission=self.analysis_mission
        algorithm,orca_mode,execution=self.avoidance_mode.currentData()
        self.cleanup_px4_sockets();args=["logistics_gazebo_sim","three_uav_mission.launch","gui:=true","auto_start:=false","dynamic_obstacles:={}".format(str(self.dynamic_enabled.isChecked()).lower()),"dynamic_avoidance_execution:={}".format(str(execution).lower()),"local_avoidance_algorithm:={}".format(algorithm),"orca_control_mode:={}".format(orca_mode),"scene_id:={}".format(sid),"spawn_x:={}".format(self.start_x.value()),"spawn_y:={}".format(self.start_y.value()),"goal_x:={}".format(self.goal_x.value()),"goal_y:={}".format(self.goal_y.value()),"target_z:={}".format(self.altitude.value()),"mission_config:={}".format(mission),"gazebo_master_uri:=http://127.0.0.1:11460"]
        self.simulation_stop_requested=False;self.simulation_start_pending=True;self.start_sim.setEnabled(False)
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
    def simulation_process_started(self):
        self.state.setText("启动命令已提交，正在等待 Gazebo 与三机 PX4 就绪…")
    def simulation_process_error(self,_error):
        self.simulation_start_pending=False
        self.start_sim.setEnabled(self.valid_analysis_signature==self.parameter_signature())
        self.state.setText("仿真启动失败")
        QMessageBox.critical(self.widget,"仿真启动失败",
            "无法启动 roslaunch：{}\n请检查 ROS 环境和启动日志。".format(self.process.errorString()))
    def simulation_process_finished(self,exit_code,_exit_status):
        self.simulation_start_pending=False
        self.start_sim.setEnabled(self.valid_analysis_signature==self.parameter_signature())
        if self.simulation_stop_requested:
            self.state.setText("三机仿真已停止")
        elif exit_code!=0:
            self.state.setText("仿真异常退出（代码 {}）".format(exit_code))
            QMessageBox.warning(self.widget,"仿真异常退出",
                "Gazebo/PX4 启动进程已退出，返回代码 {}。\n请查看 roslaunch 日志。".format(exit_code))
        else:self.state.setText("三机仿真已结束")
        self.simulation_stop_requested=False
    def stop_simulation(self):
        if self.process.state()==QProcess.NotRunning:
            result=QProcess.execute("pkill",["-INT","-f",
                "[r]oslaunch.*three_uav_mission.launch"])
            if result==0:
                self.state.setText("正在停止外部三机仿真…")
                QTimer.singleShot(5000,self.force_stop_external_simulation)
            else:
                self.state.setText("没有检测到运行中的三机仿真")
            return
        self.simulation_stop_requested=True;self.state.setText("正在停止三机仿真…")
        try:os.killpg(int(self.process.processId()),signal.SIGINT)
        except (OSError,ProcessLookupError):self.process.terminate()
        QTimer.singleShot(5000,self.force_stop_simulation)
    def force_stop_simulation(self):
        if self.process.state()==QProcess.NotRunning:return
        try:os.killpg(int(self.process.processId()),signal.SIGTERM)
        except (OSError,ProcessLookupError):self.process.kill()
    def force_stop_external_simulation(self):
        if QProcess.execute("pgrep",["-f",
                "[r]oslaunch.*three_uav_mission.launch"])==0:
            QProcess.execute("pkill",["-TERM","-f",
                "[r]oslaunch.*three_uav_mission.launch"])
        else:self.state.setText("三机仿真已停止")
    def call(self,name):
        try:r=rospy.ServiceProxy(name,Trigger)();self.state.setText(r.message)
        except rospy.ServiceException as exc:QMessageBox.warning(self.widget,"\u670d\u52a1\u8c03\u7528\u5931\u8d25",str(exc))
    def state_cb(self,msg):
        try:
            d=json.loads(msg.data)
            if self.simulation_start_pending:
                self.simulation_start_pending=False
                QMessageBox.information(self.widget,"仿真启动成功",
                    "Gazebo、三机 PX4 与任务节点已连接，可以开始任务。")
            self.state.setText(d["state"]);ph={"TAKEOFF_HOLD":"\u8d77\u98de\u7b49\u5f85","DEPARTURE_FORMATION":"\u51fa\u53d1\u7f16\u961f","OUTBOUND":"\u524d\u5f80\u914d\u9001\u70b9","DELIVERY_FORMATION":"\u6295\u9012\u4e00\u5b57\u7f16\u961f","DELIVERY_DESCENT":"\u6295\u9012\u4e0b\u964d","DELIVERY_RELEASE":"\u8d27\u7269\u6295\u9012","DELIVERY_ASCENT":"\u6295\u9012\u56de\u5347","CRUISE_REFORMATION":"\u6062\u590d\u5de1\u822a\u961f\u5f62","RETURN":"\u8fd4\u822a","HOME_FORMATION":"\u8fd4\u822a\u7f16\u961f","HOME_DESCENT":"\u8d77\u70b9\u964d\u843d"};self.stage.setText("{} - {}".format(d["stage"],ph.get(d.get("phase"),"-")))
            self.progress.setValue(int(1000*d["progress"]))
        except Exception:pass
    def diag_cb(self,msg):
        if not msg.status:return
        s=msg.status[0];v={x.key:x.value for x in s.values};self.safety.setText("{} | \u95f4\u8ddd {}m | \u51c0\u7a7a {}m | \u8bef\u5dee {}m".format(s.message,v.get("min_separation_m","-"),v.get("min_obstacle_clearance_m","-"),v.get("max_tracking_error_m","-")));self.safety.setStyleSheet("color:{}".format("#c62828" if s.level>=2 else "#ef6c00" if s.level==1 else "#2e7d32"))
    def dynamic_risk_cb(self,msg):
        try:
            value=json.loads(msg.data);level=value.get("level","STALE")
            if level=="STALE":text=value.get("message","等待动态障碍数据")
            else:text="{} | 最近 {} | 净空 {}m | 冲突倒计时 {}s".format(level,value.get("nearest_vehicle","-"),value.get("minimum_clearance_m","-"),value.get("time_to_conflict_s","无"))
            avoidance=value.get("avoidance") or {}
            algorithm=avoidance.get("algorithm") or value.get("local_avoidance_algorithm")
            if algorithm:text+=" | 算法 {}".format(algorithm)
            if avoidance.get("viable") and avoidance.get("command_type")=="per_vehicle_velocity":
                text+=" | ORCA速度建议 {} 架（{}）".format(len(avoidance.get("commands") or []),"受限接管" if self.avoidance_mode.currentData()[1]=="limited" else "影子模式")
            elif avoidance.get("viable") and avoidance.get("command_type")=="per_vehicle_trajectory":
                timing=avoidance.get("solve_time_ms") or {};text+=" | MPC影子轨迹 {} 架 | 求解 {} ms | 暖启动 {} 架".format(len(avoidance.get("trajectories") or []),timing.get("total","-"),avoidance.get("warm_started_vehicle_count",0))
            elif avoidance.get("viable"):text+=" | 建议整队偏移 {}".format(avoidance.get("selected_offset"))
            elif level in ("WARNING","CRITICAL"):
                summary=avoidance.get("rejection_summary") or {};names={"DYNAMIC_CONFLICT":"动态冲突","DYNAMIC_CLEARANCE":"动态净空不足","MPC_SOLVER_FAILURE":"MPC 求解失败","MPC_SOLVER_TIMEOUT":"MPC 求解超时","E_BOUNDARY":"越界","E_VERTICAL_CLEARANCE":"高度违规","E_CORRIDOR_TOO_NARROW":"建筑净空不足"}
                reasons="、".join("{}×{}".format(names.get(key,key),count) for key,count in sorted(summary.items()))
                text+=" | 无安全候选，保持悬停"+("（{}）".format(reasons) if reasons else "")
            self.dynamic_risk.setText(text)
            self.dynamic_risk.setStyleSheet("color:{}".format({"CRITICAL":"#c62828","WARNING":"#ef6c00","SAFE":"#2e7d32"}.get(level,"#607d8b")))
        except (TypeError,ValueError):self.dynamic_risk.setText("动态风险数据格式错误")
    def shutdown_plugin(self):
        self.analysis_timer.stop();self.analysis_timeout.stop()
        if self.analysis_process.state()!=QProcess.NotRunning:self.analysis_process.kill();self.analysis_process.waitForFinished(1000)
        self.stop_simulation()
