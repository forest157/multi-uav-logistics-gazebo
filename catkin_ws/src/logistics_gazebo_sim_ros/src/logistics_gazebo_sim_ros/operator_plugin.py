import json
import os
import subprocess
import signal
import rospy
import rospkg
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from logistics_gazebo_sim_ros.scenes import SCENES, SCALE, metric_xy
from python_qt_binding.QtCore import QObject, QProcess, Qt, QTimer, Signal
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import (QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
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
        package_path=rospkg.RosPack().get_path("logistics_gazebo_sim_ros")
        header=QHBoxLayout();header.setSpacing(14)
        logo=QLabel();logo.setObjectName("brandLogo");logo.setFixedSize(84,84);logo.setAlignment(Qt.AlignCenter)
        logo_path=os.path.join(package_path,"resources","nuaa.jpg");pixmap=QPixmap(logo_path)
        if pixmap.isNull():
            logo.setText("NUAA");rospy.logwarn("Unable to load rqt branding image: %s",logo_path)
        else:
            logo.setPixmap(pixmap.scaled(80,80,Qt.KeepAspectRatio,Qt.SmoothTransformation))
        heading=QVBoxLayout();heading.setSpacing(3)
        title=QLabel("三机物流任务控制台 · ROS OMPL 3D 实验版");title.setObjectName("title");subtitle=QLabel("场景预览 · 地图选点 · 低空避障 · 编队往返");subtitle.setObjectName("subtitle")
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
        simrow=QHBoxLayout();self.start_sim=QPushButton("\u89c4\u5212\u5e76\u542f\u52a8\u4e09\u673a\u4eff\u771f");self.stop_sim=QPushButton("\u505c\u6b62\u4eff\u771f");simrow.addWidget(self.start_sim);simrow.addWidget(self.stop_sim);form.addRow(simrow);root.addWidget(box)
        self.start_sim.setObjectName("primary");self.stop_sim.setObjectName("secondary");self.start_sim.setToolTip("先校验参数并规划安全航线，再启动 Gazebo/PX4")
        mission=QGroupBox("\u4efb\u52a1\u63a7\u5236");row=QHBoxLayout(mission)
        self.start=QPushButton("\u5f00\u59cb");self.pause=QPushButton("\u6682\u505c");self.resume=QPushButton("\u7ee7\u7eed");self.reset=QPushButton("\u91cd\u7f6e\u4efb\u52a1");self.land=QPushButton("\u7d27\u6025\u964d\u843d")
        self.start.setObjectName("primary");self.land.setObjectName("danger");self.pause.setObjectName("secondary");self.resume.setObjectName("secondary");self.reset.setObjectName("secondary")
        for button in (self.start,self.pause,self.resume,self.reset,self.land):row.addWidget(button)
        root.addWidget(mission)
        status=QGroupBox("\u72b6\u6001");sf=QFormLayout(status);self.state=QLabel("\u672a\u542f\u52a8");self.stage=QLabel("-");self.safety=QLabel("\u7b49\u5f85\u6570\u636e")
        self.state.setObjectName("statusBadge");self.stage.setObjectName("statusBadge")
        self.progress=QProgressBar();self.progress.setRange(0,1000);self.progress.setValue(0);self.progress.setFormat("%p%")
        sf.addRow("任务",self.state);sf.addRow("阶段",self.stage);sf.addRow("任务进度",self.progress);sf.addRow("安全状态",self.safety);root.addWidget(status);root.addStretch();context.add_widget(self.widget)
        self.process=QProcess(self.widget);self.pick_mode="start";self.point_bridge=PointBridge();self.point_bridge.point_received.connect(self.apply_clicked_point)
        self.scene.currentIndexChanged.connect(self.update_defaults);self.start_sim.clicked.connect(self.launch_sim);self.stop_sim.clicked.connect(self.stop_simulation)
        self.pick_start.clicked.connect(lambda:self.begin_pick("start"));self.pick_goal.clicked.connect(lambda:self.begin_pick("goal"))
        self.start.clicked.connect(lambda:self.call("/fleet_mission_player/start"));self.pause.clicked.connect(lambda:self.call("/fleet_mission_player/pause"));self.resume.clicked.connect(lambda:self.call("/fleet_mission_player/resume"));self.reset.clicked.connect(lambda:self.call("/fleet_mission_player/reset"));self.land.clicked.connect(lambda:self.call("/fleet_mission_player/land"))
        self.preview_pub=rospy.Publisher("/operator/preview_markers",MarkerArray,queue_size=1,latch=True);rospy.Subscriber("/clicked_point",PointStamped,self.clicked_point_cb,queue_size=1)
        for spin in (self.start_x,self.start_y,self.goal_x,self.goal_y):spin.valueChanged.connect(lambda _v:self.publish_preview())
        rospy.Subscriber("/fleet/mission_state",String,self.state_cb,queue_size=1);rospy.Subscriber("/fleet/diagnostics",DiagnosticArray,self.diag_cb,queue_size=1)
        self.update_defaults()
    def update_defaults(self):
        sx,sy,gx,gy,alt=SCENE_DEFAULTS[self.scene.currentData()]
        for widget,value in ((self.start_x,sx),(self.start_y,sy),(self.goal_x,gx),(self.goal_y,gy),(self.altitude,alt)):widget.setValue(value)
        QTimer.singleShot(50,self.publish_preview)
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
        sid=self.scene.currentData();pkg=rospkg.RosPack().get_path("logistics_gazebo_sim_ros");mission="/tmp/logistics_custom_mission_{}.yaml".format(os.getpid())
        cmd=[os.path.join(pkg,"scripts","generate_missions"),"--scene",str(sid),"--start-x",str(self.start_x.value()),"--start-y",str(self.start_y.value()),"--goal-x",str(self.goal_x.value()),"--goal-y",str(self.goal_y.value()),"--altitude",str(self.altitude.value()),"--formation",self.formation.currentData(),"--output",mission]
        try:
            result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,universal_newlines=True,timeout=15)
        except (OSError,subprocess.TimeoutExpired) as exc:QMessageBox.critical(self.widget,"\u89c4\u5212\u5931\u8d25","\u65e0\u6cd5\u8fd0\u884c\u822a\u7ebf\u89c4\u5212\u5668\uff1a{}".format(exc));return
        if result.returncode or not os.path.isfile(mission):QMessageBox.warning(self.widget,"\u89c4\u5212\u5931\u8d25",self.planning_error(result.stderr.strip() or result.stdout.strip()));return
        self.cleanup_px4_sockets();args=["logistics_gazebo_sim_ros","three_uav_mission.launch","gui:=true","auto_start:=false","scene_id:={}".format(sid),"spawn_x:={}".format(self.start_x.value()),"spawn_y:={}".format(self.start_y.value()),"goal_x:={}".format(self.goal_x.value()),"goal_y:={}".format(self.goal_y.value()),"target_z:={}".format(self.altitude.value()),"mission_config:={}".format(mission),"gazebo_master_uri:=http://127.0.0.1:11460"]
        self.process.start("setsid",["roslaunch"]+args);self.state.setText("\u89c4\u5212\u6210\u529f\uff0c\u4eff\u771f\u542f\u52a8\u4e2d")
    def stop_simulation(self):
        if self.process.state()==QProcess.NotRunning:return
        try:os.killpg(int(self.process.processId()),signal.SIGINT)
        except (OSError,ProcessLookupError):self.process.terminate()
        QTimer.singleShot(5000,self.force_stop_simulation)
    def force_stop_simulation(self):
        if self.process.state()==QProcess.NotRunning:return
        try:os.killpg(int(self.process.processId()),signal.SIGTERM)
        except (OSError,ProcessLookupError):self.process.kill()
    def call(self,name):
        try:r=rospy.ServiceProxy(name,Trigger)();self.state.setText(r.message)
        except rospy.ServiceException as exc:QMessageBox.warning(self.widget,"\u670d\u52a1\u8c03\u7528\u5931\u8d25",str(exc))
    def state_cb(self,msg):
        try:
            d=json.loads(msg.data);self.state.setText(d["state"]);ph={"TAKEOFF_HOLD":"\u8d77\u98de\u7b49\u5f85","DEPARTURE_FORMATION":"\u51fa\u53d1\u7f16\u961f","OUTBOUND":"\u524d\u5f80\u914d\u9001\u70b9","DELIVERY_FORMATION":"\u6295\u9012\u4e00\u5b57\u7f16\u961f","DELIVERY_DESCENT":"\u6295\u9012\u4e0b\u964d","DELIVERY_RELEASE":"\u8d27\u7269\u6295\u9012","DELIVERY_ASCENT":"\u6295\u9012\u56de\u5347","CRUISE_REFORMATION":"\u6062\u590d\u5de1\u822a\u961f\u5f62","RETURN":"\u8fd4\u822a","HOME_FORMATION":"\u8fd4\u822a\u7f16\u961f","HOME_DESCENT":"\u8d77\u70b9\u964d\u843d"};self.stage.setText("{} - {}".format(d["stage"],ph.get(d.get("phase"),"-")))
            self.progress.setValue(int(1000*d["progress"]))
        except Exception:pass
    def diag_cb(self,msg):
        if not msg.status:return
        s=msg.status[0];v={x.key:x.value for x in s.values};self.safety.setText("{} | \u95f4\u8ddd {}m | \u51c0\u7a7a {}m | \u8bef\u5dee {}m".format(s.message,v.get("min_separation_m","-"),v.get("min_obstacle_clearance_m","-"),v.get("max_tracking_error_m","-")));self.safety.setStyleSheet("color:{}".format("#c62828" if s.level>=2 else "#ef6c00" if s.level==1 else "#2e7d32"))
    def shutdown_plugin(self):self.stop_simulation()
