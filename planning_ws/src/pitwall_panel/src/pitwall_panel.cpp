#include "pitwall_panel/pitwall_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace pitwall_panel
{

PitwallPanel::PitwallPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout;

  // --- STATE 배너 ---
  state_banner_ = new QLabel("WAITING");
  state_banner_->setAlignment(Qt::AlignCenter);
  state_banner_->setMinimumHeight(44);
  state_banner_->setStyleSheet(
    "QLabel { background:#555; color:white; font-size:20px; font-weight:bold;"
    " border-radius:6px; }");
  root->addWidget(state_banner_);

  // --- Ego 제어 ---
  auto * ego_box = new QGroupBox("Ego");
  auto * ego_lay = new QVBoxLayout;
  ego_status_label_ = new QLabel("-");
  ego_lay->addWidget(ego_status_label_);
  auto * ego_btns = new QHBoxLayout;
  auto * btn_auto = new QPushButton("Autonomous");
  auto * btn_manual = new QPushButton("Manual (kb/joy)");
  ego_btns->addWidget(btn_auto);
  ego_btns->addWidget(btn_manual);
  ego_lay->addLayout(ego_btns);
  ego_box->setLayout(ego_lay);
  root->addWidget(ego_box);

  // --- 상대 차량 ---
  auto * opp_box = new QGroupBox("Opponent (2D Goal Pose = spawn)");
  auto * opp_lay = new QVBoxLayout;
  opp_status_label_ = new QLabel("-");
  opp_lay->addWidget(opp_status_label_);
  auto * mode_lay = new QHBoxLayout;
  mode_lay->addWidget(new QLabel("Mode:"));
  opp_mode_combo_ = new QComboBox;
  opp_mode_combo_->addItems({"path", "ftg", "manual"});
  mode_lay->addWidget(opp_mode_combo_, 1);
  opp_lay->addLayout(mode_lay);
  auto * opp_btns = new QHBoxLayout;
  auto * btn_minus = new QPushButton("Opp speed -0.5");
  auto * btn_plus = new QPushButton("Opp speed +0.5");
  opp_btns->addWidget(btn_minus);
  opp_btns->addWidget(btn_plus);
  opp_lay->addLayout(opp_btns);
  auto * btn_remove = new QPushButton("Remove opponent");
  opp_lay->addWidget(btn_remove);
  opp_box->setLayout(opp_lay);
  root->addWidget(opp_box);

  // --- 장애물 / 센서 / perception ---
  auto * sim_box = new QGroupBox("Sim utils");
  auto * sim_lay = new QVBoxLayout;
  auto * btn_clear_obs = new QPushButton("Clear static obstacles");
  sim_lay->addWidget(btn_clear_obs);
  auto * lidar_lay = new QHBoxLayout;
  ego_lidar_check_ = new QCheckBox("Ego LiDAR");
  ego_lidar_check_->setChecked(true);
  opp_lidar_check_ = new QCheckBox("Opp LiDAR");
  opp_lidar_check_->setChecked(true);
  lidar_lay->addWidget(ego_lidar_check_);
  lidar_lay->addWidget(opp_lidar_check_);
  sim_lay->addLayout(lidar_lay);
  auto * perc_lay = new QHBoxLayout;
  perc_lay->addWidget(new QLabel("Perception:"));
  perception_combo_ = new QComboBox;
  perception_combo_->addItems({"tracking_merge", "lidar_overlay"});
  perc_lay->addWidget(perception_combo_, 1);
  sim_lay->addLayout(perc_lay);
  sim_box->setLayout(sim_lay);
  root->addWidget(sim_box);

  // --- 이벤트 로그 ---
  auto * ev_box = new QGroupBox("Events (/pitwall/events)");
  auto * ev_lay = new QVBoxLayout;
  event_list_ = new QListWidget;
  event_list_->setMinimumHeight(120);
  ev_lay->addWidget(event_list_);
  auto * btn_clear_log = new QPushButton("Clear log");
  ev_lay->addWidget(btn_clear_log);
  ev_box->setLayout(ev_lay);
  root->addWidget(ev_box, 1);

  setLayout(root);

  // --- 버튼 연결 (publisher는 onInitialize 이후 유효 — 널 가드) ---
  connect(btn_auto, &QPushButton::clicked, this, [this]() {
      PublishString(pub_ego_mode_, "autonomous");
    });
  connect(btn_manual, &QPushButton::clicked, this, [this]() {
      PublishString(pub_ego_mode_, "manual");
    });
  connect(opp_mode_combo_, &QComboBox::currentTextChanged, this,
    [this](const QString & text) {
      PublishString(pub_opp_mode_, text.toStdString());
    });
  connect(btn_minus, &QPushButton::clicked, this, [this]() {
      if (pub_opp_delta_) {
        std_msgs::msg::Float32 m;
        m.data = -0.5F;
        pub_opp_delta_->publish(m);
      }
    });
  connect(btn_plus, &QPushButton::clicked, this, [this]() {
      if (pub_opp_delta_) {
        std_msgs::msg::Float32 m;
        m.data = 0.5F;
        pub_opp_delta_->publish(m);
      }
    });
  connect(btn_remove, &QPushButton::clicked, this, [this]() {
      if (pub_remove_opp_) {pub_remove_opp_->publish(std_msgs::msg::Empty());}
    });
  connect(btn_clear_obs, &QPushButton::clicked, this, [this]() {
      if (pub_clear_obs_) {pub_clear_obs_->publish(std_msgs::msg::Empty());}
    });
  connect(ego_lidar_check_, &QCheckBox::toggled, this, [this](bool on) {
      if (pub_ego_lidar_) {
        std_msgs::msg::Bool m;
        m.data = on;
        pub_ego_lidar_->publish(m);
      }
    });
  connect(opp_lidar_check_, &QCheckBox::toggled, this, [this](bool on) {
      if (pub_opp_lidar_) {
        std_msgs::msg::Bool m;
        m.data = on;
        pub_opp_lidar_->publish(m);
      }
    });
  connect(perception_combo_, &QComboBox::currentTextChanged, this,
    [this](const QString & text) {
      PublishString(pub_perception_, text.toStdString());
    });
  connect(btn_clear_log, &QPushButton::clicked, this, [this]() {
      event_list_->clear();
    });
}

void PitwallPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_ego_mode_ = node_->create_publisher<std_msgs::msg::String>("/sim/ego_mode", 10);
  pub_opp_mode_ = node_->create_publisher<std_msgs::msg::String>("/sim/opp_mode", 10);
  pub_opp_delta_ =
    node_->create_publisher<std_msgs::msg::Float32>("/sim/opp_speed_delta", 10);
  pub_remove_opp_ =
    node_->create_publisher<std_msgs::msg::Empty>("/sim/remove_opponent", 10);
  pub_clear_obs_ =
    node_->create_publisher<std_msgs::msg::Empty>("/sim/clear_obstacles", 10);
  pub_ego_lidar_ =
    node_->create_publisher<std_msgs::msg::Bool>("/sim/ego_lidar_enable", 10);
  pub_opp_lidar_ =
    node_->create_publisher<std_msgs::msg::Bool>("/sim/opp_lidar_enable", 10);
  pub_perception_ =
    node_->create_publisher<std_msgs::msg::String>("/sim/perception_mode", 10);

  sub_events_ = node_->create_subscription<std_msgs::msg::String>(
    "/pitwall/events", 50,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_events_.push_back(msg->data);
      // STATE 배너용 복구 상태 추적 (H2H 상태머신 전까지 이벤트에서 유도)
      if (msg->data.find("복구 시작") != std::string::npos) {
        in_recovery_ = true;
      } else if (msg->data.find("복구 완료") != std::string::npos ||
        msg->data.find("모드 전환") != std::string::npos)
      {
        in_recovery_ = false;
      }
    });
  sub_ego_status_ = node_->create_subscription<std_msgs::msg::String>(
    "/sim/ego_status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      ego_status_ = msg->data;
    });
  sub_opp_status_ = node_->create_subscription<std_msgs::msg::String>(
    "/sim/opp_status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      opp_status_ = msg->data;
    });
  sub_otwpnts_ = node_->create_subscription<f1tenth_planning_msgs::msg::WpntArray>(
    "/planner/avoidance/otwpnts", 10,
    [this](const f1tenth_planning_msgs::msg::WpntArray::SharedPtr msg) {
      if (!msg->wpnts.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_otwpnts_ = node_->now();
      }
    });
  sub_pred_ = node_->create_subscription<f1tenth_planning_msgs::msg::ObstacleArray>(
    "/opponent_prediction/obstacles", 10,
    [this](const f1tenth_planning_msgs::msg::ObstacleArray::SharedPtr msg) {
      if (!msg->obstacles.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_pred_ = node_->now();
      }
    });

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &PitwallPanel::RefreshUi);
  refresh_timer_->start(250);
}

void PitwallPanel::PublishString(
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub,
  const std::string & data)
{
  if (pub) {
    std_msgs::msg::String msg;
    msg.data = data;
    pub->publish(msg);
  }
}

void PitwallPanel::RefreshUi()
{
  std::deque<std::string> events;
  std::string ego_status;
  std::string opp_status;
  bool recovery = false;
  bool overtake = false;
  bool trailing = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    events.swap(pending_events_);
    ego_status = ego_status_;
    opp_status = opp_status_;
    recovery = in_recovery_;
    const auto now = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    overtake = (now - last_otwpnts_).seconds() < 0.7;
    trailing = (now - last_pred_).seconds() < 0.7;
  }

  // 이벤트 로그 (최근 200줄, 자동 스크롤)
  for (const auto & e : events) {
    event_list_->addItem(QString::fromStdString(e));
  }
  while (event_list_->count() > 200) {
    delete event_list_->takeItem(0);
  }
  if (!events.empty()) {
    event_list_->scrollToBottom();
  }

  ego_status_label_->setText(QString::fromStdString(
      ego_status.empty() ? "-" : ego_status));
  opp_status_label_->setText(QString::fromStdString(
      opp_status.empty() ? "-" : opp_status));

  // STATE 배너: manual > RECOVERY > OVERTAKE > TRAILING > GB_TRACK
  // (H2H 상태머신이 생기면 그 상태 토픽 구독으로 교체)
  QString state;
  QString color;
  if (ego_status.find("manual") != std::string::npos) {
    state = "MANUAL";
    color = "#607d8b";
  } else if (recovery) {
    state = "RECOVERY";
    color = "#d32f2f";
  } else if (overtake) {
    state = "OVERTAKE";
    color = "#7b1fa2";
  } else if (trailing) {
    state = "TRAILING";
    color = "#f9a825";
  } else if (!ego_status.empty()) {
    state = "GB_TRACK";
    color = "#2e7d32";
  } else {
    state = "WAITING";
    color = "#555";
  }
  state_banner_->setText(state);
  state_banner_->setStyleSheet(
    "QLabel { background:" + color +
    "; color:white; font-size:20px; font-weight:bold; border-radius:6px; }");
}

}  // namespace pitwall_panel

PLUGINLIB_EXPORT_CLASS(pitwall_panel::PitwallPanel, rviz_common::Panel)
